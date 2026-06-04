/*
 * main.c — Open-loop motor drive, STM32G474RE (NUCLEO-G474RE)
 *
 * g_mode selects the control scheme live:
 *   0 = FOC (sinusoidal, open-loop voltage injection via Park/Clarke/SVPWM)
 *   1 = 6-step (trapezoidal commutation)
 *
 * Binary telemetry frame (921600 baud, 50 Hz):
 *   [0xAA][0x55][Ia:i16][Ib:i16][Ic:i16][Vbus:u16][theta_or_step:u16]
 *   [omega:i16][AmbT:i16][PhT:i16][fault:u8][0x55]  — 20 bytes total
 *
 *   Scales: currents *100 -> cA, Vbus *100 -> cV,
 *           FOC: theta *10000 -> 0..62831 rad*10000
 *           6-step: theta_or_step = 0..5
 *           omega *10 -> 0.1 rad/s, temps *10 -> 0.1 °C
 */

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "hal/hal_init.h"
#include "hal/hal_pwm.h"
#include "hal/hal_uart.h"
#include "hal/hal_spi.h"
#include "hal/hal_adc.h"
#include "control/angle_ramp.h"
#include "control/transforms.h"
#include "control/svpwm.h"
#include "control/commutator.h"
#include "drv835x.h"

/* ------------------------------------------------------------------ */
/* Tuning knobs — edit live in debugger / Live Expressions             */
/* ------------------------------------------------------------------ */
volatile uint32_t g_mode     = 0u;     /* 0 = FOC, 1 = 6-step */
volatile uint32_t g_enable   = 1u;     /* 0 = disabled, 1 = enabled */
volatile float    g_freqHz   = 30.0f;  /* electrical Hz */
volatile float    g_Vdc      = 12.0f;  /* actual bus voltage */

/* FOC knobs */
volatile float    g_Vd       = 2.0f;   /* d-axis voltage */
volatile float    g_Vq       = 0.0f;   /* q-axis voltage */

/* 6-step knobs */
volatile float    g_duty       = 0.3f;   /* running duty [0..1] */
volatile float    g_align_duty = 0.15f;  /* alignment duty [0..1] */

/* ------------------------------------------------------------------ */
/* Telemetry                                                           */
/* ------------------------------------------------------------------ */
volatile float    g_theta    = 0.0f;   /* FOC: electrical angle (rad) */
volatile uint8_t  g_step     = 0u;     /* 6-step: commutation step 0..5 */
volatile float    g_omega    = 0.0f;   /* electrical angular velocity (rad/s) */
volatile uint32_t g_isrCount = 0u;

/* ------------------------------------------------------------------ */
/* Binary frame packing                                                */
/* ------------------------------------------------------------------ */
#define TELEM_SYNC0  0xAAu
#define TELEM_SYNC1  0x55u
#define TELEM_TAIL   0x55u
#define TELEM_LEN    20u

static void pack_be16(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)((uint16_t)v >> 8);
    buf[1] = (uint8_t)((uint16_t)v & 0xFFu);
}

static void pack_beu16(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)(v >> 8);
    buf[1] = (uint8_t)(v & 0xFFu);
}

static int16_t clamp16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/* ------------------------------------------------------------------ */
/* Control state                                                       */
/* ------------------------------------------------------------------ */
#define CONTROL_TS   (1.0f / 20000.0f)  /* 50 µs ISR period */

/* FOC: ramp from 0 to g_freqHz over 2 s */
#define RAMP_TICKS   40000u

/* 6-step: hold step 0 for 1 s to align rotor before commutating */
#define ALIGN_TICKS  20000u

/* FOC state */
static AngleRamp_t  s_ramp;
static uint32_t     s_rampTick   = 0u;

/* 6-step state */
static Commutator_t s_comm;
static uint32_t     s_alignTick  = 0u;
static uint8_t      s_aligning   = 0u;

/* Shared */
static uint32_t     s_enablePrev = 0u;
static uint32_t     s_modePrev   = 0u;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static void restore_all_ccer(void)
{
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
               | TIM_CCER_CC2E | TIM_CCER_CC2NE
               | TIM_CCER_CC3E | TIM_CCER_CC3NE;
}

static void init_foc(void)
{
    s_rampTick = 0u;
    AngleRamp_init(&s_ramp, 0.0f, CONTROL_TS);
    g_theta = 0.0f;
    restore_all_ccer();
}

static void init_sixstep(void)
{
    Commutator_init(&s_comm);
    s_alignTick = 0u;
    s_aligning  = 1u;
}

/* ------------------------------------------------------------------ */
/* TIM1 Update ISR — 20 kHz                                           */
/* ------------------------------------------------------------------ */
void TIM1_UP_TIM16_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;
    g_isrCount++;

    ADC_ConvertCurrents();

    uint32_t en   = g_enable;
    uint32_t mode = g_mode;

    /* Enable/disable transitions */
    if (en != s_enablePrev)
    {
        if (en)
        {
            if (mode == 0u) init_foc();
            else            init_sixstep();
            PWM_EnableOutput();
        }
        else
        {
            PWM_DisableOutput();
            s_aligning = 0u;
            restore_all_ccer();
        }
        s_enablePrev = en;
        s_modePrev   = mode;
    }

    /* Live mode switch while running — restart cleanly */
    if (en && (mode != s_modePrev))
    {
        PWM_DisableOutput();
        restore_all_ccer();
        if (mode == 0u) init_foc();
        else            init_sixstep();
        PWM_EnableOutput();
        s_modePrev = mode;
    }

    if (!en)
        return;

    /* ---- FOC -------------------------------------------------------- */
    if (mode == 0u)
    {
        float freq = (s_rampTick < RAMP_TICKS)
                   ? g_freqHz * ((float)s_rampTick / (float)RAMP_TICKS)
                   : g_freqHz;
        if (s_rampTick < RAMP_TICKS) s_rampTick++;
        AngleRamp_setFreq(&s_ramp, freq);

        float theta_prev = g_theta;
        float theta = AngleRamp_update(&s_ramp);
        g_theta = theta;

        float dtheta = theta - theta_prev;
        if      (dtheta >  3.14159f) dtheta -= 6.28318f;
        else if (dtheta < -3.14159f) dtheta += 6.28318f;
        g_omega = dtheta * 20000.0f;

        float Valpha, Vbeta;
        ParkInv(g_Vd, g_Vq, theta, &Valpha, &Vbeta);

        float Va, Vb, Vc;
        ClarkeInv(Valpha, Vbeta, &Va, &Vb, &Vc);

        float dutyA, dutyB, dutyC;
        float vdc = g_Vdc < 1.0f ? 1.0f : g_Vdc;
        SVPWM_calc(Va, Vb, Vc, vdc, &dutyA, &dutyB, &dutyC);
        PWM_SetDuty(dutyA, dutyB, dutyC);
        g_step = 0u;
    }
    /* ---- 6-step ----------------------------------------------------- */
    else
    {
        if (s_aligning)
        {
            PWM_SetStep(0u, g_align_duty);
            g_step  = 0u;
            g_omega = 0.0f;
            g_theta = 0.0f;
            if (++s_alignTick >= ALIGN_TICKS)
                s_aligning = 0u;
            return;
        }

        float prev_angle = s_comm.angle;
        Commutator_update(&s_comm, g_freqHz, CONTROL_TS);

        float dtheta = s_comm.angle - prev_angle;
        if (dtheta < 0.0f) dtheta += 6.28318f;
        g_omega = dtheta * 20000.0f;
        g_theta = s_comm.angle;
        g_step  = s_comm.step;
        PWM_SetStep(s_comm.step, g_duty);
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */
int main(void)
{
    uint32_t last_telem = 0u;
    DRV835x_Status_t drv;

    SystemClock_Config();
    GPIO_Init();
    TIM1_Init();
    UART_Init();
    SPI_Init();
    DRV835x_Configure();
    ADC_Init();
    DRV835x_CalibrateCSA();

    AngleRamp_init(&s_ramp, 0.0f, CONTROL_TS);
    Commutator_init(&s_comm);

    for (;;)
    {
        ADC_Sample();

        /* Send binary telemetry at 50 Hz (400 ticks * 50 µs = 20 ms) */
        if ((g_isrCount - last_telem) >= 400u)
        {
            last_telem = g_isrCount;

            DRV835x_ReadStatus(&drv);
            int hw_fault = GD_NFAULT_ACTIVE();

            float Ic = -(g_adcSoA + g_adcSoB);

            uint8_t fault = (uint8_t)((hw_fault ? 0x01u : 0u) |
                                      (DRV835x_HasFault(&drv) ? 0x02u : 0u));

            /* theta slot: FOC sends angle*10000, 6-step sends step index */
            uint16_t theta_word = (g_mode == 0u)
                ? (uint16_t)(g_theta * 10000.0f) % 62832u
                : (uint16_t)g_step;

            uint8_t frame[TELEM_LEN];
            frame[0] = TELEM_SYNC0;
            frame[1] = TELEM_SYNC1;
            pack_be16 (&frame[2],  clamp16(g_adcSoA       * 100.0f));
            pack_be16 (&frame[4],  clamp16(g_adcSoB       * 100.0f));
            pack_be16 (&frame[6],  clamp16(Ic             * 100.0f));
            pack_beu16(&frame[8],  (uint16_t)(g_adcVbus   * 100.0f));
            pack_beu16(&frame[10], theta_word);
            pack_be16 (&frame[12], clamp16(g_omega        * 10.0f));
            pack_be16 (&frame[14], clamp16(g_adcAmbTemp   * 10.0f));
            pack_be16 (&frame[16], clamp16(g_adcPhaseTemp * 10.0f));
            frame[18] = fault;
            frame[19] = TELEM_TAIL;

            UART_WriteBytes(frame, TELEM_LEN);
        }

        __WFI();
    }
}
