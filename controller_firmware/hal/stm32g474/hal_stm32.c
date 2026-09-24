#include "hal.h"
#include "system.h"
#include "board.h"
#include "pwm.h"
#include "adc.h"
#include "spi.h"
#include "drv8353.h"
#include "fdcan.h"
#include "flash.h"
#include "usb_cdc.h"

void hal_init(void)
{
    system_init();
    board_init();
    spi_init();
    pwm_init();
    adc_init();
    usb_init();
    fdcan_init();
}

void hal_start(void)
{
    adc_irq_enable();
    pwm_start();
    wdg_init();
}

uint32_t hal_millis(void) { return ms_ticks; }

uint32_t hal_irq_save(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}

void hal_irq_restore(uint32_t s) { __set_PRIMASK(s); }

void hal_pwm_set(float da, float db, float dc) { pwm_set(da, db, dc); }
void hal_pwm_enable(int on) { if (on) pwm_on(); else pwm_off(); }

int hal_gate_init(void) { return drv_init(); }
void hal_gate_clear(void) { drv_clear_fault(); }

int hal_gate_status(uint16_t *s1, uint16_t *s2)
{
    *s1 = drv_read(0x00);
    *s2 = drv_read(0x01);
    return (*s1 & 0x400) != 0;
}

void hal_temp_poll(float *t_fet, float *t_amb) { temp_poll(t_fet, t_amb); }

int hal_can_send(uint32_t id, const uint8_t *d, uint8_t len) { return fdcan_send(id, d, len); }
int hal_can_recv(uint32_t *id, uint8_t *d, uint8_t *len) { return fdcan_recv(id, d, len); }

int hal_serial_write(const uint8_t *d, size_t n) { return usb_write(d, (uint32_t)n); }
size_t hal_serial_read(uint8_t *d, size_t max) { return usb_read(d, max); }

int hal_nv_read(void *d, size_t n) { return flash_read(d, n); }
int hal_nv_write(const void *d, size_t n) { return flash_write(d, n); }

void hal_led_toggle(void) { GPIOA->ODR ^= 1u << LED_PIN; }
void hal_wdg_kick(void) { wdg_kick(); }

/* 20 kHz: fires at ADC1 end of injected sequence (centre of low-side on time) */
void ADC1_2_IRQHandler(void)
{
    hal_sample_t s;
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    enc_cs_low();
    s.ia = (uint16_t)ADC2->JDR1;
    s.ib = (uint16_t)ADC1->JDR1;
    s.ic = (uint16_t)ADC3->JDR1;
    s.vbus = (uint16_t)ADC1->JDR2;
    enc_start();
    s.enc = enc_finish();
    app_control_isr(&s);
}
