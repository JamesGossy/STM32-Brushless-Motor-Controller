/*
 * drv835x.c — DRV8350S / DRV8353S SPI register access and fault reporting
 *
 * SPI frame format (section 8.5, SLVSDY6):
 *   [15]    R/W bit: 1=read, 0=write
 *   [14:11] Address (4 bits)
 *   [10:0]  Data (11 bits)
 *
 * On a read, the device returns the register contents in bits [10:0] of the
 * MISO frame clocked back during the same 16-bit transaction.
 */

#include "drv835x.h"
#include "hal/hal_spi.h"
#include "hal/hal_adc.h"
#include <stdio.h>

#define DRV_READ(addr)        ((uint16_t)(0x8000u | ((addr) << 11u)))
#define DRV_WRITE(addr, data) ((uint16_t)(((addr) << 11u) | ((data) & 0x07FFu)))

static uint16_t drv_read_reg(uint8_t addr)
{
    uint16_t rx = SPI_GD_Transfer(DRV_READ(addr));
    return rx & 0x07FFu;
}

static void drv_write_reg(uint8_t addr, uint16_t data)
{
    SPI_GD_Transfer(DRV_WRITE(addr, data));
}

#define CSA_NORMAL  (DRV835X_CSA_GAIN_10 | DRV835X_CSA_VREF_DIV)
#define CSA_CAL_ALL (DRV835X_CSA_GAIN_10 | DRV835X_CSA_VREF_DIV | \
                     DRV835X_CSA_CAL_A | DRV835X_CSA_CAL_B | DRV835X_CSA_CAL_C)
#define CAL_SAMPLES 64u

void DRV835x_Configure(void)
{
    /* Set CSA gain to 10 V/V, bias = VREF/2.
     * With 7 mΩ shunt: full-scale = ±50 A -> ±3.5 V swing around VREF/2.
     * Default power-on value is 20 V/V which would clip at ±23 A. */
    drv_write_reg(DRV835X_REG_CSA_CTRL, CSA_NORMAL);
}

void DRV835x_CalibrateCSA(void)
{
    /* Enable calibration mode: DRV internally shorts SP/SN inputs so SOx
     * output reflects only the amplifier offset, not real current. */
    drv_write_reg(DRV835X_REG_CSA_CTRL, CSA_CAL_ALL);

    /* Short settling delay (~10 µs at 170 MHz) */
    for (volatile uint32_t i = 0; i < 1700u; i++);

    /* Average CAL_SAMPLES readings to get a stable offset voltage */
    float sumA = 0.0f, sumB = 0.0f;
    for (uint32_t i = 0; i < CAL_SAMPLES; i++)
        ADC_Sample_Raw(&sumA, &sumB);

    g_adcSoA_offset = (sumA / (float)CAL_SAMPLES) - ADC_CSA_VBIAS;
    g_adcSoB_offset = (sumB / (float)CAL_SAMPLES) - ADC_CSA_VBIAS;

    /* Restore normal operation and re-arm hardware trigger */
    drv_write_reg(DRV835X_REG_CSA_CTRL, CSA_NORMAL);
    ADC_RearmHwTrigger();

    printf("CSA cal: SoA_offset=%.4fV  SoB_offset=%.4fV\r\n",
           (double)g_adcSoA_offset, (double)g_adcSoB_offset);
}

void DRV835x_ReadStatus(DRV835x_Status_t *s)
{
    s->fault1 = drv_read_reg(DRV835X_REG_FAULT_STATUS1);
    s->vgs2   = drv_read_reg(DRV835X_REG_VGS_STATUS2);
}

void DRV835x_PrintFaults(const DRV835x_Status_t *s, int hw_fault)
{
    uint16_t f = s->fault1;
    uint16_t v = s->vgs2;

    if (!hw_fault && !DRV835x_HasFault(s)) {
        printf("OK\r\n");
        return;
    }

    if (hw_fault && !DRV835x_HasFault(s)) {
        printf("FAULT: nFAULT pin asserted\r\n");
        return;
    }

    printf("FAULT:");
    if (f & DRV835X_VDS_OCP) printf(" VDS_OCP");
    if (f & DRV835X_GDF)     printf(" GDF");
    if (f & DRV835X_UVLO)    printf(" UVLO");
    if (f & DRV835X_OTSD)    printf(" OTSD");
    if (f & DRV835X_VDS_HA)  printf(" VDS_HA");
    if (f & DRV835X_VDS_LA)  printf(" VDS_LA");
    if (f & DRV835X_VDS_HB)  printf(" VDS_HB");
    if (f & DRV835X_VDS_LB)  printf(" VDS_LB");
    if (f & DRV835X_VDS_HC)  printf(" VDS_HC");
    if (f & DRV835X_VDS_LC)  printf(" VDS_LC");
    if (v & DRV835X_SA_OC)   printf(" SA_OC");
    if (v & DRV835X_SB_OC)   printf(" SB_OC");
    if (v & DRV835X_SC_OC)   printf(" SC_OC");
    if (v & DRV835X_OTW)     printf(" OTW");
    if (v & DRV835X_GDUV)    printf(" GDUV");
    if (v & DRV835X_VGS_HA)  printf(" VGS_HA");
    if (v & DRV835X_VGS_LA)  printf(" VGS_LA");
    if (v & DRV835X_VGS_HB)  printf(" VGS_HB");
    if (v & DRV835X_VGS_LB)  printf(" VGS_LB");
    if (v & DRV835X_VGS_HC)  printf(" VGS_HC");
    if (v & DRV835X_VGS_LC)  printf(" VGS_LC");
    printf("\r\n");
}
