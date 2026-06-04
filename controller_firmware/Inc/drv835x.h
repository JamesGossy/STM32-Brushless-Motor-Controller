#pragma once

#include <stdint.h>

/*
 * drv835x.h — DRV8350S / DRV8353S SPI register interface
 *
 * SPI frame (16-bit, MSB first):
 *   Bit 15:    R/W (1=read, 0=write)
 *   Bits 14-11: register address (4 bits)
 *   Bits 10-0:  data (11 bits)
 *
 * Register map (SPI variant, section 8.6 of SLVSDY6):
 *   0x00  Fault Status 1  (read-only)
 *   0x01  VGS Status 2    (read-only)
 *   0x02  Driver Control
 *   0x03  Gate Drive HS
 *   0x04  Gate Drive LS
 *   0x05  OCP Control
 *   0x06  CSA Control
 */

/* Register addresses */
#define DRV835X_REG_FAULT_STATUS1   0x00u
#define DRV835X_REG_VGS_STATUS2     0x01u
#define DRV835X_REG_DRIVER_CTRL     0x02u
#define DRV835X_REG_GATE_DRIVE_HS   0x03u
#define DRV835X_REG_GATE_DRIVE_LS   0x04u
#define DRV835X_REG_OCP_CTRL        0x05u
#define DRV835X_REG_CSA_CTRL        0x06u

/* CSA Control register bits */
#define DRV835X_CSA_GAIN_5          (0u << 6)   /* 5 V/V  */
#define DRV835X_CSA_GAIN_10         (1u << 6)   /* 10 V/V */
#define DRV835X_CSA_GAIN_20         (2u << 6)   /* 20 V/V (power-on default) */
#define DRV835X_CSA_GAIN_40         (3u << 6)   /* 40 V/V */
#define DRV835X_CSA_VREF_DIV        (1u << 9)   /* SOx bias = VREF/2 (default) */
#define DRV835X_CSA_CAL_A           (1u << 4)   /* short SPA/SNA -> measures amp offset */
#define DRV835X_CSA_CAL_B           (1u << 3)   /* short SPB/SNB */
#define DRV835X_CSA_CAL_C           (1u << 2)   /* short SPC/SNC */

/* Fault Status 1 bit masks */
#define DRV835X_FAULT       (1u << 10)   /* any fault latched */
#define DRV835X_VDS_OCP     (1u <<  9)   /* VDS overcurrent (any FET) */
#define DRV835X_GDF         (1u <<  8)   /* gate drive fault */
#define DRV835X_UVLO        (1u <<  7)   /* undervoltage lockout */
#define DRV835X_OTSD        (1u <<  6)   /* overtemp shutdown */
#define DRV835X_VDS_HA      (1u <<  5)   /* VDS OCP high-side A */
#define DRV835X_VDS_LA      (1u <<  4)   /* VDS OCP low-side A */
#define DRV835X_VDS_HB      (1u <<  3)   /* VDS OCP high-side B */
#define DRV835X_VDS_LB      (1u <<  2)   /* VDS OCP low-side B */
#define DRV835X_VDS_HC      (1u <<  1)   /* VDS OCP high-side C */
#define DRV835X_VDS_LC      (1u <<  0)   /* VDS OCP low-side C */

/* VGS Status 2 bit masks */
#define DRV835X_SA_OC       (1u <<  10)  /* sense amp A overcurrent */
#define DRV835X_SB_OC       (1u <<  9)   /* sense amp B overcurrent */
#define DRV835X_SC_OC       (1u <<  8)   /* sense amp C overcurrent */
#define DRV835X_OTW         (1u <<  7)   /* overtemp warning */
#define DRV835X_GDUV        (1u <<  6)   /* gate drive undervoltage */
#define DRV835X_VGS_HA      (1u <<  5)   /* VGS fault high-side A */
#define DRV835X_VGS_LA      (1u <<  4)   /* VGS fault low-side A */
#define DRV835X_VGS_HB      (1u <<  3)   /* VGS fault high-side B */
#define DRV835X_VGS_LB      (1u <<  2)   /* VGS fault low-side B */
#define DRV835X_VGS_HC      (1u <<  1)   /* VGS fault high-side C */
#define DRV835X_VGS_LC      (1u <<  0)   /* VGS fault low-side C */

typedef struct {
    uint16_t fault1;   /* raw Fault Status 1 register */
    uint16_t vgs2;     /* raw VGS Status 2 register   */
} DRV835x_Status_t;

void DRV835x_Configure(void);   /* write gain=10 V/V to CSA Control register */
void DRV835x_CalibrateCSA(void); /* measure and store amp offsets, call once at startup with PWM disabled */
void DRV835x_ReadStatus(DRV835x_Status_t *s);
/* hw_fault: 1 if nFAULT GPIO is asserted (active-low), 0 otherwise */
void DRV835x_PrintFaults(const DRV835x_Status_t *s, int hw_fault);

/* Returns 1 if any fault bit is set */
static inline int DRV835x_HasFault(const DRV835x_Status_t *s)
{
    return (s->fault1 & DRV835X_FAULT) ? 1 : 0;
}
