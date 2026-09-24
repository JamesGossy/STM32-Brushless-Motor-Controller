/*
 * drv8353.c - gate driver configuration and status.
 *
 * SPI frame: bit 15 = read, bits 14:11 = register, bits 10:0 = data.
 * The driver's nFAULT pin only drives an LED on this board, so faults are
 * found by polling the status registers (see hal_gate_status()).
 */
#include "drv8353.h"
#include "spi.h"
#include "system.h"

/* Register values written at start-up and checked by reading them back. */
static const uint16_t config[][2] = {
    {0x02, 0x400},  /* driver control: OCP shuts down all bridges, 6x PWM mode */
    {0x03, 0x388},  /* HS gate drive: unlock, 550 mA source / 1100 mA sink */
    {0x04, 0x788},  /* LS gate drive: cycle-by-cycle, 4 us TDRIVE, 550 / 1100 mA */
    {0x05, 0x112},  /* OCP: 100 ns dead time, latched, OCP_DEG 01, VDS trip 80 mV */
    {0x06, 0x283},  /* current amps: VREF/2 bias, gain 20 V/V, 1 V sense trip */
};

/* One 16-bit transfer. Interrupts are off because the control loop uses the
   same SPI bus for the encoder. */
static uint16_t transfer(uint16_t word)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    spi_mode_driver();
    GPIOC->BRR = 1u << GD_CS_PIN;
    *(volatile uint16_t *)&SPI1->DR = word;
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    uint16_t reply = *(volatile uint16_t *)&SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY) {}
    GPIOC->BSRR = 1u << GD_CS_PIN;
    spi_mode_encoder();

    __set_PRIMASK(primask);
    delay_us(1);    /* minimum CS high time between frames */
    return reply & 0x7FF;
}

uint16_t drv_read(uint8_t addr) { return transfer(0x8000u | ((uint16_t)addr << 11)); }

void drv_write(uint8_t addr, uint16_t data) { transfer(((uint16_t)addr << 11) | (data & 0x7FF)); }

/* Wake the driver, write the configuration and verify it. Returns 1 if OK. */
int drv_init(void)
{
    GPIOB->BSRR = 1u << ENABLE_PIN;
    delay_ms(2);                    /* wake-up time before SPI works */
    drv_write(0x02, 0x001);         /* clear any power-up faults */

    int ok = 1;
    for (unsigned i = 0; i < sizeof config / sizeof config[0]; i++) {
        drv_write(config[i][0], config[i][1]);
        if (drv_read(config[i][0]) != config[i][1]) ok = 0;
    }
    return ok;
}

/* Clear latched faults (CLR_FLT bit in driver control). */
void drv_clear_fault(void) { drv_write(0x02, config[0][1] | 0x001); }
