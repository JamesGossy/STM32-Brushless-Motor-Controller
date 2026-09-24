#include "drv8353.h"
#include "spi.h"
#include "system.h"

static uint16_t drv_xfer(uint16_t w)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    spi_mode_drv();
    GPIOC->BRR = 1u << GD_CS_PIN;
    *(volatile uint16_t *)&SPI1->DR = w;
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    uint16_t r = *(volatile uint16_t *)&SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY) {}
    GPIOC->BSRR = 1u << GD_CS_PIN;
    spi_mode_enc();
    __set_PRIMASK(pm);
    delay_us(1);
    return r & 0x7FF;
}

uint16_t drv_read(uint8_t addr) { return drv_xfer(0x8000u | ((uint16_t)addr << 11)); }

void drv_write(uint8_t addr, uint16_t data) { drv_xfer(((uint16_t)addr << 11) | (data & 0x7FF)); }

static const uint16_t regs[][2] = {
    {0x02, 0x400},  /* OCP_ACT all bridges, 6x PWM */
    {0x03, 0x388},  /* unlock, IDRIVEP_HS 550 mA, IDRIVEN_HS 1100 mA */
    {0x04, 0x788},  /* CBC, TDRIVE 4 us, IDRIVEP_LS 550 mA, IDRIVEN_LS 1100 mA */
    {0x05, 0x112},  /* 100 ns dead time, latched OCP, OCP_DEG 01, VDS 80 mV */
    {0x06, 0x283},  /* VREF/2 bias, gain 20, SEN_LVL 1 V */
};

int drv_init(void)
{
    GPIOB->BSRR = 1u << ENABLE_PIN;
    delay_ms(2);
    drv_write(0x02, 0x001);
    int ok = 1;
    for (unsigned i = 0; i < sizeof regs / sizeof regs[0]; i++) {
        drv_write(regs[i][0], regs[i][1]);
        if (drv_read(regs[i][0]) != regs[i][1]) ok = 0;
    }
    return ok;
}

void drv_clear_fault(void) { drv_write(0x02, regs[0][1] | 0x001); }
