/**
 * copyright (C) 2018 m2enu All Rights Reserved.
 *
 * @file RedBear Blend V2
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_drv_twi.h"

#define GPIO_CFG_OUTPUT(pin)    nrf_gpio_cfg_output(pin)
#define LED_BLEND_V2            25
#define LED_LEVEL_ON_           1
#define LED_LEVEL_OFF           0
#define LED_ON_(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_ON_)
#define LED_OFF(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_OFF)
#define LED_TGL(pin)            nrf_gpio_pin_toggle(pin)
#define LED_TICKS_MS            500
#define WAIT_MS(msec_wait)      nrf_delay_ms(msec_wait)

#define TWI_NO_STOP             false
#define TWI_STOP                true

#define BME280_ADDR             0x76
#define BME280_CTRL_MEAS        0xf4

/**
 * @brief TWI driver instance
 */
static const nrf_drv_twi_t p_twi = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief TWI driver configuration
 */
static const nrf_drv_twi_config_t p_cfg = {
    .scl                = 27,
    .sda                = 26,
    .frequency          = NRF_TWI_FREQ_100K,
    .interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT,
    .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT,
};

/** <!-- twi_init {{{1 -->
 * @brief initialize TWI Master
 * @return nothing
 */
void twi_init(void) {
    ret_code_t err;
    err = nrf_drv_twi_init(&p_twi, &p_cfg, NULL, NULL);
    if (err == NRF_SUCCESS) {
        nrf_drv_twi_enable(&p_twi);
    } else {
        nrf_drv_twi_uninit(&p_twi);
    }
}

/** <!-- twi_wr {{{1 -->
 * @brief sending data to TWI slave
 * @param[in] sadr slave address in 7bits
 * @param[in] data pointer to transmit buffer
 * @param[in] length number of bytes to send
 * @return result of write
 */
uint32_t twi_wr(uint8_t sadr, const uint8_t* data, uint8_t length) {
    ret_code_t err;
    err = nrf_drv_twi_tx(&p_twi, sadr, data, length, TWI_NO_STOP);
    return err;
}

/** <!-- twi_rd {{{1 -->
 * @brief reading data from TWI slave
 * @param[in] sadr slave address in 7bits
 * @param[in] radr register address
 * @param[in] data pointer to receive buffer
 * @param[in] length number of bytes to be received
 * @return result of write
 */
uint32_t twi_rd(uint8_t sadr, uint8_t radr, uint8_t* data, uint8_t length) {
    ret_code_t err;
    err = nrf_drv_twi_tx(&p_twi, sadr, &radr, 1, TWI_STOP);
    if (err == NRF_SUCCESS) {
        err = nrf_drv_twi_rx(&p_twi, sadr, data, length);
    }
    return err;
}

/** <!-- gpio_init {{{1 -->
 * @brief initialize GPIO(s)
 * @return nothing
 */
void gpio_init(void) {
    GPIO_CFG_OUTPUT(LED_BLEND_V2);
    LED_OFF(LED_BLEND_V2);
}

/** <!-- main {{{1 -->
 * @brief main function
 * @return result
 */
int main(void) {
    gpio_init();
    twi_init();

    uint8_t data[8];
    uint32_t err;
    const uint8_t BME280_VALUE = 0xb9;
    while (1) {
        LED_OFF(LED_BLEND_V2);
        WAIT_MS(100);

        // write
        data[0] = BME280_CTRL_MEAS;
        data[1] = BME280_VALUE;
        err = twi_wr(BME280_ADDR, data, 2);

        // read
        data[0] = 0x00;
        err = twi_rd(BME280_ADDR, BME280_CTRL_MEAS, &data[0], 1);
        if ((err == NRF_SUCCESS) && (data[0] == BME280_VALUE)) {
            LED_ON_(LED_BLEND_V2);
        } else {
            LED_OFF(LED_BLEND_V2);
        }
        WAIT_MS(1000);
    }

    return 0;
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
