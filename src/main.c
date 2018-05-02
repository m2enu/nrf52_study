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

#define GPIO_CFG_OUTPUT(pin)    nrf_gpio_cfg_output(pin)
#define LED_BLEND_V2            25
#define LED_LEVEL_ON_           1
#define LED_LEVEL_OFF           0
#define LED_ON_(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_ON_)
#define LED_OFF(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_OFF)
#define LED_TGL(pin)            nrf_gpio_pin_toggle(pin)
#define LED_TICKS_MS            500
#define WAIT_MS(msec_wait)      nrf_delay_ms(msec_wait)

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

    while (1) {
        LED_TGL(LED_BLEND_V2);
        WAIT_MS(LED_TICKS_MS);
    }

    return 0;
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
