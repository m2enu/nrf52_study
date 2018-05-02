/**
 * copyright (C) 2018 m2enu All Rights Reserved.
 *
 * @file RedBear Blend V2
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "sdk_errors.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"

#define GPIO_CFG_OUTPUT(pin)    nrf_gpio_cfg_output(pin)
#define LED_BLEND_V2            25
#define LED_LEVEL_ON_           1
#define LED_LEVEL_OFF           0
#define LED_ON_(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_ON_)
#define LED_OFF(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_OFF)
#define LED_TGL(pin)            nrf_gpio_pin_toggle(pin)
#define LED_TICKS_MS            100
#define WAIT_MS(msec_wait)      nrf_delay_ms(msec_wait)

const nrf_drv_timer_t TIMER0 = NRF_DRV_TIMER_INSTANCE(0);

/** <!-- gpio_init {{{1 -->
 * @brief initialize GPIO(s)
 * @return nothing
 */
void gpio_init(void) {
    GPIO_CFG_OUTPUT(LED_BLEND_V2);
    LED_OFF(LED_BLEND_V2);
}

/** <!-- timer0_event_handler {{{1 -->
 * @brief handler for timer 0
 * @param[in] event_type timer event
 * @param[in] p_context general purpose parameter
 * @return nothing
 */
void timer0_event_handler(nrf_timer_event_t event_type, void* p_context) {
    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            LED_TGL(LED_BLEND_V2);
            break;
        default:
            break;
    }
}

/** <!-- timer0_init {{{1 -->
 * @brief initialize timer 0
 * @return nothing
 */
void timer0_init(void) {
    nrf_drv_timer_config_t cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    ret_code_t err = nrf_drv_timer_init(&TIMER0, &cfg, timer0_event_handler);
    APP_ERROR_CHECK(err);

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&TIMER0, (uint32_t)LED_TICKS_MS);

    nrf_drv_timer_extended_compare(
        &TIMER0, NRF_TIMER_CC_CHANNEL0, ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&TIMER0);
}

/** <!-- main {{{1 -->
 * @brief main function
 * @return result
 */
int main(void) {
    gpio_init();
    timer0_init();

    while (1) {
        __WFI();
    }

    return 0;
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
