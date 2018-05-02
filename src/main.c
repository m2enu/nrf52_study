/**
 * copyright (C) 2018 m2enu All Rights Reserved.
 *
 * @file RedBear Blend V2
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_gpio.h"

#define GPIO_CFG_OUTPUT(pin)    nrf_gpio_cfg_output(pin)
#define LED_BLEND_V2            25
#define LED_LEVEL_ON_           1
#define LED_LEVEL_OFF           0
#define LED_ON_(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_ON_)
#define LED_OFF(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_OFF)
#define LED_TGL(pin)            nrf_gpio_pin_toggle(pin)
#define LED_TICKS_MS            500
#define WAIT_MS(msec_wait)      nrf_delay_ms(msec_wait)

#define UART_BLEND_V2_RX        11 //!< RedBear Blend V2 UART RX pin number
#define UART_BLEND_V2_TX        12 //!< RedBear Blend V2 UART TX pin number

/** <!-- uart_error_handler {{{1
 * @brief UART error handler
 * @param[in] p_event UART event
 * @return nothing
 */
void uart_error_handler(app_uart_evt_t* p_event) {
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
        default:
            break;
    }
}

/** <!-- uart_init {{{1 -->
 * @brief initialize UART
 * @return nothing
 */
void uart_init(void) {
    const app_uart_comm_params_t cfg = {
        .rx_pin_no = UART_BLEND_V2_RX,
        .tx_pin_no = UART_BLEND_V2_TX,
        .rts_pin_no = UART_PIN_DISCONNECTED,
        .cts_pin_no = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
        .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200,
    };

    ret_code_t err;
    APP_UART_INIT(&cfg, uart_error_handler, APP_IRQ_PRIORITY_LOWEST, err);
    APP_ERROR_CHECK(err);
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
    uart_init();

    uint8_t c;
    while (1) {
        while (app_uart_get(&c) != NRF_SUCCESS);
        while (app_uart_put(c ) != NRF_SUCCESS);

        switch (c) {
            case '0':
                printf("LED ON\r\n");
                LED_OFF(LED_BLEND_V2);
                break;
            case '1':
                printf("LED OFF\r\n");
                LED_ON_(LED_BLEND_V2);
                break;
            case 't':
                printf("LED TOGGLE\r\n");
                LED_TGL(LED_BLEND_V2);
                break;
            default:
                break;
        }
    }

    return 0;
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
