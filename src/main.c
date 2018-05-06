/**
 * copyright (C) 2018 m2enu All Rights Reserved.
 *
 * @file RedBear Blend V2
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_uart.h"

#define GPIO_CFG_OUTPUT(pin)    nrf_gpio_cfg_output(pin)
#define LED_BLEND_V2            25
#define LED_LEVEL_ON_           1
#define LED_LEVEL_OFF           0
#define LED_ON_(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_ON_)
#define LED_OFF(pin)            nrf_gpio_pin_write(pin, LED_LEVEL_OFF)
#define LED_TGL(pin)            nrf_gpio_pin_toggle(pin)
#define LED_TICKS_MS            500
#define WAIT_MS(msec_wait)      nrf_delay_ms(msec_wait)

#define UART_BLEND_V2_RX        8 //!< RedBear Blend V2 UART RX pin number
#define UART_BLEND_V2_TX        6 //!< RedBear Blend V2 UART TX pin number

/**
 * @brief UART driver instance
 */
const nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);

/** <!-- uart_init {{{1 -->
 * @brief initialize UART in blocking mode
 * @return nothing
 */
void uart_init(void) {
    const nrf_drv_uart_config_t cfg = {
        .pselrxd = UART_BLEND_V2_RX,
        .pseltxd = UART_BLEND_V2_TX,
        .pselcts = NRF_UART_PSEL_DISCONNECTED,
        .pselrts = NRF_UART_PSEL_DISCONNECTED,
        .p_context = NULL,
        .hwfc = (nrf_uart_hwfc_t)NRF_UART_HWFC_DISABLED,
        .parity = (nrf_uart_parity_t)NRF_UART_PARITY_EXCLUDED,
        .baudrate = (nrf_uart_baudrate_t)NRF_UART_BAUDRATE_115200,
        .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
#ifdef UARTE_PRESENT
        .use_easy_dma = DEFAULT_CONFIG_USE_EASY_DMA,
#endif
    };
    ret_code_t err;
    err = nrf_drv_uart_init(&m_uart, &cfg, NULL);
    if (err != NRF_SUCCESS) {
        return;
    }
    
#ifdef UARTE_PRESENT
    if (!cfg.use_easy_dma)
#endif
    {
        nrf_drv_uart_rx_enable(&m_uart);
    }
}

/** <!-- uart_getc {{{1 -->
 * @brief receive character via UART
 * @param[in] buf pointer to received data container
 * @return result of receive
 */
uint32_t uart_getc(uint8_t* buf) {
    if (buf == NULL) {
        return NRF_ERROR_NULL;
    }
    ret_code_t err;
    err = nrf_drv_uart_rx(&m_uart, buf, 1);
    return err;
}

/** <!-- uart_putc {{{1 -->
 * @brief sending character via UART
 * @param[in] buf character
 * @return result of send
 */
uint32_t uart_putc(const uint8_t buf) {
    ret_code_t err;
    err = nrf_drv_uart_tx(&m_uart, &buf, 1);
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
    uart_init();

    uint8_t c;
    while (1) {
        while (uart_getc(&c) != NRF_SUCCESS);
        while (uart_putc(c ) != NRF_SUCCESS);

        switch (c) {
            case '0':
                LED_OFF(LED_BLEND_V2);
                break;
            case '1':
                LED_ON_(LED_BLEND_V2);
                break;
            case 't':
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
