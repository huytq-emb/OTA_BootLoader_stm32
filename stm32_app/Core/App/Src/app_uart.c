/*
 * app_uart.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include "app.h"
#include "app_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* Ring buffer RX USART1 */
#define RX_RB_SZ 256
static volatile uint8_t  rx_rb[RX_RB_SZ];
static volatile uint16_t rx_head = 0, rx_tail = 0;

/**
 * @brief       Pop one byte from ring buffer (non-blocking)
 * @param       out Pointer to store the byte
 * @retval      1 if byte available, 0 if buffer empty
 */
static inline int rb_pop(uint8_t *out) {
    if (rx_head == rx_tail) return 0;
    *out = rx_rb[rx_tail % RX_RB_SZ];
    rx_tail++;
    return 1;
}
/**
 * @brief       Push one byte to ring buffer (with overflow protection)
 * @param       b Byte to push
 * @retval      None
 */
static inline void rb_push(uint8_t b) {
    rx_rb[rx_head % RX_RB_SZ] = b;
    rx_head++;
    if ((uint16_t)(rx_head - rx_tail) > RX_RB_SZ) rx_tail = rx_head - RX_RB_SZ;
}

/**
 * @brief       Initialize GPIO pins for UART and LED
 * @param       None
 * @retval      None
 */
static void uart_gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* LED PC13: output push-pull 10 MHz */
    GPIOC->CRH &= ~(0xFU << ((13U - 8U) * 4U));
    GPIOC->CRH |=  (0x1U << ((13U - 8U) * 4U));

    /* PA9 TX1 (AF PP), PA10 RX1 (floating) */
    uint32_t crh = GPIOA->CRH;
    crh &= ~((0xFU) << ((9U - 8U)  * 4U));  crh |= ((0xBU) << ((9U - 8U)  * 4U));
    crh &= ~((0xFU) << ((10U - 8U) * 4U));  crh |= ((0x4U) << ((10U - 8U) * 4U));
    GPIOA->CRH = crh;

    /* PB10 TX3 (AF PP), PB11 RX3 (floating) */
    uint32_t crh_b = GPIOB->CRH;
    crh_b &= ~((0xFU) << ((10U - 8U) * 4U)); crh_b |= ((0xBU) << ((10U - 8U) * 4U));
    crh_b &= ~((0xFU) << ((11U - 8U) * 4U)); crh_b |= ((0x4U) << ((11U - 8U) * 4U));
    GPIOB->CRH = crh_b;
}

/**
 * @brief       Configure UART peripheral with specified parameters
 * @param       U Pointer to USART peripheral
 * @param       pclk Peripheral clock frequency in Hz
 * @param       baud Desired baud rate
 * @param       enable_rx 1 to enable RX, 0 for TX only
 * @retval      None
 */
static void uart_setup(USART_TypeDef *U, uint32_t pclk, uint32_t baud, int enable_rx) {
    U->CR1 = 0; U->CR2 = 0; U->CR3 = 0;
    const uint32_t div_mantissa = pclk / (16U * baud);
    const uint32_t div_frac     = ((pclk % (16U * baud)) * 16U) / (16U * baud);
    U->BRR = (div_mantissa << 4) | (div_frac & 0xFU);
    U->CR1 = USART_CR1_TE | USART_CR1_UE;
    if (enable_rx) U->CR1 |= USART_CR1_RE;
}

/**
 * @brief       Initialize UART peripherals and interrupts
 * @param       None
 * @retval      None
 */
void uart_init(void) {
    uart_gpio_init();
    uart_setup(USART1, SYS_CORE_CLOCK_HZ,   UART1_BAUD, 1);
    uart_setup(USART3, SYS_CORE_CLOCK_HZ/2, UART3_BAUD, 0);

    USART1->CR1 |= USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 10);
    NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief       USART1 interrupt handler for receiving data
 * @param       None
 * @retval      None
 */
void USART1_IRQHandler(void) {
    const uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        const uint8_t d = (uint8_t)USART1->DR;
        rb_push(d);
    }
}

/**
 * @brief       Send data buffer via USART1
 * @param       p Pointer to data buffer
 * @param       n Number of bytes to send
 * @retval      None
 */
void uart1_write(const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) { while (!(USART1->SR & USART_SR_TXE)) {} USART1->DR = p[i]; }
}

/**
 * @brief       Send string via USART1
 * @param       s Pointer to null-terminated string
 * @retval      None
 */
void uart1_write_str(const char *s) { while (*s) { while(!(USART1->SR & USART_SR_TXE)){} USART1->DR = *s++; } }

/**
 * @brief       Read one byte from USART1 with timeout (FreeRTOS compatible)
 * @param       out Pointer to store received byte
 * @param       timeout_ms Timeout in milliseconds
 * @retval      0 on success, -1 on timeout
 */
int uart1_read_byte(uint8_t *out, uint32_t timeout_ms) {
    const TickType_t t0 = xTaskGetTickCount();
    const TickType_t to = pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        if (rb_pop(out)) return 0;
        if ((xTaskGetTickCount() - t0) >= to) return -1;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief       Send one byte via USART3
 * @param       b Byte to send
 * @retval      None
 */
static void uart3_write_byte(uint8_t b) { while (!(USART3->SR & USART_SR_TXE)) {} USART3->DR = b; }

/**
 * @brief       Send string via USART3
 * @param       s Pointer to null-terminated string  
 * @retval      None
 */
void uart3_write_str(const char *s) { while (*s) uart3_write_byte((uint8_t)*s++); }

/**
 * @brief       Printf-style formatted logging via USART3
 * @param       fmt Format string
 * @param       ... Variable arguments
 * @retval      None
 */
void log_printf(const char *fmt, ...) {
    char buf[160];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart3_write_str(buf);
}
