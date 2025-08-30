/*
 * uart.c — USART1 (ESP32) with RX ring buffer + ISR
 *          USART3 (PC log) simple blocking TX
 * Bare-metal CMSIS (no HAL)
 *
 * Pins:
 *   USART1: PA9 (TX1, AF PP 50MHz), PA10 (RX1, floating input)  -> ESP32
 *   USART3: PB10 (TX3, AF PP 50MHz), PB11 (RX3, floating input) -> PC (RX3 not used)
 */

#include "boot.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* =========================
 *  Ring buffer for USART1 RX
 * ========================= */
#define RB_SIZE 1024
static volatile uint8_t  rb[RB_SIZE];
static volatile uint16_t rb_head = 0;
static volatile uint16_t rb_tail = 0;

/* =========================
 *  Millisecond timebase (SysTick)
 * ========================= */
static volatile uint32_t g_tick_ms = 0;

/**
 * @brief       SysTick interrupt handler for 1ms timebase
 * @param       None
 * @retval      None
 */
void SysTick_Handler(void) {
    g_tick_ms++;
}

/**
 * @brief       Get current millisecond tick count
 * @param       None
 * @retval      Current millisecond count since startup
 */
static inline uint32_t millis(void) { return g_tick_ms; }

/* =========================
 *  USART1 IRQ: push bytes into RX ring
 * ========================= */
/**
 * @brief       USART1 interrupt handler for RX data reception
 * @param       None
 * @retval      None
 */
void USART1_IRQHandler(void) {
    /* RXNE: data available */
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t d = (uint8_t)USART1->DR; /* read clears RXNE */
        uint16_t next = (uint16_t)((rb_head + 1U) % RB_SIZE);
        if (next != rb_tail) {
            rb[rb_head] = d;
            rb_head = next;
        } else {
            /* overflow -> drop byte */
        }
    }
    /* Note: ORE cleared by "read SR then DR"; SR was read above, DR read too */
    (void)USART1->SR;
}

/* =========================
 *  GPIO/clock setup
 * ========================= */
/**
 * @brief       Initialize GPIO pins for UART1 and UART3
 * @param       None
 * @retval      None
 */
static void uart_gpio_init(void) {
    /* Enable clocks: GPIOA/B, AFIO, USART1 (APB2), USART3 (APB1) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* ------- PA9: TX1 (AF PP 50MHz), PA10: RX1 (floating input) ------- */
    uint32_t crh_a = GPIOA->CRH;
    /* PA9: MODE=11 (50MHz), CNF=10 (AF PP) */
    crh_a &= ~((0xFU) << ((9U - 8U) * 4U));
    crh_a |=  ((0xBU) << ((9U - 8U) * 4U));
    /* PA10: MODE=00 (input), CNF=01 (floating) */
    crh_a &= ~((0xFU) << ((10U - 8U) * 4U));
    crh_a |=  ((0x4U) << ((10U - 8U) * 4U));
    GPIOA->CRH = crh_a;

    /* ------- PB10: TX3 (AF PP 50MHz), PB11: RX3 (floating input) ------- */
    uint32_t crh_b = GPIOB->CRH; /* PB8..PB15 in CRH */
    /* PB10: MODE=11, CNF=10 */
    crh_b &= ~((0xFU) << ((10U - 8U) * 4U));
    crh_b |=  ((0xBU) << ((10U - 8U) * 4U));
    /* PB11: MODE=00, CNF=01 */
    crh_b &= ~((0xFU) << ((11U - 8U) * 4U));
    crh_b |=  ((0x4U) << ((11U - 8U) * 4U));
    GPIOB->CRH = crh_b;
}

/* =========================
 *  USART setups
 * ========================= */
/**
 * @brief       Configure USART1 for communication with ESP32
 * @param       baud Baud rate for USART1
 * @retval      None
 */
static void uart1_setup(uint32_t baud) {
    /* APB2 for USART1 is 72MHz */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    const uint32_t pclk = SYS_CORE_CLOCK_HZ; /* 72 MHz */
    /* BRR = mantissa[15:4] | fraction[3:0], where USARTDIV = pclk/(16*baud) */
    const uint32_t div_mantissa = pclk / (16U * baud);
    const uint32_t div_frac     = ((pclk % (16U * baud)) * 16U) / (16U * baud);
    USART1->BRR = (div_mantissa << 4) | (div_frac & 0xFU);

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief       Configure USART3 for PC logging communication
 * @param       baud Baud rate for USART3
 * @retval      None
 */
static void uart3_setup(uint32_t baud) {
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;

    /* APB1 for USART3 is 36MHz */
    const uint32_t pclk = SYS_CORE_CLOCK_HZ / 2U;
    const uint32_t div_mantissa = pclk / (16U * baud);
    const uint32_t div_frac     = ((pclk % (16U * baud)) * 16U) / (16U * baud);
    USART3->BRR = (div_mantissa << 4) | (div_frac & 0xFU);

    USART3->CR1 = USART_CR1_TE | USART_CR1_UE; /* TX only (logging) */
}

/* =========================
 *  Public API
 * ========================= */
/**
 * @brief       Initialize UART peripherals and SysTick timer
 * @param       None
 * @retval      None
 */
void uart_init(void) {
    uart_gpio_init();

    /* SysTick @1kHz */
    SysTick->LOAD = (SYS_CORE_CLOCK_HZ / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    uart1_setup(UART1_BAUD);
    uart3_setup(UART3_BAUD);
}

/**
 * @brief       Send one byte via USART1
 * @param       b Byte to transmit
 * @retval      None
 */
void uart1_write_byte(uint8_t b) {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = b;
}

/**
 * @brief       Send multiple bytes via USART1
 * @param       p Pointer to data buffer
 * @param       n Number of bytes to send
 * @retval      None
 */
void uart1_write(const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        uart1_write_byte(p[i]);
    }
}

/**
 * @brief       Read one byte from USART1 ring buffer with timeout
 * @param       out Pointer to store received byte
 * @param       timeout_ms Timeout in milliseconds
 * @retval      0 on success, -1 on timeout
 */
int uart1_read_byte(uint8_t *out, uint32_t timeout_ms) {
    const uint32_t t0 = millis();
    while (rb_head == rb_tail) {
        if ((millis() - t0) >= timeout_ms) return -1;
    }
    const uint8_t d = rb[rb_tail];
    rb_tail = (uint16_t)((rb_tail + 1U) % RB_SIZE);
    *out = d;
    return 0;
}

/* =========================
 *  Logging to PC via USART3
 * ========================= */
/**
 * @brief       Send one byte via USART3 for PC logging
 * @param       b Byte to transmit
 * @retval      None
 */
static void uart3_write_byte(uint8_t b) {
    while (!(USART3->SR & USART_SR_TXE)) { }
    USART3->DR = b;
}

/**
 * @brief       Send string via USART3 for PC logging
 * @param       s Null-terminated string to send
 * @retval      None
 */
void uart3_write_str(const char *s) {
    while (*s) uart3_write_byte((uint8_t)*s++);
}

/**
 * @brief       Printf-style formatted logging via USART3
 * @param       fmt Format string
 * @param       ... Variable arguments
 * @retval      None
 */
void log_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    uart3_write_str(buf);
}

/* =========================
 *  delay_ms exported
 * ========================= */
void delay_ms(uint32_t ms) {
    const uint32_t t0 = g_tick_ms;
    while ((g_tick_ms - t0) < ms) { __NOP(); }
}
