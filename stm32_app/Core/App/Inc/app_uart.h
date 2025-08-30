/*
 * app_uart.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#ifndef APP_INC_APP_UART_H_
#define APP_INC_APP_UART_H_

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(void);
void uart1_write(const uint8_t *p, uint32_t n);
void uart1_write_str(const char *s);
int  uart1_read_byte(uint8_t *out, uint32_t timeout_ms);
void uart3_write_str(const char *s);
void log_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_INC_APP_UART_H_ */
