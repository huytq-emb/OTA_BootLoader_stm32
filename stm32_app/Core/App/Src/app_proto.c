/*
 * app_proto.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include "app.h"
#include "app_uart.h"
#include "app_meta.h"
#include "app_crc16.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief       Read exact number of bytes from UART with per-byte timeout
 * @param       buf Buffer to store received data
 * @param       len Number of bytes to read
 * @param       per_byte_timeout_ms Timeout per byte in milliseconds
 * @retval      0 on success, -1 on timeout
 */
static int read_exact(uint8_t *buf, uint32_t len, uint32_t per_byte_timeout_ms) {
    for (uint32_t i = 0; i < len; i++) {
        if (uart1_read_byte(&buf[i], per_byte_timeout_ms) < 0) return -1;
    }
    return 0;
}

/**
 * @brief       Send ACK response with sequence number
 * @param       seq Sequence number to acknowledge
 * @retval      None
 */
static void send_ack(uint8_t seq)  { uint8_t a[2] = { ACK_BYTE,  seq }; uart1_write(a, 2); }

/**
 * @brief       Send NACK response with sequence number
 * @param       seq Sequence number to not acknowledge
 * @retval      None
 */
static void send_nack(uint8_t seq) { uint8_t a[2] = { NACK_BYTE, seq }; uart1_write(a, 2); }

/**
 * @brief       UART command processing task
 * @param       arg Task parameter (unused)
 * @retval      None (never returns)
 */
void uart_cmd_task(void *arg) {
    (void)arg;
    for (;;) {
        uint8_t b;
        if (uart1_read_byte(&b, 100) == 0) {
            if (b != STX) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

            uint8_t head[3];
            if (read_exact(head, 3, 50) < 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
            const uint8_t type = head[0];
            const uint8_t seq  = head[1];
            const uint8_t lenL = head[2];
            uint8_t lenH;
            if (read_exact(&lenH, 1, 50) < 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
            const uint16_t len = (uint16_t)lenL | ((uint16_t)lenH << 8);

            if (len > 64) {
                uint8_t drop;
                for (uint16_t i = 0; i < (uint16_t)(len + 2); i++) {
                    if (uart1_read_byte(&drop, 10) < 0) break;
                }
                vTaskDelay(pdMS_TO_TICKS(1)); continue;
            }

            uint8_t payload[64];
            if (len && read_exact(payload, len, 200) < 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

            uint8_t c16lsb, c16msb;
            if (read_exact(&c16lsb, 1, 50) < 0 || read_exact(&c16msb, 1, 50) < 0) {
                vTaskDelay(pdMS_TO_TICKS(1)); continue;
            }
            const uint16_t rx_crc = (uint16_t)c16lsb | ((uint16_t)c16msb << 8);

            uint8_t cbuf[5 + 64];
            cbuf[0]=STX; cbuf[1]=type; cbuf[2]=seq; cbuf[3]=lenL; cbuf[4]=lenH;
            if (len) memcpy(&cbuf[5], payload, len);
            const uint16_t calc = crc16_ccitt(cbuf, 5 + len);
            if (calc != rx_crc) { send_nack(seq); vTaskDelay(pdMS_TO_TICKS(1)); continue; }

            if (type == TYPE_Q && len == 1) {
                const uint8_t cmd = payload[0];
                log_printf("[APP] RX Q: cmd=0x%02X seq=%u len=%u\n", cmd, seq, (unsigned)len);

                if (cmd == CMD_BOOT) {
                    send_ack(seq);
                    while (!(USART1->SR & USART_SR_TC)) {}
                    log_printf("[APP] CMD_BOOT -> set BOOT_REQ & reset\n");
                    meta_request_boot();
                    vTaskDelay(pdMS_TO_TICKS(10));
                    NVIC_SystemReset();
                } else {
                    send_ack(seq);
                }
            } else {
                send_nack(seq);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
