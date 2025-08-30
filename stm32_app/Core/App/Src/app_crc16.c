/*
 * app_crc16.c
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include <stdint.h>

/**
 * @brief       Calculate CRC16-CCITT checksum
 * @param       data Pointer to data buffer
 * @param       len Length of data in bytes
 * @retval      16-bit CRC checksum
 */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
