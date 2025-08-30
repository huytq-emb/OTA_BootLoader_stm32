/*
 * crc32.c — IEEE 802.3 CRC-32 (reflected I/O)
 *
 * Polynomial : 0x04C11DB7  (reflected: 0xEDB88320)
 * Init       : 0xFFFFFFFF
 * XorOut     : 0xFFFFFFFF
 * Compatible with ESP32 side (same parameters).
 *
 * Properties:
 * - Supports incremental updates. Pass previous CRC result
 *   into `crc` parameter to continue calculation for new data block.
 * - Test vector: "123456789" -> 0xCBF4_3926
 *
 * Usage example:
 *   uint32_t crc = 0;                              // start with 0 (function handles ~crc)
 *   crc = crc32_update(crc, buf1, len1);           // block 1
 *   crc = crc32_update(crc, buf2, len2);           // block 2
 *   // crc is the final CRC-32
 */

#include "boot.h"
#include <stdint.h>

/**
 * @brief       Calculate IEEE 802.3 CRC-32 with incremental update capability
 * @param       crc Previous CRC value (use 0 for first call)
 * @param       data Pointer to data buffer
 * @param       len Length of data in bytes
 * @retval      Updated CRC-32 value
 */
uint32_t crc32_update(uint32_t crc, const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;

    /* Enter "working" state: ~crc for compatibility with both first call (crc=0)
       and subsequent updates (crc is the final result from previous call). */
    crc = ~crc;

    for (uint32_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (unsigned b = 0; b < 8; ++b) {
            /* mask = 0xFFFFFFFF if LSB of crc is 1, otherwise 0.
               This trick helps avoid if/else branch: */
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    /* Return "final" state: apply XorOut. */
    return ~crc;
}
