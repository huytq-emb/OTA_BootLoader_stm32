/*
 * app_meta.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include "app.h"
#include <string.h>

/**
 * @brief       Request bootloader entry by setting boot request flag in metadata
 * @param       None
 * @retval      None
 */
void meta_request_boot(void) {
    bl_meta_t m;
    memcpy(&m, (void *)META_BASE, sizeof(m));
    if (m.magic != META_MAGIC || m.flags == 0xFFFFFFFFu) {
        m.magic = META_MAGIC; m.version = 0; m.size = 0; m.crc32 = 0; m.flags = 0;
    }
    m.flags &= ~MF_IN_PROGRESS;
    m.flags |=  MF_BOOT_REQ;

    FLASH->KEYR = 0x45670123; FLASH->KEYR = 0xCDEF89AB;

    FLASH->CR |= FLASH_CR_PER; FLASH->AR = META_BASE; FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR &= ~FLASH_CR_PER;

    FLASH->CR |= FLASH_CR_PG;
    uint16_t *src = (uint16_t *)&m;
    uint16_t *dst = (uint16_t *)META_BASE;
    for (size_t i = 0; i < sizeof(m)/2; i++) {
        dst[i] = src[i];
        while (FLASH->SR & FLASH_SR_BSY) {}
    }
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
}
