/*
 * flash.c — Flash erase/program helpers for STM32F1 (halfword programming)
 * - Blocking, no dynamic allocation.
 * - Caller is responsible for ensuring valid address ranges.
 * - REQUIREMENT: Must call flash_unlock() before erase/program, and flash_lock() after completion.
 */

#include "boot.h"

/* ============ Internal Helpers ============ */

/**
 * @brief       Wait for flash operation to complete (BSY=0)
 * @param       None
 * @retval      None
 */
static inline void flash_wait(void) {
    while (FLASH->SR & FLASH_SR_BSY) { /* spin */ }
}

/**
 * @brief       Clear flash error flags and check for errors
 * @param       None
 * @retval      0 if OK, -1 if programming/write protection error
 */
static inline int flash_clear_and_check_errors(void) {
    /* EOP (End of operation) flag indicates completion of an operation, must be cleared after use. */
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;  /* write 1 to clear */
    }
    /* If there are programming errors/write protection violations → clear flags and report error */
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
        FLASH->SR |= FLASH_SR_PGERR | FLASH_SR_WRPRTERR; /* write 1 to clear */
        return -1;
    }
    return 0;
}

/**
 * @brief       Check if address range is within metadata region
 * @param       addr Start address
 * @param       len Length in bytes
 * @retval      1 if within metadata region, 0 otherwise
 */
static inline int within_meta_region(uint32_t addr, uint32_t len) {
    const uint32_t end = addr + (len ? len : 1U);
    return (addr >= META_BASE) && (end <= (META_BASE + FLASH_PAGE_SIZE));
}

/* ============ API ============ */

/**
 * @brief       Unlock flash for write/erase operations
 * @param       None
 * @retval      None
 */
void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
}

/**
 * @brief       Lock flash to prevent accidental writes
 * @param       None
 * @retval      None
 */
void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief       Wait for flash busy and check for errors
 * @param       None
 * @retval      0 if OK, -1 if error occurred
 */
int flash_wait_busy(void) {
    flash_wait();
    return flash_clear_and_check_errors();
}

/**
 * @brief       Erase one flash page at specified address
 * @param       page_addr Page start address (must be page-aligned)
 * @retval      0 on success, -1 on error
 */
int flash_erase_page(uint32_t page_addr) {
    flash_wait();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = page_addr;
    FLASH->CR |= FLASH_CR_STRT;

    flash_wait();

    FLASH->CR &= ~FLASH_CR_PER;

    return flash_clear_and_check_errors();
}

/**
 * @brief       Write one halfword (16-bit) to flash memory
 * @param       addr Target address (must be 2-byte aligned)
 * @param       half 16-bit data to write
 * @retval      0 on success, -1 on error
 */
int flash_write_halfword(uint32_t addr, uint16_t half) {
    flash_wait();

    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)addr = half; /* halfword programming */
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;

    if (flash_clear_and_check_errors() != 0) {
        return -1;
    }

    /* Verify readback */
    return (*(__IO uint16_t*)addr == half) ? 0 : -1;
}

/**
 * @brief       Write byte array to flash memory (APP or metadata regions)
 * @param       addr Target address
 * @param       data Pointer to data buffer
 * @param       len Number of bytes to write
 * @retval      0 on success, -1 on error or invalid address range
 */
int flash_write(uint32_t addr, const uint8_t* data, uint32_t len) {
    /* ALLOWED: APP region or Metadata page */
    if (!( within_app_region(addr, len ? len : 1U) ||
           within_meta_region(addr, len ? len : 1U) )) {
        return -1;
    }

    uint32_t i = 0;
    while (i < len) {
        uint16_t hw;
        if ((i + 1U) < len) {
            hw = (uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8);
        } else {
            hw = (uint16_t)data[i] | ((uint16_t)0xFFU << 8); /* pad with 0xFF */
        }
        if (flash_write_halfword(addr + i, hw) != 0) {
            return -1;
        }
        i += 2U;
    }
    return 0;
}

/**
 * @brief       Erase flash pages to accommodate specified number of bytes in APP region
 * @param       bytes Number of bytes that need to be stored (rounded up to page boundary)
 * @retval      0 on success, -1 on error
 */
int flash_erase_app_region(uint32_t bytes) {
    if (bytes > APP_MAX_SIZE) return -1;

    const uint32_t start = APP_BASE;
    const uint32_t end   = APP_BASE + ((bytes + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;

    for (uint32_t a = start; a < end; a += FLASH_PAGE_SIZE) {
        if (flash_erase_page(a) != 0) {
            return -1;
        }
    }
    return 0;
}
