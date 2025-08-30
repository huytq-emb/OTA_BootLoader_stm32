#ifndef APP_H
#define APP_H
/*
 * Application header (runs under FreeRTOS native API)
 *
 * Links:
 *   - UART1 <-> ESP32 (command bridge)
 *   - UART3 -> PC (logging)
 *
 * Responsibilities:
 *   - Parse UART frame TYPE_Q/CMD_BOOT and ACK
 *   - Share protocol/metadata constants with bootloader (must match boot.h)
 */

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Flash base of Application (must match bootloader map) ===== */
#ifndef APP_BASE
#define APP_BASE            0x08004000UL
#endif

/* ===== Clocks ===== */
#define SYS_CORE_CLOCK_HZ   72000000UL

/* ===== GPIO: On-board LED PC13 ===== */
#define LED_PORT            GPIOC
#define LED_PIN             13

/* ===== UARTs ===== */
#define UART1_BAUD          115200U   /* ESP32 link */
#define UART3_BAUD          115200U   /* PC log    */

/* ===== Protocol (must match bootloader) ===== */
enum {
    STX    = 0x55,
    TYPE_Q = 0x10,
};

enum {
    CMD_INFO   = 1,
    CMD_BOOT   = 2,
    CMD_ERASE  = 3,
    CMD_BEGIN  = 4,
    CMD_VERIFY = 5,
    CMD_RUN    = 6
};

#define ACK_BYTE            0xAA
#define NACK_BYTE           0xEE

/* ===== Boot Metadata (same layout as bootloader) ===== */
#define META_MAGIC          0x4D455441UL  /* 'META' */
#define MF_BOOT_REQ         (1u << 0)
#define MF_VALID            (1u << 1)
#define MF_IN_PROGRESS      (1u << 2)

#ifndef FLASH_64K
  #define META_BASE         0x0801FC00UL
#else
  #define META_BASE         0x0800FC00UL
#endif

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t crc32;
    uint32_t flags;
} bl_meta_t;


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_H */
