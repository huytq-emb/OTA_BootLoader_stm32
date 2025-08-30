#ifndef BOOT_H
#define BOOT_H
/**
 * Bootloader public header (bare-metal, no HAL)
 *
 * Target     : STM32F103C8T6 @ 72 MHz
 * Links      : UART1 <-> ESP32 (protocol + ACK/NACK), UART3 -> PC log
 * Flash map  : Boot (16 KB) | App | Metadata (end of Flash)
*/

#include <stdint.h>
#include <stdarg.h>
#include "stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========= Build-time options ========= */
/* If build for 64 KB chip (F103C8 64KB) */
// #define FLASH_64K

/* ========= Clock parameters ========= */
#define SYS_CORE_CLOCK_HZ   (72000000UL)

/* ========= Flash Map =========
   (128 KB):
     Bootloader : 0x0800_0000 - 0x0800_3FFF (16 KB)
     Application: 0x0800_4000 - 0x0801_FBFF
     Metadata   : 0x0801_FC00 - 0x0801_FFFF (last page, 1 KB)

   (64 KB):
     Metadata   : 0x0800_FC00 - 0x0800_FFFF
*/
#define FLASH_BASE_ADDR     (0x08000000UL)
#define BOOT_BASE           (0x08000000UL)
#define BOOT_SIZE           (16UL * 1024UL)
#define APP_BASE            (0x08004000UL)

#ifndef FLASH_64K
  #define FLASH_TOTAL_SIZE  (128UL * 1024UL)
  #define META_BASE         (0x0801FC00UL)
#else
  #define FLASH_TOTAL_SIZE  (64UL  * 1024UL)
  #define META_BASE         (0x0800FC00UL)
#endif

/*
 * FLASH_PAGE_SIZE is defined in another file (stm32_hal_flash) .
 */
// #define FLASH_PAGE_SIZE   (1024UL)

#define APP_END             (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE - FLASH_PAGE_SIZE - 1UL)
#define APP_MAX_SIZE        (APP_END - APP_BASE + 1UL)

/* ========= Metadata ========= */
#define META_MAGIC          (0x4D455441UL) /* 'META' */

#define MF_BOOT_REQ         (1u << 0)  /* Ứng dụng yêu cầu vào bootloader */
#define MF_VALID            (1u << 1)  /* Ảnh app đã verify OK */
#define MF_IN_PROGRESS      (1u << 2)  /* Đang cập nhật (chưa hoàn tất) */
#define MF_ROLLBACK         (1u << 3)  /* (tùy chọn) yêu cầu rollback */

typedef struct {
    uint32_t magic;    /* = META_MAGIC */
    uint32_t version;  /* phiên bản FW đang cài */
    uint32_t size;     /* kích thước ảnh app (bytes) */
    uint32_t crc32;    /* CRC-32 của ảnh app */
    uint32_t flags;    /* cờ trạng thái (MF_*) */
} bl_meta_t;

/* ========= FW Header từ ESP32 (gói "FWUP") ========= */
typedef struct __attribute__((packed)) {
    uint8_t  tag[4];   /* "FWUP" */
    uint16_t ver;      /* little-endian */
    uint32_t size;     /* tổng bytes của ảnh FW */
    uint32_t crc32;    /* CRC-32 của ảnh FW */
    uint32_t flags;    /* cờ/đặt chỗ */
} fwup_hdr_t;

/* ========= UART Protocol ========= */
enum {
    STX   = 0x55,
    TYPE_H = 0x01, /* Header */
    TYPE_D = 0x02, /* Data */
    TYPE_E = 0x03, /* End  */
    TYPE_Q = 0x10  /* Command */
};

enum {
    CMD_INFO   = 1,
    CMD_BOOT   = 2,
    CMD_ERASE  = 3,
    CMD_BEGIN  = 4,
    CMD_VERIFY = 5,
    CMD_RUN    = 6
};

#define ACK_BYTE  0xAA
#define NACK_BYTE 0xEE

/* ========= UART pins & baud =========
 * USART1: PA9  (TX1), PA10 (RX1)   -> ESP32
 * USART3: PB10 (TX3), PB11 (RX3)   -> PC Log
 */
#define UART1_BAUD  (115200U)
#define UART3_BAUD  (115200U)

/* ========= API ========= */
/* uart.c */
void uart_init(void);
void uart1_write_byte(uint8_t b);
void uart1_write(const uint8_t *p, uint32_t n);
int  uart1_read_byte(uint8_t *out, uint32_t timeout_ms); /* trả -1 nếu timeout */
void uart3_write_str(const char *s);
void log_printf(const char *fmt, ...);

/* crc32.c */
uint32_t crc32_update(uint32_t crc, const void *data, uint32_t len);

/* flash.c */
void flash_unlock(void);
void flash_lock(void);
int  flash_wait_busy(void);
int  flash_erase_page(uint32_t page_addr);
int  flash_write_halfword(uint32_t addr, uint16_t half);
int  flash_write(uint32_t addr, const uint8_t *data, uint32_t len); /* len lẻ: byte cuối pad 0xFF */
int  flash_erase_app_region(uint32_t bytes); /* xóa [APP_BASE .. APP_BASE+bytes) theo trang */

/* boot.c (state machine + metadata + jump) */
void boot_main(void);

/* utils (trong boot.c) */
void delay_ms(uint32_t ms);

/* Expose metadata helpers (phục vụ CLI/debug) */
int  meta_read(bl_meta_t *m);
int  meta_write(const bl_meta_t *m);
void meta_load_defaults(bl_meta_t *m);

/* ========= Safe region checks ========= */
/* Kiểm tra (addr,len) có nằm hoàn toàn trong vùng APP hay không */
static inline int within_app_region(uint32_t addr, uint32_t len) {
    if (addr < APP_BASE) return 0;
    if ((addr + len - 1U) > APP_END) return 0;
    return 1;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BOOT_H */
