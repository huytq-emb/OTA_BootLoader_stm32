/*
 * boot.c — Bootloader state machine (STM32F103C8T6 @72MHz, bare-metal)
 *
 * Responsibilities:
 *  - Early clock bring-up (HSE→PLL x9 = 72 MHz), basic System_Init
 *  - Decide: jump to APP or stay in bootloader (fail-safe rules)
 *  - UART1 framed protocol:  STX | TYPE | SEQ | LEN(LE16) | PAYLOAD | CRC16-CCITT
 *  - Commands (TYPE_Q): INFO / ERASE / BEGIN / VERIFY / RUN
 *  - Data path: receive FWUP header (TYPE_H), write chunks (TYPE_D), END (TYPE_E)
 *  - Metadata stored at the last flash page
 *
 * Notes:
 *  - CRC32 of app image matches crc32_update() (IEEE 802.3 reflected)
 *  - All flash ops require flash_unlock()/flash_lock() around them
 */

#include "boot.h"
#include <string.h>
#include <stdio.h>

/* =========================
 *  Clock / System Bring-up
 * ========================= */

/**
 * @brief       Initialize system clock to 72MHz using HSE and PLL
 * @param       None
 * @retval      None
 */
static void clock_init(void) {
    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }

    /* Flash: 2 wait states + prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    /* PLL: HSE * 9 = 72MHz */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC;              /* HSE as PLL source */
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    /* Bus prescalers: AHB=1, APB2=1, APB1=2 (36MHz) */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (0 << 4);                     /* AHB = /1 */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;          /* APB1 = /2 */
    RCC->CFGR |= (0 << 11);                    /* APB2 = /1 */

    /* Enable PLL and switch SYSCLK to PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
}

void System_Init(void) {
    /* Minimal reset to defaults */
    RCC->CR   |= RCC_CR_HSION;
    RCC->CFGR  = 0x00000000;
    RCC->CR   &= ~(RCC_CR_HSEON | RCC_CR_PLLON);
    RCC->CIR   = 0x00000000;

    clock_init();
    SystemCoreClock = 72000000UL;
}

/* =========================
 *  CRC16-CCITT for frames
 * ========================= */

/**
 * @brief       Calculate CRC16-CCITT checksum for frame validation
 * @param       data Pointer to data buffer
 * @param       len Length of data in bytes
 * @retval      16-bit CRC checksum
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len) {
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

/* =========================
 *  Metadata helpers
 * ========================= */

/**
 * @brief       Load default metadata values
 * @param       m Pointer to metadata structure to initialize
 * @retval      None
 */
void meta_load_defaults(bl_meta_t *m) {
    memset(m, 0, sizeof(*m));
    m->magic = META_MAGIC;
    m->flags = 0;
}

/**
 * @brief       Read metadata from flash memory
 * @param       m Pointer to metadata structure to fill
 * @retval      0 on success, -1 if metadata is invalid
 */
int meta_read(bl_meta_t *m) {
    memcpy(m, (void *)META_BASE, sizeof(*m));
    if (m->magic != META_MAGIC) {
        meta_load_defaults(m);
        return -1;
    }
    return 0;
}

/**
 * @brief       Write metadata to flash memory
 * @param       m Pointer to metadata structure to write
 * @retval      0 on success, -1 on error
 */
int meta_write(const bl_meta_t *m) {
    flash_unlock();
    if (flash_erase_page(META_BASE) != 0) { flash_lock(); return -1; }
    const int rc = flash_write(META_BASE, (const uint8_t *)m, sizeof(*m));
    flash_lock();
    return rc;
}

/**
 * @brief       Mark firmware update as in progress in metadata
 * @param       None
 * @retval      0 on success, -1 on error
 */
static int meta_mark_in_progress(void) {
    bl_meta_t m; meta_read(&m);
    m.magic = META_MAGIC;
    m.flags |= MF_IN_PROGRESS;
    m.flags &= ~MF_VALID;
    return meta_write(&m);
}

/* =========================
 *  Jump to Application
 * ========================= */

/**
 * @brief       Jump to application firmware after validation
 * @param       None
 * @retval      None (function never returns)
 */
static void jump_to_app(void) {
    uint32_t sp = *(__IO uint32_t *)(APP_BASE + 0);
    uint32_t pc = *(__IO uint32_t *)(APP_BASE + 4);

    /* Sanity: STM32F103C8T6 has 20KB SRAM: [0x20000000 .. 0x20005000) */
    if (sp < 0x20000000UL || sp > 0x20005000UL) return;

    /* Be nice: stop SysTick/UARTs, relocate VTOR, then jump */
    SysTick->CTRL = 0;
    USART1->CR1 = 0; USART3->CR1 = 0;

    SCB->VTOR = APP_BASE;
    __disable_irq();
    __set_MSP(sp);
    ((void (*)(void))pc)();   /* never returns */
}

/* =========================
 *  UART frame I/O helpers
 * ========================= */

/**
 * @brief       Read exact number of bytes from UART with timeout
 * @param       buf Buffer to store received data
 * @param       len Number of bytes to read
 * @param       to_ms Timeout in milliseconds
 * @retval      0 on success, -1 on timeout/error
 */
static int read_exact(uint8_t *buf, uint32_t len, uint32_t to_ms) {
    for (uint32_t i = 0; i < len; i++) {
        if (uart1_read_byte(&buf[i], to_ms) < 0) return -1;
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

/* =========================
 *  FW receive context
 * ========================= */

static fwup_hdr_t g_hdr;
static uint32_t   g_write_addr;
static uint32_t   g_bytes_written;
static uint32_t   g_running_crc;

/* =========================
 *  Process one frame
 * ========================= */

/**
 * @brief       Process one UART frame (command or data)
 * @param       None  
 * @retval      0 on success/idle, -1 on error, 1 on jump to app
 */
static int process_frame(void) {
    uint8_t b;

    /* Seek STX (non-blocking-ish): timeout 1s then idle */
    if (uart1_read_byte(&b, 1000) < 0) return 0;
    if (b != STX) return 0;  /* skip garbage */

    /* Header: TYPE(1), SEQ(1), LEN(LE16) */
    uint8_t hdr[1 + 1 + 2];
    if (read_exact(hdr, sizeof(hdr), 50) < 0) return -1;

    uint8_t  type = hdr[0];
    uint8_t  seq  = hdr[1];
    uint16_t len  = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);

    if (len > 1024) { send_nack(seq); return -1; }  /* payload guard */

    static uint8_t payload[1024];
    if (len) {
        if (read_exact(payload, len, 500) < 0) { send_nack(seq); return -1; }
    }

    /* Verify frame CRC16 (over STX..payload) */
    uint8_t framebuf[5 + 1024];
    framebuf[0] = STX; framebuf[1] = type; framebuf[2] = seq;
    framebuf[3] = (uint8_t)(len & 0xFF);
    framebuf[4] = (uint8_t)(len >> 8);
    if (len) memcpy(&framebuf[5], payload, len);

    uint8_t c16lsb, c16msb;
    if (read_exact(&c16lsb, 1, 50) < 0 || read_exact(&c16msb, 1, 50) < 0) { send_nack(seq); return -1; }

    const uint16_t rx_crc = (uint16_t)c16lsb | ((uint16_t)c16msb << 8);
    const uint16_t calc   = crc16_ccitt(framebuf, 5 + len);

    if (rx_crc != calc) {
        log_printf("[BL] CRC16 bad: rx=0x%04X calc=0x%04X\n", rx_crc, calc);
        send_nack(seq);
        return -1;
    }

    /* ---------- Valid frame -> handle by TYPE ---------- */

    if (type == TYPE_Q) {
        if (len != 1) { send_nack(seq); return -1; }
        const uint8_t cmd = payload[0];

        switch (cmd) {
        case CMD_INFO:
            /* ACK + print to PC for human */
            log_printf("[BL] INFO: ver=%lu size=%lu crc=0x%08lX flags=0x%08lX\n",
                       (unsigned long)g_hdr.ver, (unsigned long)g_hdr.size,
                       (unsigned long)g_hdr.crc32, (unsigned long)0);
            send_ack(seq);
            break;

        case CMD_ERASE:
            flash_unlock();
            if (flash_erase_app_region(APP_MAX_SIZE) != 0) { flash_lock(); send_nack(seq); break; }
            flash_lock();
            log_printf("[BL] ERASE done\n");
            send_ack(seq);
            break;

        case CMD_BEGIN:
            g_write_addr    = APP_BASE;
            g_bytes_written = 0;
            g_running_crc   = 0;
            if (meta_mark_in_progress() != 0) {
                log_printf("[BL][ERR] meta_write IN_PROGRESS failed!\n");
                send_nack(seq);
                return -1;
            }
            log_printf("[BL] BEGIN\n");
            send_ack(seq);
            break;

        case CMD_VERIFY: {
            /* Compute CRC32 over the exact g_hdr.size bytes */
            uint32_t crc = 0;
            uint8_t *p = (uint8_t *)APP_BASE;
            for (uint32_t off = 0; off < g_hdr.size; ) {
                uint32_t chunk = g_hdr.size - off;
                if (chunk > 512) chunk = 512;
                crc = crc32_update(crc, p + off, chunk);
                off += chunk;
            }

            log_printf("[BL] VERIFY calc=0x%08lX expect=0x%08lX\n",
                       (unsigned long)crc, (unsigned long)g_hdr.crc32);

            if (crc == g_hdr.crc32 && g_hdr.size > 0 && g_hdr.size <= APP_MAX_SIZE) {
                bl_meta_t m; meta_read(&m);
                m.magic   = META_MAGIC;
                m.version = g_hdr.ver;
                m.size    = g_hdr.size;
                m.crc32   = g_hdr.crc32;
                m.flags  |= MF_VALID;
                m.flags  &= ~(MF_IN_PROGRESS | MF_BOOT_REQ);

                if (meta_write(&m) == 0) {
                    log_printf("[BL] VERIFY OK. VALID set.\n");
                    send_ack(seq);
                    delay_ms(20);
                    log_printf("[BL] Auto-run APP...\n");
                    jump_to_app();           /* auto-run the new app */
                } else {
                    log_printf("[BL][ERR] meta_write failed!\n");
                    send_nack(seq);
                }
            } else {
                bl_meta_t m; meta_read(&m);
                m.flags &= ~MF_VALID;
                meta_write(&m);
                log_printf("[BL] VERIFY FAIL.\n");
                send_nack(seq);
            }
        } break;

        case CMD_RUN: {
            bl_meta_t m; meta_read(&m);
            if (m.flags & MF_VALID) {
                send_ack(seq);
                log_printf("[BL] RUN APP...\n");
                delay_ms(10);
                jump_to_app();
            } else {
                log_printf("[BL] RUN rejected (not VALID)\n");
                send_nack(seq);
            }
        } break;

        default:
            send_nack(seq);
            break;
        }
        return 1;
    }

    else if (type == TYPE_H) {
        if (len != sizeof(fwup_hdr_t)) { send_nack(seq); return -1; }
        memcpy(&g_hdr, payload, sizeof(g_hdr));

        if (memcmp(g_hdr.tag, "FWUP", 4) != 0)            { send_nack(seq); return -1; }
        if (g_hdr.size == 0 || g_hdr.size > APP_MAX_SIZE) { send_nack(seq); return -1; }

        /* Erase only the needed region (rounded up) */
        flash_unlock();
        if (flash_erase_app_region(g_hdr.size) != 0) { flash_lock(); send_nack(seq); return -1; }
        flash_lock();

        g_write_addr    = APP_BASE;
        g_bytes_written = 0;
        g_running_crc   = 0;

        (void)meta_mark_in_progress();

        log_printf("[BL] HEADER ver=%u size=%lu crc=0x%08lX\n",
                   g_hdr.ver, (unsigned long)g_hdr.size, (unsigned long)g_hdr.crc32);
        send_ack(seq);
        return 1;
    }

    else if (type == TYPE_D) {
        if (g_bytes_written >= g_hdr.size) { send_nack(seq); return -1; }

        /* Clamp to remaining to keep CRC correct; ignore extras beyond declared size */
        uint32_t to_write = len;
        if (g_bytes_written + to_write > g_hdr.size) {
            to_write = g_hdr.size - g_bytes_written;
        }

        /* Program to flash (halfword-padded inside flash_write) */
        flash_unlock();
        if (flash_write(g_write_addr, payload, to_write) != 0) { flash_lock(); send_nack(seq); return -1; }
        flash_lock();

        /* Update running CRC over exact received bytes */
        g_running_crc   = crc32_update(g_running_crc, payload, to_write);
        g_write_addr   += to_write;
        g_bytes_written += to_write;

        send_ack(seq);
        return 1;
    }

    else if (type == TYPE_E) {
        log_printf("[BL] END received: bytes=%lu / %lu\n",
                   (unsigned long)g_bytes_written, (unsigned long)g_hdr.size);
        send_ack(seq);
        return 1;
    }

    else {
        send_nack(seq);
        return -1;
    }
}

/* =========================
 *  Boot main loop
 * ========================= */

/**
 * @brief       Main bootloader entry point and decision logic
 * @param       None
 * @retval      None (may never return if jumping to app)
 */
void boot_main(void) {
    uart_init();
    log_printf("[BOOT] STM32F103 Bootloader @%lu Hz\n", (unsigned long)SYS_CORE_CLOCK_HZ);

    /* Decide: jump or stay (fail-safe rules) */
    bl_meta_t meta;
    const int have = (meta_read(&meta) == 0);
    if (!have) meta_load_defaults(&meta);

    const int stay =
        (!have) ||
        (meta.flags & MF_BOOT_REQ) ||
        (meta.flags & MF_IN_PROGRESS) ||
        !(meta.flags & MF_VALID);

    if (!stay) {
        log_printf("[BOOT] Valid app & no request -> jump\n");
        jump_to_app();
        /* If returns, fall through to stay */
    } else {
        log_printf("[BOOT] Stay in bootloader (flags=0x%08lX)\n", (unsigned long)meta.flags);
    }

    /* Main loop: process frames + minimal CLI (UART3) */
    log_printf("[BOOT] Awaiting UART1 frames...\n");
    for (;;) {
        (void)process_frame();

        /* Minimal CLI via UART3 RX */
        if (USART3->SR & USART_SR_RXNE) {
            uint8_t c = (uint8_t)USART3->DR;
            if (c == 'i') {
                bl_meta_t m; meta_read(&m);
                log_printf("[CLI] info: ver=%lu size=%lu crc=0x%08lX flags=0x%08lX\n",
                           (unsigned long)m.version, (unsigned long)m.size,
                           (unsigned long)m.crc32, (unsigned long)m.flags);
            } else if (c == 'e') {
                flash_unlock();
                (void)flash_erase_app_region(APP_MAX_SIZE);
                flash_lock();
                log_printf("[CLI] erase app region done\n");
            } else if (c == 'v') {
                uint32_t crc = 0;
                for (uint32_t off = 0; off < meta.size; ) {
                    uint32_t chunk = meta.size - off; if (chunk > 512) chunk = 512;
                    crc = crc32_update(crc, (uint8_t *)APP_BASE + off, chunk);
                    off += chunk;
                }
                log_printf("[CLI] verify calc=0x%08lX expect=0x%08lX\n",
                           (unsigned long)crc, (unsigned long)meta.crc32);
            } else if (c == 'b') {
                bl_meta_t m; meta_read(&m);
                m.flags |= MF_BOOT_REQ;
                meta_write(&m);
                NVIC_SystemReset();
            } else if (c == 'h') {
                log_printf("CLI: i=info e=erase v=verify b=boot h=help\n");
            }
        }
    }
}

/* =========================
 *  Reset handler entry
 * ========================= */

/**
 * @brief       Reset handler and system initialization entry point
 * @param       None
 * @retval      Never returns
 */
int main(void) {
    System_Init();
    boot_main();
    while (1) { }
}
