/*
 * app_system.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include "app.h"
#include "app_system.h"

/**
 * @brief       Initialize system clock to 72MHz using HSE and PLL
 * @param       None
 * @retval      None
 */
static void clock_init(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC;      // HSE
    RCC->CFGR |= RCC_CFGR_PLLMULL9;    // x9 = 72MHz

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 = 36MHz

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

/**
 * @brief       Initialize system (clock, reset controller)
 * @param       None
 * @retval      None
 */
void System_Init(void) {
    RCC->CR   |= RCC_CR_HSION;
    RCC->CFGR  = 0x00000000;
    RCC->CR   &= ~(RCC_CR_HSEON | RCC_CR_PLLON);
    RCC->CIR   = 0x00000000;

    clock_init();
    SystemCoreClock = SYS_CORE_CLOCK_HZ;
}

/**
 * @brief       Relocate vector table to application base address
 * @param       None
 * @retval      None
 */
void relocate_vtor(void) {
    SCB->VTOR = APP_BASE;
    __DSB(); __ISB();
}

