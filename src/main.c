#include "led.h"
#include "stm32.h"
#include "text.h"
#include "vector_table.h"
#include "vga.h"


static void set_sysclk_to_72MHz(void) {
    // turn on HSE
    RCC->CR |= (RCC_CR_HSEON);
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    // add flash latency and prefetch (since flash can't keep up now)
    FLASH->ACR &= ~(FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
    FLASH->ACR |= (FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE);

    // setup PLL to 72MHz (8MHz HSE * 9 multiplier)
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE);
    RCC->CFGR |= (RCC_CFGR_PLLMUL_9 | RCC_CFGR_PLLSRC_HSE);

    // start PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // set PLL as sysclk
    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
}
/*
static void set_sysclk_to_60MHz(void) {
    // turn on HSE
    RCC->CR |= (RCC_CR_HSEON);
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    // add flash latency and prefetch (since flash can't keep up now)
    FLASH->ACR &= ~(FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
    FLASH->ACR |= (FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE);

    // setup PLL to 60MHz (8MHz HSE / 2 * 15 multiplier)
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= (RCC_CFGR_PLLMUL_15 | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE);

    // start PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // set PLL as sysclk
    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
}

static void set_sysclk_to_40MHz(void) {
    // turn on HSE
    RCC->CR |= (RCC_CR_HSEON);
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    // add flash latency and prefetch (since flash can't keep up now)
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |= (FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE);

    // setup PLL to 40MHz (8MHz HSE * 5 multiplier)
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE);
    RCC->CFGR |= (RCC_CFGR_PLLMUL_5 | RCC_CFGR_PLLSRC_HSE);

    // start PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // set PLL as sysclk
    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
}
*/

static char text[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 !@#$%^&*()_+-=[]{}\\|,./<>?";

void main() {

    set_sysclk_to_72MHz();

    led_init();

    vga_init();

    text_print(text, sizeof(text) - 1);

    while (true) {
        //vga_vsync();
    }

}
