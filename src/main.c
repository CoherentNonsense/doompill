#include "led.h"
#include "stm32.h"
#include "vector_table.h"
#include "vga.h"


static u8 buffer[DISPLAY_HEIGHT][DISPLAY_WIDTH_BYTES];

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

static void set_sysclk_to_40MHz(void) {
    // turn on HSE
    RCC->CR |= (RCC_CR_HSEON);
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    // add flash latency and prefetch (since flash can't keep up now)
    FLASH->ACR &= ~(FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
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

void main() {

    set_sysclk_to_40MHz();

    led_init();

    vga_init(buffer);

    while (true) {
        if (GPIOA->IDR & 1 << 7) {
            led_on();
        } else {
            led_off();
        }
    }

}
