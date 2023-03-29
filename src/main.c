#include "stm32.h"
#include "vector_table.h"


static void set_sysclk_to_72MHz(void) {
    // turn on HSE
    RCC->CR &= ~(RCC_CR_HSEON);
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

static void start_vsync_timer(void) {
    // enable TIM2 clock gate
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Reset CR1
    TIM2->CR1 = 0;

    // set set prescalar for TIM2 (72MHz / (7199 + 1) = 10KHz)
    TIM2->PSC = 7199;

    // count to 10000 (1 tps)
    TIM2->ARR = 10000;
    TIM2->DIER |= TIM_DIER_UIE;

    // set interupt priority
    NVIC->IPR[TIM2_IRQn] = 0;

    // enable interupt
	//NVIC->ISER[((u32)(TIM2_IRQn) >> 5)] = (1 << ((u32)(TIM2_IRQn) & 0x1f));
	NVIC->ISER[0] |= (1 << TIM2_IRQn);

    // start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

volatile bool led_state;
void tim2_handler(void) {
    GPIOC->BSRR = led_state ? GPIO_BSRR_SET(13) : GPIO_BSRR_RST(13);
    led_state ^= 1;
    TIM2->SR = ~(TIM_SR_UIF);
}

void main() {

    set_sysclk_to_72MHz();

    // Enable port C clock gate.
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // configure GPIO C pin 13 as output.
    GPIOC->CRH &= ~(GPIO_CRH_CNF(13) | GPIO_CRH_MODE(13));
    GPIOC->CRH |= (GPIO_CRH_CNF_OUTPUT_PUSH_PULL(13) | GPIO_CRH_MODE(13));

    start_vsync_timer();

    while (true) {}

}
