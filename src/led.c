#include "led.h"

#include "stm32.h"


static bool led_state;



// ============ //
//  Public API  //
// ============ //
void led_init(void) {
    // turn on peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // setup PC13
    GPIOC->CRH &= ~(GPIO_CRH_CNF(13) | GPIO_CRH_MODE(13));
    GPIOC->CRH |= (GPIO_CRH_CNF_OUTPUT_PUSH_PULL(13) | GPIO_CRH_MODE_50MHz(13));
}


void led_on(void) {
    led_state = true;
    GPIOC->BSRR = GPIO_BSRR_RST(13);
}


void led_off(void) {
    led_state = false;
    GPIOC->BSRR = GPIO_BSRR_SET(13);
}


void led_toggle(void) {
    led_state ^= 1;
    led_state ? led_on() : led_off();
}
