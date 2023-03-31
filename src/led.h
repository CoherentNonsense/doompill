// STM32f103 LED Library
// Simple wrapper around the onboard LED
// 
// Peripherals used:
// PC13 (duh)

#ifndef LED_H
#define LED_H

void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);

#endif
