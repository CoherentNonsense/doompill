// STM32f103 VGA Library
// Draws a pixel buffer to a display.
//
// Peripherals used:
// PA1 -> VSYNC (VGA pin 14)
// PA8 -> HSYNC (VGA pin 13)
// PA7 -> Data  (VGA pin 1(red), 2(green), or 3(blue))
// 
// TIM1 -> Handle VGA timing
// DMA1/SPI1 -> Used to output data to PA7

#ifndef VGA_H
#define VGA_H

#include "types.h"

#define DISPLAY_WIDTH (800)
#define DISPLAY_HEIGHT (600)
#define DISPLAY_WIDTH_BYTES (DISPLAY_WIDTH / 8)

// the buffer is a DISPLAY_WIDTH * DISPLAY_HEIGHT array of bits where each bit
// represents a pixel on the screen.
void vga_init();

// wait until the next frame
void vga_vsync();

#endif // !VGA_H
