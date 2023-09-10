// here is a reference for the timing info i will be using
// (source: http://tinyvga.com/vga-timing/800x600@56Hz)
//
// SVGA 800x600 @ 60 Hz
//
// [horizontal timing]
// | scanline     | pixels | time (µs) |
// |-----------------------------------|
// | visible area | 800px  | 20.0      |
// | front porch  | 40px   |  1.0      |
// | sync pulse   | 128px  |  3.2      |
// | back porch   | 88px   |  2.2      |
// | whole line   | 1056px | 26.4      |
//
// [vertical timing]
// | scanline     | pixels | time (ms) |
// |-----------------------------------|
// | visible area | 600px  | 15.84     |
// | front porch  | 1px    |  0.0264   |
// | sync pulse   | 4px    |  0.1056   |
// | back porch   | 23px   |  0.6072   |
// | whole line   | 628px  | 16.5792   |
//
// i can't render the full 800x600, so instead, i will render 200x150, where
// each pixel will be made up of 4x4 physical pixels.

#include "vga.h"

#include "led.h"
#include "stm32.h"
#include "text.h"
#include "vector_table.h"
#include "types.h"


static u8 buffer[DISPLAY_WIDTH_BYTES];

static u32 current_row;

static bool waiting_for_vsync;


// sets the VSYNC pin to high
static void set_vsync_high(void) {
    GPIOA->BSRR = GPIO_BSRR_SET(1); 
}


// sets the VSYNC pin to low
static void set_vsync_low(void) {
    GPIOA->BSRR = GPIO_BSRR_RST(1);
}


static void init_timers(void) {
    // ========================= //
    //  Setup Horizontal Timing  //
    // ========================= //
    //
    // below is a timing diagram of a single row of VGA signal.
    // there are two important times that we must consider.
    //
    // (1*): tim1_cc_handler
    // when we near the end of the back porch, we need to prepare the DMA to
    // start sending pixels out though SPI1.
    //
    // (2*): dma1_channel3_handler
    // after the the DMA is finished sending pixels, we should prepare the
    // buffer for the next line (this is unused for now since the user has
    // access to the entire buffer, but having a raserizer would be cool)
    //
    //                             (1*)                  (2*)
    // [-- sync --][-- back porch --][-- display pixels --][-- front porch --]
    //     3.2µs         2.20µs             20.0µs              1.00µs      
    //    128 clk        88 clk             800 clk             40 clk
    //
    // [--------------------------- total -----------------------------------]
    //                             26.40µs
    //                             1056 clk
    //
    // Channel1: used to output HSYNC (when timer is less than 128 clk)
    // ‾‾‾‾‾‾‾‾‾‾‾\___________________________________________________________
    //
    // Channel2: used to trigger (1*) (when timer is less than 128 + 88 = 216)
    // ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________________________________________
    //
    // each row will be run 628 times per frame.
    //
    // clock cycles (clk) are calculated based on a 40GHz system clock:
    // f(x) = 40GHz * (1s / 1,000,000µs) * (x)µs
    // e.g f(3.2) = 40GHz * (1s / 1,000,000µs) * 3.2µs = 128 clk
   
    const u32 HORIZONTAL_TOTAL_CLK = 2048;
    const u32 CHANNEL1_PULSE_CLK = 144;
    const u32 CHANNEL2_PULSE_CLK = 400;

    // setup TIM1
    TIM1->ARR = HORIZONTAL_TOTAL_CLK;

    // setup TIM1::channel1
    TIM1->CCER = TIM_CCER_CC1E;         // enable compare channel 1 output
    TIM1->BDTR = TIM_BDTR_MOE;          // main output (for the hsync PA8)
    TIM1->CCR1 = CHANNEL1_PULSE_CLK;    // from description above
    TIM1->CCMR1 = TIM_CCMR1_OCM1_PWM1;  // pulse until we reach CCR1

    // setup TIM1::channel2
    TIM1->CCR2 = CHANNEL2_PULSE_CLK;    // from description above
    TIM1->DIER = TIM_DIER_CC2IE;        // set interrupt (1*)
}



// ============ //
//  Interrupts  //
// ============ //
__attribute__((section(".ramtext")))
void tim1_cc_handler(void) {
    if (current_row < 600) {         // PIXELS START    
        DMA1->IFCR = DMA_IFCR_CTCIF3;
        DMA1->CMAR3 = (u32)buffer;
        DMA1->CCR3 = 0x93;

    } else if (current_row == 601) { // VSYNC START 
        waiting_for_vsync = false;
        set_vsync_high();
 
    } else if (current_row == 603) { // VSYNC END
        set_vsync_low();
    }

    TIM1->SR = 0;
    current_row += 1;
    current_row %= 625;
}

__attribute__((section(".ramtext")))
void dma1_channel3_handler(void) {
    DMA1->CCR3 = 0x92;
    DMA1->IFCR = DMA_IFCR_CTCIF3;
    DMA1->CNDTR3 = DISPLAY_WIDTH_BYTES;

    text_rasterize(buffer, (current_row >> 0) % 8);
}



// ============ //
//  public API  //
// ============ //
void vga_init() {
    // enable port A, TIM1, SPI1
    RCC->APB2ENR |= (
        RCC_APB2ENR_IOPAEN |
        RCC_APB2ENR_TIM1EN |
        RCC_APB2ENR_SPI1EN
    );

    // enable DMA1
    RCC->AHBENR |= (RCC_AHBENR_DMA1);

    // setup PA1 (VSYNC) and PA7 (DATA)
    GPIOA->CRL &= ~(GPIO_CRL(1) | GPIO_CRL(7));
    GPIOA->CRL |= (
        GPIO_CRL_MODE_50MHz(1) | GPIO_CRL_CNF_OUTPUT_PUSH_PULL(1) |
        GPIO_CRL_MODE_50MHz(7) | GPIO_CRL_CNF_ALT_PUSH_PULL(7)
    );

    // setup PA8 (HSYNC)
    GPIOA->CRH &= ~(GPIO_CRH(8));
    GPIOA->CRH |= (GPIO_CRH_MODE_50MHz(8) | GPIO_CRH_CNF_ALT_PUSH_PULL(8));
 
    // setup DMA1
    DMA1->CCR3 = (
        DMA_CCR_MINC |
        DMA_CCR_DIR_FROM_MEMORY |
        DMA_CCR_TCIE |
        DMA_CCR_PL_VERY_HIGH
    );
    DMA1->CPAR3 = (u32)&(SPI1->DR);
    DMA1->CNDTR3 = DISPLAY_WIDTH_BYTES;

    // setup SPI1
    // we will send pixel data at 40 / 2 = 10Mhz
    SPI1->CR1 = (
        SPI_CR1_SPE |
        SPI_CR1_MSTR |
        SPI_CR1_BR_DIV_2
    );
    SPI1->CR2 = SPI_CR2_TXDMAEN;

    init_timers();

    // setup interrupts
    NVIC->IPR[TIM1_CC_IRQn] = 0 << 4;
    NVIC->ISER[(u32)(TIM1_CC_IRQn >> 5)] = (1 << ((u32)(TIM1_CC_IRQn) & 0b11111));

    NVIC->IPR[DMA1_Channel3_IRQn] = 15 << 4;
    NVIC->ISER[(u32)(DMA1_Channel3_IRQn >> 5)] = (1 << ((u32)(DMA1_Channel3_IRQn) & 0b11111));

    // start timer
    TIM1->CR1 = TIM_CR1_CEN;
}


void vga_vsync() {
    waiting_for_vsync = true;
    while (waiting_for_vsync) {
        __asm__("wfi");
    }
}
