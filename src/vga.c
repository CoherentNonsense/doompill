// here is a reference for the timing info i will be using
// (source: http://tinyvga.com/vga-timing/800x600@56Hz)
//
// SVGA 800x600 @ 56 Hz
//
// [horizontal timing]
// | scanline     | pixels | time (µs) |
// |-----------------------------------|
// | visible area | 800px  | 22.222222 |
// | front porch  | 24px   |  0.666667 |
// | sync pulse   | 72px   |  2.0      |
// | back porch   | 128px  |  3.555556 |
// | whole line   | 1024px | 28.444444 |
//
// [vertical timing]
// | scanline     | pixels | time (ms) |
// |-----------------------------------|
// | visible area | 600px  | 17.066667 |
// | front porch  | 1px    |  0.028444 |
// | sync pulse   | 2px    |  0.056889 |
// | back porch   | 22px   |  0.625778 |
// | whole line   | 625px  | 17.777778 |
//
// i can't render the full 800x600, so instead, i will render 200x150, where
// each pixel will be made up of 4x4 physical pixels.

#include "vga.h"

#include "led.h"
#include "stm32.h"
#include "vector_table.h"
#include "types.h"


static volatile u8 buffer[DISPLAY_WIDTH_BYTES] = {
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
};
static u32 current_row;

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
    //    2.00µs         3.55µs             22.222µs             0.667µs      
    //    144 clk        256 clk            1560 clk             48 clk
    //
    // [--------------------------- total -----------------------------------]
    //                             28.44µs
    //                             2048 clk
    //
    // Channel1: used to output HSYNC (when timer is less than 144 clk)
    // ‾‾‾‾‾‾‾‾‾‾‾\___________________________________________________________
    //
    // Channel2: used to trigger (1*) (when timer is less than 144 + 256 = 400)
    // ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________________________________________
    //
    // each row will be done 625 times per frame.
    //
    // clock cycles (clk) are calculated based on a 72GHz SYSCLK:
    // f(x) = 72GHz * (1s / 1,000,000µs) * (x)µs
    // e.g f(2) = 72GHz * (1s / 1,000,000) * 2µs = 144 clk
   
    const u32 HORIZONTAL_TOTAL_CLK = 1056;
    const u32 CHANNEL1_PULSE_CLK = 128;
    const u32 CHANNEL2_PULSE_CLK = 216; // HACK: was 216

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
    TIM1->SR = 0;

    if (current_row < 600) {         // PIXELS START
        DMA1->IFCR = DMA_IFCR_CTCIF3;
        DMA1->CMAR3 = (u32)&buffer[0];
        DMA1->CCR3 = 0x93;
    } else if (current_row == 600) {
    } if (current_row == 601) { // VSYNC START
        set_vsync_high();
    } else if (current_row == 605) { // VSYNC END
        set_vsync_low();
    }

    current_row += 1;
    current_row %= 628;
}


// TODO: use a rasterizer and put it here
__attribute__((section(".ramtext")))
void dma1_channel3_handler(void) {
    DMA1->CCR3 = 0x92;
    DMA1->IFCR = DMA_IFCR_CTCIF3;
    DMA1->CNDTR3 = DISPLAY_WIDTH_BYTES;
}



// ============ //
//  public API  //
// ============ //
void vga_init(u8 framebuffer[DISPLAY_HEIGHT][DISPLAY_WIDTH_BYTES]) {
    // keep reference to framebuffer
    //buffer = *framebuffer;

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
    // we will send pixel data at 40 / 4 = 10Mhz
    SPI1->CR1 = (
        SPI_CR1_SPE |
        SPI_CR1_MSTR |
        SPI_CR1_BR_DIV_4
    );
    SPI1->CR2 = SPI_CR2_TXDMAEN;

    init_timers();

    // setup interrupts
    NVIC->IPR[TIM1_CC_IRQn] = 0b0000;
    NVIC->ISER[(u32)(TIM1_CC_IRQn >> 5)] = (1 << ((u32)(TIM1_CC_IRQn) & 0b11111));

    NVIC->IPR[DMA1_Channel3_IRQn] = 0b0001;
    NVIC->ISER[(u32)(DMA1_Channel3_IRQn >> 5)] = (1 << ((u32)(DMA1_Channel3_IRQn) & 0b11111));

    // start timer
    TIM1->CR1 = TIM_CR1_CEN;
}
