// NOTE: this file only contains the registers I've needed for this project
//       and was intended for me to learn how to read the datasheet so it
//       may not be the best way to do things.
//
// register details can be found in the reference manual.
// ST RM0008 Reference manual
//
// the NVIC details can be found in this other manual (why?? idk)
// ST PM0056 Programming manual


#ifndef STM32_H
#define STM32_H

#include "types.h"


// some custom types for volatile memory
typedef volatile u8  __reg8;
typedef volatile u16 __reg16;
typedef volatile u32 __reg;


// =========================================== //
//  NVIC (Nested Vector Interrupt Controller)  //
// =========================================== //
struct NVIC_type {
    __reg ISER[3];       // 0x000
    __reg RES0[29];
    __reg ICER[3];       // 0x080
    __reg RES1[29];
    __reg ISPR[3];       // 0x100
    __reg RES2[29];
    __reg ICPR[3];       // 0x180
    __reg RES3[29];
    __reg IABR[3];       // 0x200
    __reg RES4[61];
    __reg8 IPR[21 * 4];  // 0x300
    __reg RES5[683];
    __reg STIR;          // 0xe00
};


// unique interrupt IDs that index into the NVIC registers which map to the vector table defined in vector_table.c
enum IRQn {
    NonMaskableInt_IRQn         = -14,    // 2 Non Maskable Interrupt
    MemoryManagement_IRQn       = -12,    // 4 Cortex-M3 Memory Management Interrupt
    BusFault_IRQn               = -11,    // 5 Cortex-M3 Bus Fault Interrupt
    UsageFault_IRQn             = -10,    // 6 Cortex-M3 Usage Fault Interrupt
    SVCall_IRQn                 = -5,     // 11 Cortex-M3 SV Call Interrupt
    DebugMonitor_IRQn           = -4,     // 12 Cortex-M3 Debug Monitor Interrupt
    PendSV_IRQn                 = -2,     // 14 Cortex-M3 Pend SV Interrupt
    SysTick_IRQn                = -1,     // 15 Cortex-M3 System Tick Interrupt
    WWDG_IRQn                   = 0,      // Window WatchDog Interrupt
    PVD_IRQn                    = 1,      // PVD through EXTI Line detection Interrupt
    TAMPER_IRQn                 = 2,      // Tamper Interrupt
    RTC_IRQn                    = 3,      // RTC global Interrupt
    FLASH_IRQn                  = 4,      // FLASH global Interrupt
    RCC_IRQn                    = 5,      // RCC global Interrupt
    EXTI0_IRQn                  = 6,      // EXTI Line0 Interrupt
    EXTI1_IRQn                  = 7,      // EXTI Line1 Interrupt
    EXTI2_IRQn                  = 8,      // EXTI Line2 Interrupt
    EXTI3_IRQn                  = 9,      // EXTI Line3 Interrupt
    EXTI4_IRQn                  = 10,     // EXTI Line4 Interrupt
    DMA1_Channel1_IRQn          = 11,     // DMA1 Channel 1 global Interrupt
    DMA1_Channel2_IRQn          = 12,     // DMA1 Channel 2 global Interrupt
    DMA1_Channel3_IRQn          = 13,     // DMA1 Channel 3 global Interrupt
    DMA1_Channel4_IRQn          = 14,     // DMA1 Channel 4 global Interrupt
    DMA1_Channel5_IRQn          = 15,     // DMA1 Channel 5 global Interrupt
    DMA1_Channel6_IRQn          = 16,     // DMA1 Channel 6 global Interrupt
    DMA1_Channel7_IRQn          = 17,     // DMA1 Channel 7 global Interrupt
    ADC1_2_IRQn                 = 18,     // ADC1 and ADC2 global Interrupt
    CAN1_TX_IRQn                = 19,     // USB Device High Priority or CAN1 TX Interrupts
    CAN1_RX0_IRQn               = 20,     // USB Device Low Priority or CAN1 RX0 Interrupts
    CAN1_RX1_IRQn               = 21,     // CAN1 RX1 Interrupt
    CAN1_SCE_IRQn               = 22,     // CAN1 SCE Interrupt
    EXTI9_5_IRQn                = 23,     // External Line[9:5] Interrupts
    TIM1_BRK_IRQn               = 24,     // TIM1 Break Interrupt
    TIM1_UP_IRQn                = 25,     // TIM1 Update Interrupt
    TIM1_TRG_COM_IRQn           = 26,     // TIM1 Trigger and Commutation Interrupt
    TIM1_CC_IRQn                = 27,     // TIM1 Capture Compare Interrupt
    TIM2_IRQn                   = 28,     // TIM2 global Interrupt
    TIM3_IRQn                   = 29,     // TIM3 global Interrupt
    TIM4_IRQn                   = 30,     // TIM4 global Interrupt
    I2C1_EV_IRQn                = 31,     // I2C1 Event Interrupt
    I2C1_ER_IRQn                = 32,     // I2C1 Error Interrupt
    I2C2_EV_IRQn                = 33,     // I2C2 Event Interrupt
    I2C2_ER_IRQn                = 34,     // I2C2 Error Interrupt
    SPI1_IRQn                   = 35,     // SPI1 global Interrupt
    SPI2_IRQn                   = 36,     // SPI2 global Interrupt
    USART1_IRQn                 = 37,     // USART1 global Interrupt
    USART2_IRQn                 = 38,     // USART2 global Interrupt
    USART3_IRQn                 = 39,     // USART3 global Interrupt
    EXTI15_10_IRQn              = 40,     // External Line[15:10] Interrupts
    RTCAlarm_IRQn               = 41,     // RTC Alarm through EXTI Line Interrupt
    OTG_FS_WKUP_IRQn            = 42,     // USB OTG FS WakeUp from suspend through EXTI Line Int
    TIM5_IRQn                   = 50,     // TIM5 global Interrupt
    SPI3_IRQn                   = 51,     // SPI3 global Interrupt
    UART4_IRQn                  = 52,     // UART4 global Interrupt
    UART5_IRQn                  = 53,     // UART5 global Interrupt
    TIM6_IRQn                   = 54,     // TIM6 global Interrupt
    TIM7_IRQn                   = 55,     // TIM7 global Interrupt
    DMA2_Channel1_IRQn          = 56,     // DMA2 Channel 1 global Interrupt
    DMA2_Channel2_IRQn          = 57,     // DMA2 Channel 2 global Interrupt
    DMA2_Channel3_IRQn          = 58,     // DMA2 Channel 3 global Interrupt
    DMA2_Channel4_IRQn          = 59,     // DMA2 Channel 4 global Interrupt
    DMA2_Channel5_IRQn          = 60,     // DMA2 Channel 5 global Interrupt
    ETH_IRQn                    = 61,     // Ethernet global Interrupt
    ETH_WKUP_IRQn               = 62,     // Ethernet Wakeup through EXTI line Interrupt
    CAN2_TX_IRQn                = 63,     // CAN2 TX Interrupt
    CAN2_RX0_IRQn               = 64,     // CAN2 RX0 Interrupt
    CAN2_RX1_IRQn               = 65,     // CAN2 RX1 Interrupt
    CAN2_SCE_IRQn               = 66,     // CAN2 SCE Interrupt
    OTG_FS_IRQn                 = 67      // USB OTG FS global Interrupt
};

#define NVIC_BASE (0xe000E100)
#define NVIC ((struct NVIC_type*)NVIC_BASE)



// ====================== //
//  FLASH (Flash Memory)  //
// ====================== //
struct FLASH_type {
    __reg ACR;    // 0x00
};

#define FLASH_BASE (0x40022000)
#define FLASH ((struct FLASH_type*)FLASH_BASE)

// FLASH::ACR (access control register)
#define FLASH_ACR_LATENCY   (0b111 << 0)
#define FLASH_ACR_LATENCY_0 (0b000 << 0)
#define FLASH_ACR_LATENCY_1 (0b001 << 0)
#define FLASH_ACR_LATENCY_2 (0b010 << 0)

#define FLASH_ACR_PRFTBE (0b1 << 4)



// =============================== //
//  RCC (Reset and Clock Control)  //
// =============================== //
struct RCC_type {
    __reg CR;         // 0x00
    __reg CFGR;       // 0x04
    __reg CIR;        // 0x08
    __reg APB2RSTR;   // 0x0c
    __reg APB1RSTR;   // 0x10
    __reg AHBENR;     // 0x14
    __reg APB2ENR;    // 0x18
    __reg APB1ENR;    // 0x1c
};

#define RCC_BASE (0x40021000)
#define RCC ((struct RCC_type*)RCC_BASE)

// RCC::CR (control reegister)
#define RCC_CR_PLLRDY (0b1 << 25)
#define RCC_CR_PLLON  (0b1 << 24)
#define RCC_CR_HSEON  (0b1 << 16)
#define RCC_CR_HSERDY (0b1 << 17)

// RCC::CFGR (configure register)
#define RCC_CFGR_PLLMUL (0b1111 << 18)
#define RCC_CFGR_PLLMUL_2  (0b0000 << 18)
#define RCC_CFGR_PLLMUL_5  (0b0011 << 18)
#define RCC_CFGR_PLLMUL_8  (0b0110 << 18)
#define RCC_CFGR_PLLMUL_9  (0b0111 << 18)
#define RCC_CFGR_PLLMUL_12 (0b1010 << 18)
#define RCC_CFGR_PLLMUL_15 (0b1101 << 18)
#define RCC_CFGR_PLLMUL_16 (0b1111 << 18)

#define RCC_CFGR_PLLXTPRE (0b1 << 17)

#define RCC_CFGR_PLLSRC (0b1 << 16)
#define RCC_CFGR_PLLSRC_HSI_DIV_2 (0b0 << 16)
#define RCC_CFGR_PLLSRC_HSE (0b1 << 16)

#define RCC_CFGR_SWS (0b11 << 2)
#define RCC_CFGR_SWS_HSI (0b00 << 2)
#define RCC_CFGR_SWS_HSE (0b01 << 2)
#define RCC_CFGR_SWS_PLL (0b10 << 2)

#define RCC_CFGR_SW (0b11 << 0)
#define RCC_CFGR_SW_HSI (0b00 << 0)
#define RCC_CFGR_SW_HSE (0b01 << 0)
#define RCC_CFGR_SW_PLL (0b10 << 0)

// RCC:AHBENR (AHB peripheral clock enable register)
#define RCC_AHBENR_DMA1 (0b1 << 0)

// RCC::APB1 (APB1 peripheral clock enable register)
#define RCC_APB1ENR_TIM2EN (0b1 << 0)

// RCC::APB2 (APB2 peripheral clock enable register)
#define RCC_APB2ENR_SPI1EN (0b1 << 12)
#define RCC_APB2ENR_TIM1EN (0b1 << 11)
#define RCC_APB2ENR_IOPCEN (0b1 << 4)
#define RCC_APB2ENR_IOPBEN (0b1 << 3)
#define RCC_APB2ENR_IOPAEN (0b1 << 2)
#define RCC_APB2ENR_AFIOEN (0b1 << 0)



// ============================ //
//  DMA (Direct Memory Access)  //
// ============================ //
struct DMA_type {
    __reg ISR;      // 0x00
    __reg IFCR;     // 0x04

    __reg CCR1;     // 0x08
    __reg CNDTR1;   // 0x0c
    __reg CPAR1;    // 0x10
    __reg CMAR1;    // 0x14
    
    __reg RES0;

    __reg CCR2;     // 0x1c
    __reg CNDTR2;   // 0x20
    __reg CPAR2;    // 0x24
    __reg CMAR2;    // 0x28

    __reg RES1;

    __reg CCR3;     // 0x30
    __reg CNDTR3;   // 0x34
    __reg CPAR3;    // 0x38
    __reg CMAR3;    // 0x3c
    
    __reg RES2;
};

#define DMA1_BASE (0x40020000)
#define DMA1 ((struct DMA_type*)DMA1_BASE)

// DMA::IFCR (interrupt flag clear register)
#define DMA_IFCR_CTCIF3 (0b1 << 9)
#define DMA_IFCR_CGIF3  (0b1 << 8)

// DMA::CCRx (channel x configuration register)
#define DMA_CCR_PL (0b11 << 12)
#define DMA_CCR_PL_HIGH (0b10 << 12)
#define DMA_CCR_PL_VERY_HIGH (0b11 << 12)
#define DMA_CCR_MINC (0b1 << 7)
#define DMA_CCR_DIR_FROM_MEMORY (0b1 << 4)
#define DMA_CCR_TCIE (0b1 << 1)
#define DMA_CCR_EN (0b1 << 0)

// SPI (serial peripheral interface)
struct SPI {
    __reg CR1;
    __reg CR2;
    __reg SR;
    __reg DR;
};

#define SPI1_BASE (0x40013000)
#define SPI1 ((struct SPI*)SPI1_BASE)

// SPI::CR1 (control register 1)
#define SPI_CR1_BIDIMODE (0b1 << 15)
#define SPI_CR1_BIDIOE (0b1 << 14)
#define SPI_CR1_SPE (0b1 << 6)
#define SPI_CR1_BR (0b111 << 3)
#define SPI_CR1_BR_DIV_2 (0b000 << 3)
#define SPI_CR1_BR_DIV_4 (0b001 << 3)
#define SPI_CR1_BR_DIV_8 (0b010 << 3)
#define SPI_CR1_BR_DIV_256 (0b111 << 3)
#define SPI_CR1_MSTR (0b1 << 2)

// SPI::CR2 (control register 2)
#define SPI_CR2_TXEIE (0b1 << 7)
#define SPI_CR2_TXDMAEN (0b1 << 1)
#define SPI_CR2_RXDMAEN (0b1 << 0)


// ====================================== //
//  GPIOx (General Purpose Input/Output)  //
// ====================================== //
struct GPIO_type {
    __reg CRL;    // 0x00
    __reg CRH;    // 0x04
    __reg IDR;    // 0x08
    __reg ODR;    // 0x0c
    __reg BSRR;   // 0x10
};

#define GPIOA_BASE (0x40010800)
#define GPIOC_BASE (0x40011000)
#define GPIOA ((struct GPIO_type*)GPIOA_BASE)
#define GPIOC ((struct GPIO_type*)GPIOC_BASE)

// GPIO::CRL (control register low)
#define GPIO_CRL_CNF(PORT) (0b11 << 2 << (4 * PORT))
#define GPIO_CRL_CNF_OUTPUT_PUSH_PULL(PORT) (0b00 << 2 << (4 * PORT))
#define GPIO_CRL_CNF_ALT_PUSH_PULL(PORT) (0b10 << 2 << (4 * PORT))

#define GPIO_CRL_MODE(PORT) (0b11 << (4 * PORT))
#define GPIO_CRL_MODE_2MHz(PORT) (0b10 << (4 * PORT))
#define GPIO_CRL_MODE_50MHz(PORT) (0b11 << (4 * PORT))

#define GPIO_CRL(PORT) (0b1111 << (4 * PORT))

// GPIO::CRH (control register high)
#define GPIO_CRH_CNF(PORT) (0b11 << 2 << (4 * (PORT - 8)))
#define GPIO_CRH_CNF_OUTPUT_PUSH_PULL(PORT) (0b00 << 2 << (4 * (PORT - 8)))
#define GPIO_CRH_CNF_ALT_PUSH_PULL(PORT) (0b10 << 2 << (4 * (PORT - 8)))

#define GPIO_CRH_MODE(PORT) (0b11 << (4 * (PORT - 8)))
#define GPIO_CRH_MODE_2MHz(PORT) (0b10 << (4 * (PORT - 8)))
#define GPIO_CRH_MODE_50MHz(PORT) (0b11 << (4 * (PORT - 8)))

#define GPIO_CRH(PORT) (0b1111 << (4 * (PORT - 8)))

// GPIO::BSRR (bit set/reset register)
#define GPIO_BSRR_SET(PIN) (0b1 << PIN)
#define GPIO_BSRR_RST(PIN) (0b1 << (16 + PIN))



// =============== //
//  TIMx (Timers)  //
// =============== //
struct TIM_type {
    __reg CR1;      // 0x00
    __reg CR2;      // 0x04
    __reg SMCR;     // 0x08
    __reg DIER;     // 0x0c
    __reg SR;       // 0x10
    __reg EGR;      // 0x14
    __reg CCMR1;    // 0x18
    __reg CCMR2;    // 0x1c
    __reg CCER;     // 0x20
    __reg CNT;      // 0x24
    __reg PSC;      // 0x28
    __reg ARR;      // 0x2c
    __reg RCR;      // 0x30
    __reg CCR1;     // 0x34
    __reg CCR2;     // 0x38
    __reg CCR3;     // 0x3c
    __reg CCR4;     // 0x40
    __reg BDTR;     // 0x44
    __reg DCR;      // 0x48
    __reg DMAR;     // 0x4c
};

#define TIM1_BASE (0x40012C00)
#define TIM2_BASE (0x40000000)
#define TIM1 ((struct TIM_type*)TIM1_BASE)
#define TIM2 ((struct TIM_type*)TIM2_BASE)

// TIM::CR1 (control register 1)
#define TIM_CR1_CEN (0b1 << 0)

// TIM::SMCR (slave mode control register)
#define TIM_SMCR_MSM (0b1 << 7)

// TIM::DIER (DMA/interrupt enable register)
#define TIM_DIER_CC3IE (0b1 << 3)
#define TIM_DIER_CC2IE (0b1 << 2)
#define TIM_DIER_CC1IE (0b1 << 1)
#define TIM_DIER_UIE (0b1 << 0)

// TIM::SR (status register)
#define TIM_SR_CC2IF (0b1 << 2)
#define TIM_SR_CC1IF (0b1 << 1)
#define TIM_SR_UIF (0b1 << 0)

// TIM::CCER (capture/compare 1 enable register)
#define TIM_CCER_CC1P (0b1 << 1)
#define TIM_CCER_CC1E (0b1 << 0)

// TIM::CCMR1 (capture/compare mode register)
#define TIM_CCMR1_OCM1 (0b111 << 4)
#define TIM_CCMR1_OCM1_PWM1 (0b110 << 4)
#define TIM_CCMR1_OCM1_PWM2 (0b111 << 4)

// TIM::BDTR (break and deadtime register)
#define TIM_BDTR_MOE (0b1 << 15)

#endif // !STM32_H
