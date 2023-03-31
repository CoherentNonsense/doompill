#include "vector_table.h"

typedef void (*InterruptVectorFn)(void);

__attribute__ ((section(".vector_table")))
const volatile InterruptVectorFn vector_table[] = {
    (void (*)(void))((u32)&_estack),    // 0x000 sp
    reset_handler,                      // 0x004 Reset
    0,                                  // 0x008 Non maskable interrupt
    0,                                  // 0x00C HardFault
    0,                                  // 0x010 Memory Management
    0,                                  // 0x014 BusFault
    0,                                  // 0x018 UsageFault
    0,                                  // 0x01C Reserved
    0,                                  // 0x020 Reserved
    0,                                  // 0x024 Reserved
    0,                                  // 0x028 Reserved
    0,                                  // 0x02C System service call
    0,                                  // 0x030 Debug Monitor
    0,                                  // 0x034 Reserved
    0,                                  // 0x038 PendSV                         
    0,                                  // 0x03C System tick timer              
    0,                                  // 0x040 Window watchdog                
    0,                                  // 0x044 PVD through EXTI Line detection
    0,                                  // 0x048 Tamper                         
    0,                                  // 0x04C RTC global                     
    0,                                  // 0x050 FLASH global                   
    0,                                  // 0x054 RCC global                     
    0,                                  // 0x058 EXTI Line0                     
    0,                                  // 0x05C EXTI Line1                     
    0,                                  // 0x060 EXTI Line2                     
    0,                                  // 0x064 EXTI Line3                     
    0,                                  // 0x068 EXTI Line4                     
    0,                                  // 0x06C DMA1_Ch1                       
    0,                                  // 0x070 DMA1_Ch2                       
    dma1_channel3_handler,              // 0x074 DMA1_Ch3
    0,                                  // 0x078 DMA1_Ch4                       
    0,                                  // 0x07C DMA1_Ch5                       
    0,                                  // 0x080 DMA1_Ch6                       
    0,                                  // 0x084 DMA1_Ch7                       
    0,                                  // 0x088 ADC1 and ADC2 global           
    0,                                  // 0x08C CAN1_TX                        
    0,                                  // 0x090 CAN1_RX0                       
    0,                                  // 0x094 CAN1_RX1                       
    0,                                  // 0x098 CAN1_SCE 
    0,                                  // 0x09C EXTI Lines 9:5
    0,                                  // 0x0A0 TIM1 Break
    0,                                  // 0x0A4 TIM1 Update 
    0,                                  // 0x0A8 TIM1 Trigger and Communication
    tim1_cc_handler,                    // 0x0AC TIM1 Capture Compare
    tim2_handler,                       // 0x0B0 TIM2                           
    0,                                  // 0x0B4 TIM3                           
    0,                                  // 0x0B8 TIM4                           
    0,                                  // 0x0BC I2C1 event                     
    0,                                  // 0x0C0 I2C1 error                     
    0,                                  // 0x0C4 I2C2 event                     
    0,                                  // 0x0C8 I2C2 error                     
    0,                                  // 0x0CC SPI1                           
    0,                                  // 0x0D0 SPI2                           
    0,                                  // 0x0D4 USART1                         
    0,                                  // 0x0D8 USART2                         
    0,                                  // 0x0DC USART3                         
    0,                                  // 0x0E0 EXTI Lines 15:10               
    0,                                  // 0x0E4 RTC alarm through EXTI line    
    0,                                  // 49  USB OTG FS Wakeup through EXTI
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // -   Reserved                      
    0,                                  // 57  TIM5                          
    0,                                  // 58  SPI3                          
    0,                                  // 59  USART4                        
    0,                                  // 60  USART5                        
    0,                                  // 61  TIM6                          
    0,                                  // 62  TIM7                          
    0,                                  // 63  DMA2_Ch1                      
    0,                                  // 64  DMA2_Ch2                      
    0,                                  // 65  DMA2_Ch3                      
    0,                                  // 66  DMA2_Ch4                      
    0,                                  // 67  DMA2_Ch5                      
    0,                                  // 68  Ethernet                      
    0,                                  // 69  Ethernet wakeup               
    0,                                  // 70  CAN2_TX                       
    0,                                  // 71  CAN2_RX0                      
    0,                                  // 72  CAN2_RX1                      
    0,                                  // 73  CAN2_SCE                      
    0,                                  // 74  USB OTG FS                    
};
