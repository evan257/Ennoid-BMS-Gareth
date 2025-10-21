#include <stdint.h>

// base address and size of SRAM program location
#define SRAM_BASE       0x20000000U
#define SRAM_SIZE       40*1024

// base address and size of SRAM program location
#define SRAM_BASE       0x20000000U
#define SRAM_SIZE       40*1024
#define SRAM_END        (SRAM_BASE + SRAM_SIZE)

void main();
void __libc_init_array(void);

void Reset_Handler(void);
// put in prototypes
// void Reset_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void NMI_Handler                    (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SVCall_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDG_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_TS_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_CAN_TX_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_CAN_RX0_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN_RX1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN_SCE_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void USBWakeUP_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void FMC_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC4_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_1_2_3_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_4_5_6_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP7_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USBWakeUp_RMP_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_BRK_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_UP_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_TRG_COM_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_CC_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4                           (void) __attribute__ ((weak, alias("Default_Handler")));

// this is visible to the linker script
uint32_t * vectors[]
__attribute__ ((section(".isr_vector")))= {
    (uint32_t *) SRAM_END,                       //  0x00
    (uint32_t *) Reset_Handler,                  //  0x04
    (uint32_t *) NMI_Handler,                    //  0x08
    (uint32_t *) HardFault_Handler,              //  0x0C
    (uint32_t *) MemManage_Handler,              //  0x10
    (uint32_t *) BusFault_Handler,               //  0x14
    (uint32_t *) UsageFault_Handler,             //  0x18
    0,                                           //  0x1C
    0,                                           //  0x20
    0,                                           //  0x24
    0,                                           //  0x28
    (uint32_t *) SVCall_Handler,                 //  0x2C
    (uint32_t *) DebugMon_Handler,               //  0x30
    0,                                           //  0x34
    (uint32_t *) PendSV_Handler,                 //  0x38
    (uint32_t *) SysTick_Handler,                //  0x3C
    (uint32_t *) WWDG_IRQHandler,                //  0x40
    (uint32_t *) PVD_IRQHandler,                 //  0x44
    (uint32_t *) TAMP_STAMP_IRQHandler,          //  0x48
    (uint32_t *) RTC_WKUP_IRQHandler,            //  0x4C
    (uint32_t *) FLASH_IRQHandler,               //  0x50
    (uint32_t *) RCC_IRQHandler,                 //  0x54
    (uint32_t *) EXTI0_IRQHandler,               //  0x58
    (uint32_t *) EXTI1_IRQHandler,               //  0x5C
    (uint32_t *) EXTI2_TS_IRQHandler,            //  0x60
    (uint32_t *) EXTI3_IRQHandler,               //  0x64
    (uint32_t *) EXTI4_IRQHandler,               //  0x68
    (uint32_t *) DMA1_Channel1_IRQHandler,       //  0x6C
    (uint32_t *) DMA1_Channel2_IRQHandler,       //  0x70
    (uint32_t *) DMA1_Channel3_IRQHandler,       //  0x74
    (uint32_t *) DMA1_Channel4_IRQHandler,       //  0x78
    (uint32_t *) DMA1_Channel5_IRQHandler,       //  0x7C
    (uint32_t *) DMA1_Channel6_IRQHandler,       //  0x80
    (uint32_t *) DMA1_Channel7_IRQHandler,       //  0x84
    (uint32_t *) ADC1_2_IRQHandler,              //  0x88
    (uint32_t *) USB_HP_CAN_TX_IRQHandler,       //  0x8C
    (uint32_t *) USB_LP_CAN_RX0_IRQHandler,      //  0x90
    (uint32_t *) CAN_RX1_IRQHandler,             //  0x94
    (uint32_t *) CAN_SCE_IRQHandler,             //  0x98
    (uint32_t *) EXTI9_5_IRQHandler,             //  0x9C
    (uint32_t *) TIM1_BRK_TIM15_IRQHandler,      //  0xA0
    (uint32_t *) TIM1_UP_TIM16_IRQHandler,       //  0xA4
    (uint32_t *) TIM1_TRG_COM_TIM17_IRQHandler,  //  0xA8
    (uint32_t *) TIM1_CC_IRQHandler,             //  0xAC
    (uint32_t *) TIM2_IRQHandler,                //  0xB0
    (uint32_t *) TIM3_IRQHandler,                //  0xB4
    (uint32_t *) TIM4_IRQHandler,                //  0xB8
    (uint32_t *) I2C1_EV_IRQHandler,             //  0xBC
    (uint32_t *) I2C1_ER_IRQHandler,             //  0xC0
    (uint32_t *) I2C2_EV_IRQHandler,             //  0xC4
    (uint32_t *) I2C2_ER_IRQHandler,             //  0xC8
    (uint32_t *) SPI1_IRQHandler,                //  0xCC
    (uint32_t *) SPI2_IRQHandler,                //  0xD0
    (uint32_t *) USART1_IRQHandler,              //  0xD4
    (uint32_t *) USART2_IRQHandler,              //  0xD8
    (uint32_t *) USART3_IRQHandler,              //  0xDC
    (uint32_t *) EXTI15_10_IRQHandler,           //  0xE0
    (uint32_t *) RTC_Alarm_IRQHandler,           //  0xE4
    (uint32_t *) USBWakeUP_IRQHandler,           //  0xE8
    (uint32_t *) TIM8_BRK_IRQHandler,            //  0xEC
    (uint32_t *) TIM8_UP_IRQHandler,             //  0xF0
    (uint32_t *) TIM8_TRG_COM_IRQHandler,        //  0xF4
    (uint32_t *) TIM8_CC_IRQHandler,             //  0xF8
    (uint32_t *) ADC3_IRQHandler,                //  0xFC
    0,                                           // 0x100
    0,                                           // 0x104
    0,                                           // 0x108
    (uint32_t *) SPI3_IRQHandler,                // 0x10C
    (uint32_t *) UART4_IRQHandler,               // 0x110
    (uint32_t *) UART5_IRQHandler,               // 0x114
    (uint32_t *) TIM6_DAC_IRQHandler,            // 0x118
    (uint32_t *) TIM7_IRQHandler,                // 0x11C
    (uint32_t *) DMA2_Channel1_IRQHandler,       // 0x120
    (uint32_t *) DMA2_Channel2_IRQHandler,       // 0x124
    (uint32_t *) DMA2_Channel3_IRQHandler,       // 0x128
    (uint32_t *) DMA2_Channel4_IRQHandler,       // 0x12C
    (uint32_t *) DMA2_Channel5_IRQHandler,       // 0x130
    (uint32_t *) ADC4_IRQHandler,                // 0x134
    0,                                           // 0x138
    0,                                           // 0x13C
    (uint32_t *) COMP_1_2_3_IRQHandler,          // 0x140
    (uint32_t *) COMP_4_5_6_IRQHandler,          // 0x144
    (uint32_t *) COMP7_IRQHandler,               // 0x148
    0,                                           // 0x14C
    0,                                           // 0x150
    0,                                           // 0x154
    0,                                           // 0x158
    0,                                           // 0x15C
    0,                                           // 0x160
    0,                                           // 0x164
    (uint32_t *) USB_HP_IRQHandler,              // 0x168
    (uint32_t *) USB_LP_IRQHandler,              // 0x16C
    (uint32_t *) USBWakeUp_RMP_IRQHandler,       // 0x170
    0,                                           // 0x174
    0,                                           // 0x178
    0,                                           // 0x17C
    0,                                           // 0x180
    (uint32_t *) FPU_IRQHandler,                 // 0x184
};

extern uint32_t _etext;
extern uint32_t _ebss;
extern uint32_t _sbss;
extern uint32_t _edata;
extern uint32_t _sdata;
extern uint32_t _sidata;


void Default_Handler()
{
    while(1){}
}

void Reset_Handler(){
    // copy initialized data (.data) from flash to sram
    uint32_t size = (uint32_t) &_edata - (uint32_t) &_sdata;
    uint8_t *pDst = (uint8_t *) &_sdata;  // sram
    uint8_t *pSrc = (uint8_t *) &_sidata; // flash
    for (uint32_t i=0; i<size; i++)
    {
        *pDst++ = *pSrc++;
    }

    // zero out uninitialized data (.bss) in sram
    size = (uint32_t) &_ebss - (uint32_t) &_sbss;
    pDst = (uint8_t *) &_sbss;
    for (uint32_t i=0; i<size; i++)
    {
        *pDst++ = 0;
    }

    // initialize system
    SystemInit();
    // initialize standard c library
    __libc_init_array();
    // run main program
    main();
}