#ifndef SET_ALL_PRIO
#define SET_ALL_PRIO
 
#include "mbed.h"
 
/*
NOTE
----
SysTick_IRQn is vector -1.
The vector mentioned in the comment after SysTick_IRQn is vector 0 
*/
 
enum FIRST_LAST_IRQ {
// ************************** Freescale **************************
// --------- TARGET_K20XX
#ifdef TARGET_K20D50M
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = SWI_IRQn,
#elif defined TARGET_TEENSY3_1
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = SWI_IRQn,
 
// --------- TARGET_K22F
#elif defined TARGET_K22F
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = Reserved101_IRQn,
 
// --------- TARGET_KLXX
#elif defined TARGET_KL05Z
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = PORTB_IRQn,
#elif defined TARGET_KL25Z
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = PORTD_IRQn,
#elif defined TARGET_KL43Z    
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = PORTCD_IRQn,
#elif defined TARGET_KL46Z    
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = PORTC_PORTD_IRQn,
 
// --------- TARGET_MCU_K64F
#elif defined TARGET_MCU_K64F    
    first_IRQ_number = SysTick_IRQn, // DMA0_IRQn
    last_IRQ_number  = ENET_Error_IRQn,
 
// **************************   Nordic  **************************
// --------- TARGET_MCU_NRF51822
#elif defined TARGET_MCU_NRF51822    
    first_IRQ_number = SysTick_IRQn, // POWER_CLOCK_IRQn
    last_IRQ_number  = SWI5_IRQn,
 
// **************************    NXP    **************************
#elif defined TARGET_LPC11U6X
    first_IRQ_number = SysTick_IRQn, // PIN_INT0_IRQn
    last_IRQ_number  = USBWAKEUP_IRQn,
#elif defined TARGET_LPC11UXX
    first_IRQ_number = SysTick_IRQn, // FLEX_INT0_IRQn
    last_IRQ_number  = Reserved6_IRQn,
#elif defined TARGET_LPC11XX_11CXX
    first_IRQ_number = SysTick_IRQn, // WAKEUP0_IRQn
    last_IRQ_number  = EINT0_IRQn,
#elif defined TARGET_LPC13XX
    first_IRQ_number = SysTick_IRQn, // PIN_INT0_IRQn
    last_IRQ_number  = Reserved5_IRQn,
#elif defined TARGET_LPC15XX
    first_IRQ_number = SysTick_IRQn, // WDT_IRQn
    last_IRQ_number  = RTC_WAKE_IRQn,
#elif defined TARGET_LPC176X
    first_IRQ_number = SysTick_IRQn, // WDT_IRQn
    last_IRQ_number  = CANActivity_IRQn,
#elif defined TARGET_LPC23XX
    first_IRQ_number = TIMER0_IRQn,
    last_IRQ_number  = I2S_IRQn,
#elif defined TARGET_LPC408X
    first_IRQ_number = SysTick_IRQn, // WDT_IRQn
    last_IRQ_number  = CMP1_IRQn,
#elif defined TARGET_LPC43XX
    first_IRQ_number = SysTick_IRQn, // DAC_IRQn
    last_IRQ_number  = QEI_IRQn,
#elif defined TARGET_LPC81X
    first_IRQ_number = SysTick_IRQn, // SPI0_IRQn
    last_IRQ_number  = PININT7_IRQn,
#elif defined TARGET_LPC82X
    first_IRQ_number = SysTick_IRQn, // SPI0_IRQn
    last_IRQ_number  = PININT7_IRQn,
 
// **************************  Renesas  **************************
// --------- TARGET_RZ_A1H
#elif defined TARGET_RZ_A1H
    first_IRQ_number = SGI0_IRQn,
    last_IRQ_number  = TINT170_IRQn,
 
// **************************    STM    **************************
// --------- TARGET_DISCO_F100RB
#elif defined TARGET_DISCO_F100RB
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = DMA1_Channel7_IRQn,
    #ifdef STM32F10X_LD
        last_IRQ_number  = USBWakeUp_IRQn,
    #endif /* STM32F10X_LD */  
    #ifdef STM32F10X_LD_VL
        last_IRQ_number  = TIM7_IRQn,
    #endif /* STM32F10X_LD_VL */
    #ifdef STM32F10X_MD
        last_IRQ_number  = USBWakeUp_IRQn,
    #endif /* STM32F10X_MD */  
    #ifdef STM32F10X_MD_VL
        last_IRQ_number  = TIM7_IRQn,
    #endif /* STM32F10X_MD_VL */
    #ifdef STM32F10X_HD
        last_IRQ_number  = DMA2_Channel4_5_IRQn,
    #endif /* STM32F10X_HD */  
    #ifdef STM32F10X_HD_VL
        last_IRQ_number  = DMA2_Channel5_IRQn,
    #endif /* STM32F10X_HD_VL */
    #ifdef STM32F10X_XL
        last_IRQ_number  = DMA2_Channel4_5_IRQn,
    #endif /* STM32F10X_XL */  
    #ifdef STM32F10X_CL
        last_IRQ_number  = OTG_FS_IRQn,
    #endif /* STM32F10X_CL */
 
// --------- TARGET_NUCLEO_F103RB
#elif defined TARGET_NUCLEO_F103RB
    first_IRQ_number =  SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = DMA1_Channel7_IRQn,
    #ifdef STM32F10X_LD
        last_IRQ_number  = USBWakeUp_IRQn,
    #endif /* STM32F10X_LD */  
    #ifdef STM32F10X_LD_VL
        last_IRQ_number  = TIM7_IRQn,
    #endif /* STM32F10X_LD_VL */
    #ifdef STM32F10X_MD
        last_IRQ_number  = USBWakeUp_IRQn,
    #endif /* STM32F10X_MD */  
    #ifdef STM32F10X_MD_VL
        last_IRQ_number  = TIM7_IRQn,
    #endif /* STM32F10X_MD_VL */
    #ifdef STM32F10X_HD
        last_IRQ_number  = DMA2_Channel4_5_IRQn,
    #endif /* STM32F10X_HD */  
    #ifdef STM32F10X_HD_VL
        last_IRQ_number  = DMA2_Channel5_IRQn,
    #endif /* STM32F10X_HD_VL */
    #ifdef STM32F10X_XL
        last_IRQ_number  = DMA2_Channel4_5_IRQn,
    #endif /* STM32F10X_XL */  
    #ifdef STM32F10X_CL
        last_IRQ_number  = OTG_FS_IRQn,
    #endif /* STM32F10X_CL */
 
// --------- TARGET_STM32F0
#elif defined TARGET_DISCO_F051R8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = CEC_CAN_IRQn,
#elif defined TARGET_NUCLEO_F030R8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = USART2_IRQn,
#elif defined TARGET_NUCLEO_F070RB
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = USB_IRQn,
#elif defined TARGET_NUCLEO_F072RB
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = USB_IRQn,
#elif defined TARGET_NUCLEO_F091RC
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = CEC_CAN_IRQn,
 
// --------- TARGET_STM32F3
#elif defined TARGET_DISCO_F303VC
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
#elif defined TARGET_DISCO_F334C8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
#elif defined TARGET_NUCLEO_F302R8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
#elif defined TARGET_NUCLEO_F303RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI4_IRQn,
#elif defined TARGET_NUCLEO_F334R8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
 
// --------- TARGET_STM32F3XX
#elif defined TARGET_STM32F3XX
    first_IRQ_number = WWDG_IRQn,
    last_IRQ_number  = FPU_IRQn,
    #ifdef STM32F303xC 
        first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
        last_IRQ_number  = FPU_IRQn,
    #endif /* STM32F303xC */
    #ifdef STM32F334x8 
        first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
        last_IRQ_number  = FPU_IRQn,
    #endif /* STM32F334x8 */
    #ifdef STM32F302x8 
        first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
        last_IRQ_number  = FPU_IRQn,
    #endif /* STM32F302x8 */
 
// --------- TARGET_STM32F4
#elif defined TARGET_DISCO_F401VC
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI4_IRQn,
#elif defined TARGET_DISCO_F429ZI
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = DMA2D_IRQn,
#elif defined TARGET_MTS_DRAGONFLY_F411RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI5_IRQn,
#elif defined TARGET_MTS_MDOT_F405RG
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
#elif defined TARGET_MTS_MDOT_F411RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI5_IRQn,
#elif defined TARGET_NUCLEO_F401RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI4_IRQn,
#elif defined TARGET_NUCLEO_F411RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = SPI5_IRQn,
#elif defined TARGET_STM32F407VG
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = FPU_IRQn,
 
// --------- TARGET_STM32F4XX
#elif defined TARGET_STM32F4XX
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = HASH_RNG_IRQn,
    #ifdef STM32F40XX
        last_IRQ_number  = FPU_IRQn,
    #endif /* STM32F40XX */
    #ifdef STM32F427X 
        last_IRQ_number  = SPI6_IRQn,
    #endif /* STM32F427X */
 
// --------- TARGET_STM32L0
#elif defined TARGET_DISCO_L053C8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = USB_IRQn,
#elif defined TARGET_NUCLEO_L053R8
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = USB_IRQn,
 
// --------- TARGET_STM32L1
#elif defined TARGET_NUCLEO_L152RE
    first_IRQ_number = SysTick_IRQn, // WWDG_IRQn
    last_IRQ_number  = COMP_ACQ_IRQn,
#endif
};
 
void NVIC_set_all_irq_priorities(int priority);
 
#endif // SET_ALL_PRIO