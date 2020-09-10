#include <stdint.h>

#define SRAM_START  (0x20000000U)

#define SRAM_SIZE (128 *1024) //bytes

#define SRAM_END  (SRAM_START + SRAM_SIZE)

#define STACK_START  (SRAM_END)

extern uint32_t _etext; // from linker file - end of text section 
extern uint32_t _edata; // from linker file - end of data section
extern uint32_t _ebss; // from linker file - end of bss section.
extern uint32_t _sdata; // from linker file - start of data section
extern uint32_t _sbss; //from linker file. - start of bss section.

extern uint32_t _la_data; //from linker file. - start of data section.

void Reset_Handler(void);
void Default_Handler(void);

void __libc_init_array(void);

int main(void);
//alias symbol.

// weak means this can be overridden by the User in their program.
// Check GNU gcc for more details on attribute, alias - common variable attribute.

void NMI_Handler(void)        __attribute__ ((weak, alias ("Default_Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void MemFault_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SVCall_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DebugMonitor_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void Systick_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void Watchdog_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void PVD_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TampStamp_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void RTC_WKUP_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void FLASH_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void RCC_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI0_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI1_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI2_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI3_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI4_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream0_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream1_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream2_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream3_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream4_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream5_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream6_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void ADC_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN1_TX_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void CAN1_RX0_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));        
void CAN1_RX1_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));        
void CAN1_SCE_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));  
void EXTI9_5_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void TIM1_BRK_TIM9_IRQHandler 		(void) __attribute__ ((weak, alias("Default_Handler")));   
void TIM1_UP_TIM10_IRQHandler 		(void) __attribute__ ((weak, alias("Default_Handler")));   
void TIM1_TRG_COM_TIM11_IRQHandler 	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void TIM2_IRQHandler 				(void) __attribute__ ((weak, alias("Default_Handler")));            
void TIM3_IRQHandler 				(void) __attribute__ ((weak, alias("Default_Handler")));            
void TIM4_IRQHandler 				(void) __attribute__ ((weak, alias("Default_Handler")));            
void I2C1_EV_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C1_ER_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C2_EV_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C2_ER_IRQHandler 			(void) __attribute__ ((weak, alias("Default_Handler")));         
void SPI1_IRQHandler  				(void) __attribute__ ((weak, alias("Default_Handler")));           
void SPI2_IRQHandler 				(void) __attribute__ ((weak, alias("Default_Handler")));            
void USART1_IRQHandler  			(void) __attribute__ ((weak, alias("Default_Handler")));         
void USART2_IRQHandler  			(void) __attribute__ ((weak, alias("Default_Handler")));        
void USART3_IRQHandler   			(void) __attribute__ ((weak, alias("Default_Handler")));        
void EXTI15_10_IRQHandler   		(void) __attribute__ ((weak, alias("Default_Handler")));     
void RTC_Alarm_IRQHandler    		(void) __attribute__ ((weak, alias("Default_Handler")));    
void OTG_FS_WKUP_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler"))); 
void TIM8_BRK_TIM12_IRQHandler   	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler    	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler 	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler          	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void FSMC_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void SDIO_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler            	(void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler            	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler          	(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler           	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler     	(void) __attribute__ ((weak, alias("Default_Handler")));
void USART6_IRQHandler           	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler          	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler          	(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler   	(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler    	(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler      	(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler           	(void) __attribute__ ((weak, alias("Default_Handler")));
void DCMI_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void CRYP_IRQHandler             	(void) __attribute__ ((weak, alias("Default_Handler")));
void HASH_RNG_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));       
void UART7_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void UART8_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));       
void SPI4_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI5_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));       
void SPI6_IRQHandler         	(void) __attribute__ ((weak, alias("Default_Handler")));
void SAI_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));     
void SAI2_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));   
void QuadSPI_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));         
void HDMI_CEC_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));   
void SPDIF_Rx_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));   
void FMPI2C1_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));   
void FMPI2C1_error_IRQHandler              	(void) __attribute__ ((weak, alias("Default_Handler")));  

//user defined sections - put and not in the data section.
//GNU section - common variable attribute
//https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Common-Variable-Attributes.html#Common-Variable-Attributes

// .isr_vector - is just a user defined section name.
//arm-none-eabi-objdump -h stm32_startup.o

//Table 62- Reference Manul Stm32F429 
//Vector table is micro-controller specific i.e. MCU.
uint32_t vectors[] __attribute__ ((section (".isr_vector"))) = {
  (uint32_t) STACK_START, // MSP
  (uint32_t) &Reset_Handler,  
  (uint32_t) &NMI_Handler,
  (uint32_t) &HardFault_Handler,
  (uint32_t) &MemFault_Handler,
  (uint32_t) &BusFault_Handler,
  (uint32_t) &UsageFault_Handler,
   0, //reserved
   0,
   0,
   0,
  (uint32_t) &SVCall_Handler,
  (uint32_t) &DebugMonitor_Handler,
   0,
  (uint32_t) &PendSV_Handler,
  (uint32_t) &Systick_Handler,
  (uint32_t) &Watchdog_Handler,
  (uint32_t) &PVD_Handler,
  (uint32_t) &TampStamp_Handler,
  (uint32_t) &RTC_WKUP_Handler,
  (uint32_t) &FLASH_Handler,
  (uint32_t) &RCC_Handler,
  (uint32_t) &EXTI0_Handler,
  (uint32_t) &EXTI1_Handler,
  (uint32_t) &EXTI2_Handler,
  (uint32_t) &EXTI3_Handler,
  (uint32_t) &EXTI4_Handler,
  (uint32_t) &DMA1_Stream0_Handler,
  (uint32_t) &DMA1_Stream1_Handler,
  (uint32_t) &DMA1_Stream2_Handler,
  (uint32_t) &DMA1_Stream3_Handler,
  (uint32_t) &DMA1_Stream4_Handler,
  (uint32_t) &DMA1_Stream5_Handler,
  (uint32_t) &DMA1_Stream6_Handler,
  (uint32_t) &ADC_Handler,
  (uint32_t) &CAN1_TX_IRQHandler,
  (uint32_t) &CAN1_RX0_IRQHandler,
  (uint32_t) &CAN1_RX1_IRQHandler,
  (uint32_t) &CAN1_SCE_IRQHandler,
  (uint32_t) &EXTI9_5_IRQHandler,
  (uint32_t) &TIM1_BRK_TIM9_IRQHandler,
  (uint32_t) &TIM1_UP_TIM10_IRQHandler,
  (uint32_t) &TIM1_TRG_COM_TIM11_IRQHandler,
  (uint32_t) &TIM1_CC_IRQHandler,
  (uint32_t) &TIM2_IRQHandler,
  (uint32_t) &TIM3_IRQHandler,
  (uint32_t) &TIM4_IRQHandler,
  (uint32_t) &I2C1_EV_IRQHandler,
  (uint32_t) &I2C1_ER_IRQHandler,
  (uint32_t) &I2C2_EV_IRQHandler,
  (uint32_t) &I2C2_ER_IRQHandler,
  (uint32_t) &SPI1_IRQHandler,
  (uint32_t) &SPI2_IRQHandler,
  (uint32_t) &USART1_IRQHandler,
  (uint32_t) &USART2_IRQHandler,
  (uint32_t) &USART3_IRQHandler,
  (uint32_t) &EXTI15_10_IRQHandler,
  (uint32_t) &RTC_Alarm_IRQHandler,
  (uint32_t) &OTG_FS_WKUP_IRQHandler,
  (uint32_t) &TIM8_BRK_TIM12_IRQHandler,
  (uint32_t) &TIM8_UP_TIM13_IRQHandler,
  (uint32_t) &TIM8_TRG_COM_TIM14_IRQHandler,
  (uint32_t) &TIM8_CC_IRQHandler,
  (uint32_t) &DMA1_Stream7_IRQHandler,
  (uint32_t) &FSMC_IRQHandler,
  (uint32_t) &SDIO_IRQHandler,
  (uint32_t) &TIM5_IRQHandler,
  (uint32_t) &SPI3_IRQHandler,
  (uint32_t) &UART4_IRQHandler,
  (uint32_t) &UART5_IRQHandler,
  (uint32_t) &TIM6_DAC_IRQHandler,
  (uint32_t) &TIM7_IRQHandler,
  (uint32_t) &DMA2_Stream0_IRQHandler,
  (uint32_t) &DMA2_Stream1_IRQHandler,
  (uint32_t) &DMA2_Stream2_IRQHandler,
  (uint32_t) &DMA2_Stream3_IRQHandler,
  (uint32_t) &DMA2_Stream4_IRQHandler,
   0,
   0,
  (uint32_t) &CAN2_TX_IRQHandler,
  (uint32_t) &CAN2_RX0_IRQHandler,
  (uint32_t) &CAN2_RX1_IRQHandler,
  (uint32_t) &CAN2_SCE_IRQHandler,
  (uint32_t) &OTG_FS_IRQHandler,
  (uint32_t) &DMA2_Stream5_IRQHandler,
  (uint32_t) &DMA2_Stream6_IRQHandler,
  (uint32_t) &DMA2_Stream7_IRQHandler,
  (uint32_t) &USART6_IRQHandler,
  (uint32_t) &I2C3_EV_IRQHandler,
  (uint32_t) &I2C3_ER_IRQHandler,
  (uint32_t) &OTG_HS_EP1_OUT_IRQHandler,
  (uint32_t) &OTG_HS_EP1_IN_IRQHandler,
  (uint32_t) &OTG_HS_WKUP_IRQHandler,
  (uint32_t) &OTG_HS_IRQHandler,
  (uint32_t) &DCMI_IRQHandler,
   0,
   0,
  (uint32_t)&FPU_IRQHandler,
   0,
   0,
  (uint32_t)&SPI4_IRQHandler,
   0,
   0,
  (uint32_t)&SAI_IRQHandler,
   0,
   0,
   0,
  (uint32_t)&SAI2_IRQHandler,
  (uint32_t)&QuadSPI_IRQHandler,
  (uint32_t)&HDMI_CEC_IRQHandler,
  (uint32_t)&SPDIF_Rx_IRQHandler,
  (uint32_t)&FMPI2C1_IRQHandler,
  (uint32_t)&FMPI2C1_error_IRQHandler ,
};


void Reset_Handler(void)
{
	//copy data section to SRAM.
	//& means - the address rather than value
	uint32_t size = (uint32_t)&_edata  -  (uint32_t)&_sdata; //& means the address of _edata.
	uint8_t *pDst = (uint8_t*) &_sdata;  //SRAM
	uint8_t *pSrc = (uint8_t*) & _la_data; //flash--end of text section or begining of data section in Flash.
	for(uint32_t i = 0; i < size; i++)
	{ 
		 *pDst++ = *pSrc++;
	}
	//Init, the .bss section to zero in SRAM
	size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	pDst = (uint8_t*) &_sbss;
	//zero out bss
	for(uint32_t i = 0; i < size; i++)
	{
		*pDst++ = 0;
	}
	
	__libc_init_array(); // initialize C standard library - newlib 
	main();
}

void Default_Handler(void)
{
	while(1);
}