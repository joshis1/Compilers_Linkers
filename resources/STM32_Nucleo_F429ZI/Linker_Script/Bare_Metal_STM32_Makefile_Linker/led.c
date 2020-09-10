#include "led.h"

#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  __IO uint32_t CKGATENR;      /*!< RCC Clocks Gated Enable Register,                            Address offset: 0x90 */ /* Only for STM32F412xG, STM32413_423xx and STM32F446xx devices */
  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */ /* Only for STM32F410xx, STM32F412xG, STM32413_423xx and STM32F446xx devices */

} RCC_TypeDef;

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region                                */
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)
#define GPIOB                 ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)

void led_init()
{


	//DocID028599 - Page 24 - User Manual
	RCC->AHB1ENR |= (0x1 <<1) ;
#if 1

	GPIOB->MODER &= ~(0x3 << 0);  //PB0 -- Green LED
	GPIOB->MODER |= (0x1 << 0);  //01b is output mode

	GPIOB->MODER &= ~(0x3 << 14);  //PB7 -- Blue LED
	GPIOB->MODER |= (0x1 << 14);  //01b is output mode

	GPIOB->MODER &= ~(0x3 << 28);  //PB14 -- Blue LED
    GPIOB->MODER |= (0x1 << 28);  //01b is output mode
#endif
}

void led_on(led_e led_color)
{
#if 1
	switch(led_color)
	{
	case GREEN_LED_PIN:
		GPIOB->ODR |= 0x1 << GREEN_LED_PIN;
		break;
	case  BLUE_LED_PIN:
		GPIOB->ODR |= 0x1 << BLUE_LED_PIN;
		break;

	case RED_LED_PIN:
		GPIOB->ODR |= 0x1 << RED_LED_PIN;
		break;
	}
#endif

}

void led_off(led_e led_color)
{
	#if 1
	switch(led_color)
	{
	case GREEN_LED_PIN:
		GPIOB->ODR &= ~(0x1 << GREEN_LED_PIN);
		break;
	case  BLUE_LED_PIN:
		GPIOB->ODR &= ~(0x1 << BLUE_LED_PIN);
		break;

	case RED_LED_PIN:
		GPIOB->ODR &= ~(0x1 << RED_LED_PIN);
		break;
	}
	#endif

}

void delay(uint32_t delay)
{
	for(volatile int i = 0; i < delay; i++);
}

void RCC_DeInit(void)
{
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON, PLLON, PLLI2S and PLLSAI(STM32F42xxx/43xxx/446xx/469xx/479xx devices) bits */
  RCC->CR &= (uint32_t)0xEAF6FFFF;
  
  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

#if 1
  /* Reset PLLI2SCFGR register */
  /**defined **/
  RCC->PLLI2SCFGR = 0x20003000;
#endif /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F401xx || STM32F411xE || STM32F446xx || STM32F413_423xx || STM32F469_479xx */

#if 1
  /* Reset PLLSAICFGR register, only available for STM32F42xxx/43xxx/446xx/469xx/479xx devices */
  /**defined **/
  RCC->PLLSAICFGR = 0x24003000;
#endif /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F446xx || STM32F469_479xx */
  
  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  /* Disable Timers clock prescalers selection, only available for STM32F42/43xxx and STM32F413_423xx devices */
  RCC->DCKCFGR = 0x00000000;

}