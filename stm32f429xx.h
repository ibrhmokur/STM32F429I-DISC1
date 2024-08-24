/*
 * stm32f429xx.h
 *
 *  Created on: Aug 11, 2024
 *      Author: OKUR
 */


#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include<stdint.h>

#define __vo              volatile
#define __IO              __vo

#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET





/*
 * Base addresses of Flash and SRAM memories
 */

#define	FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U //112KB --> (112*1024): 114688bytes in decimal =  1C000 in hex
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U //ROM_BASEADDR: System Memory
#define SRAM				SRAM1_BASEADDR


/*
 * Base addresses of AHBx and AHPx Bus Peripheral
 */

#define	PERIPH_BASE			0x40000000U
#define	APB1PERIPH_BASE		PERIPH_BASE
#define	APB2PERIPH_BASE		0x40010000U
#define	AHB1PERIPH_BASE		0x40020000U
#define	AHB2PERIPH_BASE		0x50000000U


/*
 * Base addresses of Peripherals which are hanging on AHB1 bus
 */

 #define	GPIOA_BASEADDR		0x40020000U
 #define	GPIOB_BASEADDR		0x40020400U
 #define	GPIOC_BASEADDR		0x40020800U
 #define	GPIOD_BASEADDR		0x40020C00U
 #define	GPIOE_BASEADDR		0x40021000U
 #define	GPIOF_BASEADDR		0x40021400U
 #define	GPIOG_BASEADDR		0x40021800U
 #define	GPIOH_BASEADDR		0x40021C00U
 #define	GPIOI_BASEADDR		0x40022000U
 #define	GPIOJ_BASEADDR		0x40022400U
 #define	GPIOK_BASEADDR		0x40022800U
 #define	CRC_BASEADDR		0x40023000U
 #define	RCC_BASEADDR		0x40023800U
 #define	FLASH_INTERFACE_REGISTER_BASEADDR		0x40023C00U
 #define	BKPSRAM_BASEADDR		0x40024000U
 #define	DMA1_BASEADDR			0x40026000U
 #define	DMA2_BASEADDR			0x40026400U
 #define	ETHERNET_MAC_BASEADDR	0x40028000U
 #define	DMA2D_BASEADDR			0x4002B000U
 #define	USB_OTH_HS_BASEADDR		0x40040000U


/*
 * Base addresses of Peripherals which are hanging on AHB2 bus
 */

 #define	RNG_BASEADDR		0x50060800U
 #define	HASH_BASEADDR		0x50060400U
 #define	CRYP_BASEADDR		0x50060000U
 #define	DCMI_BASEADDR		0x50050000U
 #define	USB_OTH_FS_BASEADDR	0x50000000U


/*
 * Base addresses of Peripherals which are hanging on AHB3 bus
 */

 #define	FMC_BASEADDR		0xA0000000U	//FMC control register


/*
 * Base addresses of Peripherals which are hanging on APB1 bus
 */

 #define	TIM2_BASEADDR		0x40000000U
 #define	TIM3_BASEADDR		0x40000400U
 #define	TIM4_BASEADDR		0x40000800U
 #define	TIM5_BASEADDR		0x40000C00U
 #define	TIM6_BASEADDR		0x40001000U
 #define	TIM7_BASEADDR		0x40001400U
 #define	TIM12_BASEADDR		0x40001800U
 #define	TIM13_BASEADDR		0x40001C00U
 #define	TIM14_BASEADDR		0x40002000U
 #define	RTC_BKP_REGISTERS_BASEADDR		0x40002800U
 #define	WWDG_BASEADDR		0x40002C00U
 #define	IWDG_BASEADDR		0x40003000U
 #define	I2S2ext_BASEADDR	0x40003400U
 #define	SPI2_I2S2_BASEADDR	0x40003800U
 #define	SPI3_I2S3_BASEADDR	0x40003C00U
 #define	I2S3ext_BASEADDR	0x40004000U
 #define	USART2_BASEADDR		0x40004400U
 #define	USART3_BASEADDR		0x40004800U
 #define	UART4_BASEADDR		0x40004C00U
 #define	UART5_BASEADDR		0x40005000U
 #define	I2C1_BASEADDR		0x40005400U
 #define	I2C2_BASEADDR		0x40005800U
 #define	I2C3_BASEADDR		0x40005C00U
 #define	CAN1_BASEADDR		0x40006400U
 #define	CAN2_BASEADDR		0x40006800U
 #define	PWR_BASEADDR		0x40007000U
 #define	DAC_BASEADDR		0x40007400U
 #define	UART7_BASEADDR		0x40007800U
 #define	UART8_BASEADDR		0x40007C00U


/*
 * Base addresses of Peripherals which are hanging on APB2 bus
 */

 #define	TIM1_BASEADDR		0x40010000U
 #define	TIM8_BASEADDR		0x40010400U
 #define	USART1_BASEADDR		0x40011000U
 #define	USART6_BASEADDR		0x40011400U
 #define	ADC1_ADC2_ADC3_BASEADDR	0x40012000U
 #define	SDIO_BASEADDR		0x40012C00U
 #define	SPI1_BASEADDR		0x40013000U
 #define	SPI4_BASEADDR		0x40013400U
 #define	SYSCFG_BASEADDR		0x40013800U
 #define	EXTI_BASEADDR		0x40023C00U
 #define	TIM9_BASEADDR		0x40014000U
 #define	TIM10_BASEADDR		0x40014400U
 #define	TIM11_BASEADDR		0x40014800U
 #define	SPI5_BASEADDR		0x40015000U
 #define	SPI6_BASEADDR		0x40015400U

 #define	SAI1_BASEADDR		0x40015800U
 #define	LCD_TFT_BASEADDR	0x40016800U


/*****************************Peripheral Registers*******************/
/*
 *RM0090 Reference manual
 *STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439
 */



/*
 * GPIO Registers
 */

 typedef struct{
         __vo uint32_t       MODER;           //GPIO port mode register                     Address offset: 0x00;
         __vo uint32_t       OTYPER;          //GPIO port output type register              Address offset: 0x04;
         __vo uint32_t       OSPEEDR;         //GPIO port output speed register             Address offset: 0x08;
         __vo uint32_t       PUPDR;           //GPIO port pull-up/pull-down register        Address offset: 0x0C;
         __vo uint32_t       IDR;             //GPIO port input data register               Address offset: 0x10;
         __vo uint32_t       ODR;             //GPIO port output data register              Address offset: 0x14;
         __vo uint32_t       BSRR;            //GPIO port bit set/reset register            Address offset: 0x18;
         __vo uint32_t       LCKR;            //GPIO port configuration lock register       Address offset: 0x1C;
         __vo uint32_t       AFR[2];          //GPIO alternate function low register        Address offset: 0x20-0x24;   AFR[0]:AFRL, AFR[1]:AFRH
 }GPIO_RegDef_t;


 /*
 * RCC Registers
 */

 typedef struct{

	 __vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	 __vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	 __vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	 __vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	 __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	 __vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	 __vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	 	  uint32_t RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	 __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	 __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	 	  uint32_t RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	 __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	 __vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	 __vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	 	  uint32_t RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  	 __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  	 __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  	 	  uint32_t RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  	 __vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
     __vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
     __vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
     	  uint32_t RESERVED4;     /*!< Reserved, 0x5C                                                                    */
     __vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
     __vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
     	  uint32_t RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
     __vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
     __vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
     	  uint32_t RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
     __vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
     __vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
     __vo uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
     __vo uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
 }RCC_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

 #define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)	// #define GPIOA	((GPIO_RegDef_t*)0x40020000U)
 #define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
 #define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
 #define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
 #define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
 #define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
 #define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
 #define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
 #define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)

 #define RCC	((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

 #define	GPIOA_PCLK_EN()           (RCC->AHB1ENR |= (1<<0))
 #define	GPIOB_PCLK_EN()           (RCC->AHB1ENR |= (1<<1))
 #define	GPIOC_PCLK_EN()           (RCC->AHB1ENR |= (1<<2))
 #define	GPIOD_PCLK_EN()           (RCC->AHB1ENR |= (1<<3))
 #define	GPIOE_PCLK_EN()           (RCC->AHB1ENR |= (1<<4))
 #define	GPIOF_PCLK_EN()           (RCC->AHB1ENR |= (1<<5))
 #define	GPIOG_PCLK_EN()           (RCC->AHB1ENR |= (1<<6))
 #define	GPIOH_PCLK_EN()           (RCC->AHB1ENR |= (1<<7))
 #define	GPIOI_PCLK_EN()           (RCC->AHB1ENR |= (1<<8))
 #define	GPIOJ_PCLK_EN()           (RCC->AHB1ENR |= (1<<9))
 #define	GPIOK_PCLK_EN()           (RCC->AHB1ENR |= (1<<10))


 /*
 * Clock Disable Macros for GPIOx peripherals
 */

 #define	GPIOA_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<0))
 #define	GPIOB_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<1))
 #define	GPIOC_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<2))
 #define	GPIOD_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<3))
 #define	GPIOE_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<4))
 #define	GPIOF_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<5))
 #define	GPIOG_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<6))
 #define	GPIOH_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<7))
 #define	GPIOI_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<8))
 #define	GPIOJ_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<9))
 #define	GPIOK_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<10))



/*
 * Clock Enable Macros for I2Cx peripherals
 */

 #define	I2C1_PCLK_EN()            (RCC->APB1ENR |= (1<<21))
 #define	I2C2_PCLK_EN()            (RCC->APB1ENR |= (1<<22))
 #define	I2C3_PCLK_EN()            (RCC->APB1ENR |= (1<<23))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

 #define	I2C1_PCLK_DI()            (RCC->APB1ENR &= ~(1<<21))
 #define	I2C2_PCLK_DI()            (RCC->APB1ENR &= ~(1<<22))
 #define	I2C3_PCLK_DI()            (RCC->APB1ENR &= ~(1<<23))



/*
 * Clock Enable Macros for SPIx peripherals
 */

 #define	SPI1_PCLK_EN()            (RCC->APB2ENR |= (1<<12))
 #define	SPI2_PCLK_EN()            (RCC->APB1ENR |= (1<<14))
 #define	SPI3_PCLK_EN()            (RCC->APB1ENR |= (1<<15))
 #define	SPI4_PCLK_EN()            (RCC->APB2ENR |= (1<<13))
 #define	SPI5_PCLK_EN()            (RCC->APB2ENR |= (1<<20))
 #define	SPI6_PCLK_EN()            (RCC->APB2ENR |= (1<<21))

/*
 * Clock Disable Macros for SPIx peripherals
 */

 #define	SPI1_PCLK_DI()            (RCC->APB2ENR &= ~(1<<12))
 #define	SPI2_PCLK_DI()            (RCC->APB1ENR &= ~(1<<14))
 #define	SPI3_PCLK_DI()            (RCC->APB1ENR &= ~(1<<15))
 #define	SPI4_PCLK_DI()            (RCC->APB2ENR &= ~(1<<13))
 #define	SPI5_PCLK_DI()            (RCC->APB2ENR &= ~(1<<20))
 #define	SPI6_PCLK_DI()            (RCC->APB2ENR &= ~(1<<21))

/*
 * Clock Enable Macros for TIMx peripherals
 */

 #define	TIM1_PCLK_EN()            (RCC->APB2ENR |= (1<<0))
 #define	TIM2_PCLK_EN()            (RCC->APB1ENR |= (1<<0))
 #define	TIM3_PCLK_EN()            (RCC->APB1ENR |= (1<<1))
 #define	TIM4_PCLK_EN()            (RCC->APB1ENR |= (1<<2))
 #define	TIM5_PCLK_EN()            (RCC->APB1ENR |= (1<<3))
 #define	TIM6_PCLK_EN()            (RCC->APB1ENR |= (1<<4))
 #define	TIM7_PCLK_EN()            (RCC->APB1ENR |= (1<<5))
 #define	TIM8_PCLK_EN()            (RCC->APB2ENR |= (1<<1))
 #define	TIM9_PCLK_EN()            (RCC->APB2ENR |= (1<<16))
 #define	TIM10_PCLK_EN()           (RCC->APB2ENR |= (1<<17))
 #define	TIM11_PCLK_EN()           (RCC->APB2ENR |= (1<<18))
 #define	TIM12_PCLK_EN()           (RCC->APB1ENR |= (1<<6))
 #define	TIM13_PCLK_EN()           (RCC->APB1ENR |= (1<<7))
 #define	TIM14_PCLK_EN()           (RCC->APB1ENR |= (1<<8))

/*
 * Clock Disable Macros for TIMx peripherals
 */

 #define	TIM1_PCLK_DI()            (RCC->APB2ENR &= ~(1<<0))
 #define	TIM2_PCLK_DI()            (RCC->APB1ENR &= ~(1<<0))
 #define	TIM3_PCLK_DI()            (RCC->APB1ENR &= ~(1<<1))
 #define	TIM4_PCLK_DI()            (RCC->APB1ENR &= ~(1<<2))
 #define	TIM5_PCLK_DI()            (RCC->APB1ENR &= ~(1<<3))
 #define	TIM6_PCLK_DI()            (RCC->APB1ENR &= ~(1<<4))
 #define	TIM7_PCLK_DI()            (RCC->APB1ENR &= ~(1<<5))
 #define	TIM8_PCLK_DI()            (RCC->APB2ENR &= ~(1<<1))
 #define	TIM9_PCLK_DI()            (RCC->APB2ENR &= ~(1<<16))
 #define	TIM10_PCLK_DI()           (RCC->APB2ENR &= ~(1<<17))
 #define	TIM11_PCLK_DI()           (RCC->APB2ENR &= ~(1<<18))
 #define	TIM12_PCLK_DI()           (RCC->APB1ENR &= ~(1<<6))
 #define	TIM13_PCLK_DI()           (RCC->APB1ENR &= ~(1<<7))
 #define	TIM14_PCLK_DI()           (RCC->APB1ENR &= ~(1<<8))


 /*
 * Clock Enable Macros for USARTx peripherals
 */

 #define	USART1_PCLK_EN()            (RCC->APB2ENR |= (1<<4))
 #define	USART2_PCLK_EN()            (RCC->APB1ENR |= (1<<17))
 #define	USART3_PCLK_EN()            (RCC->APB1ENR |= (1<<18))
 #define	UART4_PCLK_EN()             (RCC->APB1ENR |= (1<<19))
 #define	UART5_PCLK_EN()             (RCC->APB1ENR |= (1<<20))
 #define	USART6_PCLK_EN()            (RCC->APB2ENR |= (1<<5))


 /*
 * Clock Disable Macros for USARTx peripherals
 */

 #define	USART1_PCLK_DI()            (RCC->APB2ENR &= ~(1<<4))
 #define	USART2_PCLK_DI()            (RCC->APB1ENR &= ~(1<<17))
 #define	USART3_PCLK_DI()            (RCC->APB1ENR &= ~(1<<18))
 #define	UART4_PCLK_DI()             (RCC->APB1ENR &= ~(1<<19))
 #define	UART5_PCLK_DI()             (RCC->APB1ENR &= ~(1<<20))
 #define	USART6_PCLK_DI()            (RCC->APB2ENR &= ~(1<<5))


/*
 * Clock Enable Macros for SYSCFG peripherals
 */

 #define	SYSCFG_PCLK_EN()            (RCC->APB2ENR |= (1<<12))
 #define	SPI2_PCLK_EN()              (RCC->APB1ENR |= (1<<14))
 #define	SPI3_PCLK_EN()              (RCC->APB1ENR |= (1<<15))


/*
 * Clock Disable Macros for SYSCFG peripherals
 */

 #define	SYSCFG_PCLK_DI()            (RCC->APB2ENR &= ~(1<<12))
 #define	SPI2_PCLK_DI()              (RCC->APB1ENR &= ~(1<<14))
 #define	SPI3_PCLK_DI()              (RCC->APB1ENR &= ~(1<<15))



 /*
  * Macros to reset GPIOx peripherals
  */

#define	GPIOA_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define	GPIOB_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define	GPIOC_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define	GPIOD_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define	GPIOE_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define	GPIOF_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define	GPIOG_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define	GPIOH_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while(0)
#define	GPIOI_REG_RESET()         do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));} while(0)


#endif /* INC_STM32F429XX_H_ */
