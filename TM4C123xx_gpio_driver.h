/*
		
		Date   - 12/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver Header
		
*/

#ifndef INC_TM4C123XX_GPIO_DRIVER_H_
#define INC_TM4C123XX_GPIO_DRIVER_H_

#include <TM4C123GH6PM.h>
#include <stdint.h>

/********************************** START: Processor Specific Details (Common for all Cortex M4 Processor) ***********************************/
 /*
 *	ARM Cortex Mx Processor NVIC ISERx register Addresses (Interrupt Set Enable Register)
 */

#define NVIC_ISER0 					      ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1 					      ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2 					      ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3 					      ((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER4 					      ((volatile uint32_t *)0xE000E110)


/*
 *	ARM Cortex Mx Processor NVIC ICERx register Addresses (Interrupt Clear Enable Register)
 */

#define NVIC_ICER0 					      ((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1 					      ((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2 					      ((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3 					      ((volatile uint32_t *)0xE000E18C)
#define NVIC_ICER4 					      ((volatile uint32_t *)0xE000E190)

/*
 *  ARM Cortex Mx Processor NVIC IPRx register Addresses (Interrupt Priority Register)
 */

#define NVIC_IPR_BASE_ADDRESS		  ((volatile uint32_t *)0xE000E400)

/*
 *  ARM Cortex Mx Processor NVIC IPRx register bits implemented
 */

#define NUM_PR_BITS_IMPLEMENTED		(__NVIC_PRIO_BITS)      //__NVIC_PRIO_BITS is defined as 3

/******************************************************************* END **********************************************************************/
/* ================================================================================ */
/* ================              GPIO PORT CONFIG                  ================ */
/* ================================================================================ */

typedef struct
{
  uint8_t PortBusType;                                  //@GPIO_Bus_Type (APB or AHB)
  uint8_t PortSleepEn;                                  //Enable or disable sleep mode for the gpio port
  uint8_t PortDeepSleepEn;                              //Enable or disable deep sleep mode for the gpio port
  
}GPIO_Port_Config_t;

/* ================================================================================ */
/* ================              GPIO PIN CONFIG                   ================ */
/* ================================================================================ */

typedef struct
{
  uint8_t PinNumber;                                    //@GPIO_Pin_Number
  uint8_t PinMode;                                      //@GPIO_Mode (GPIO Direction -> Input or output)
  uint8_t PinDigitalEn;                                 //Enable or disable the GPIODEN register
  uint8_t PinAnalogEn;                                  //Enable or disable analog func of pin
  struct PinAltFucn                                     //Configure for Alternate Functionality 
    {
      uint8_t Enable;
      uint8_t PMCx;                                     //@GPIOAltFuncModes
      uint8_t ADCTriggerEn; 
      uint8_t DMATriggerEn;
      
    }PinAltFuncType; 
  uint8_t PinDriveSel;                                  //Pin drive current selection @GPIO_Drive_Sel  
  struct PinOutput                                      //Pin output type configuration
    {
      uint8_t PullUpEn;
      uint8_t PullDownEn;
      uint8_t OpenDrainEn;
      uint8_t SlewRateEn;                               //Enable slew rate only if pin drive is 8mA
    }PinOutputType;
  struct PinIntr
    {
      uint8_t InterruptEn;                              // Sets bits in the GPIOIM register to unmask interrupts 
      uint8_t IntrSenseType;                            // Edges or levels
      uint8_t IntrBothEdgesEn;                          // Only if the intr type is to sense edges(if set the GPIOIEV reg is ignored)
      uint8_t IntrEventType;                            // Rising or high level, falling or low level
      
    }PinIntrControl;
    
}GPIO_Pin_Config_t;

/* ================================================================================ */
/* ================                GPIO HANDLE                     ================ */
/* ================================================================================ */

typedef struct
{

  GPIOA_Type* pGPIO_Base_Address;
  GPIO_Pin_Config_t PinConfig;
  GPIO_Port_Config_t PortConfig;
  
}GPIO_Handle_t;


/************************************************ GPIO Macros **************************************************************/

/*
*   @GPIO_Bus_Type Macros
*/

#define APB       0
#define AHB       1

/*
*   GPIO AHB Port Number code
*/

#define GPIO_PORT_TO_CODE(x)             ( (x == GPIOA ? 0:\
                                            x == GPIOB ? 1:\
                                            x == GPIOC ? 2:\
                                            x == GPIOD ? 3:\
                                            x == GPIOE ? 4:\
                                            x == GPIOF ? 5:0) )

/*
*   GPIO condition check for both APB and AHB
*/
 
#define GPIOA_APB_AHB(x)                 ((x == GPIOA) || (x == GPIOA_AHB))
#define GPIOB_APB_AHB(x)                 ((x == GPIOB) || (x == GPIOB_AHB))
#define GPIOC_APB_AHB(x)                 ((x == GPIOC) || (x == GPIOC_AHB))
#define GPIOD_APB_AHB(x)                 ((x == GPIOD) || (x == GPIOD_AHB))
#define GPIOE_APB_AHB(x)                 ((x == GPIOE) || (x == GPIOE_AHB))
#define GPIOF_APB_AHB(x)                 ((x == GPIOF) || (x == GPIOF_AHB))

/*
*   GPIO Clock Enable Macros
*/

#define GPIOA_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 0))
#define GPIOB_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 1))
#define GPIOC_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 2))
#define GPIOD_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 3))
#define GPIOE_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 4))
#define GPIOF_CLOCK_EN                    (SYSCTL -> RCGC2 |= (1 << 5))

/*
*   GPIO Sleep Clock Enable Macros
*/

#define GPIOA_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 0))
#define GPIOB_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 1))
#define GPIOC_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 2))
#define GPIOD_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 3))
#define GPIOE_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 4))
#define GPIOF_SLEEP_CLOCK_EN              (SYSCTL -> SCGC2 |= (1 << 5))

/*
*   GPIO Deep Sleep Clock Enable Macros
*/

#define GPIOA_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 0))
#define GPIOB_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 1))
#define GPIOC_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 2))
#define GPIOD_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 3))
#define GPIOE_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 4))
#define GPIOF_DEEP_SLEEP_CLOCK_EN         (SYSCTL -> DCGC2 |= (1 << 5))


/*
*   GPIO Clock Disable Macros
*/

#define GPIOA_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 0))
#define GPIOB_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 1))
#define GPIOC_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 2))
#define GPIOD_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 3))
#define GPIOE_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 4))
#define GPIOF_CLOCK_DI                    (SYSCTL -> RCGC2 &= (uint32_t)~(1 << 5))

/*
*   GPIO Sleep Clock Disable Macros
*/

#define GPIOA_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 0))
#define GPIOB_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 1))
#define GPIOC_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 2))
#define GPIOD_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 3))
#define GPIOE_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 4))
#define GPIOF_SLEEP_CLOCK_DI               (SYSCTL -> SCGC2 &= (uint32_t)~(1 << 5))

/*
*   GPIO Deep Sleep Clock Disable Macros
*/

#define GPIOA_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 0))
#define GPIOB_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 1))
#define GPIOC_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 2))
#define GPIOD_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 3))
#define GPIOE_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 4))
#define GPIOF_DEEP_SLEEP_CLOCK_DI          (SYSCTL -> DCGC2 &= (uint32_t)~(1 << 5))

/*
*   GPIO Register reset macros
*/

#define GPIOA_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 0));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 0));}while(0)
#define GPIOB_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 1));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 1));}while(0)
#define GPIOC_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 2));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 2));}while(0)
#define GPIOD_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 3));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 3));}while(0)
#define GPIOE_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 4));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 4));}while(0)
#define GPIOF_REG_RESET         do{(SYSCTL -> SRCR2 |= (1 << 5));(SYSCTL -> SRCR2 &= (uint32_t)~(1 << 5));}while(0)

/*
*   @GPIO_Pin_Number Macros
*/

#define PIN_NUMBER_0              0
#define PIN_NUMBER_1              1
#define PIN_NUMBER_2              2
#define PIN_NUMBER_3              3
#define PIN_NUMBER_4              4
#define PIN_NUMBER_5              5
#define PIN_NUMBER_6              6
#define PIN_NUMBER_7              7

/*
*   @GPIO_Mode Macros
*/

#define GPIO_MODE_INPUT           0
#define GPIO_MODE_OUTPUT          1

/*
 * 	@GPIOAltFuncMode
 * 	Alternate Function Mode macros
 */

#define GPIO_ALT_FUNC_0			    	0         // Use if the pin is to be configured as ADC or DMA trigger
#define GPIO_ALT_FUNC_1			    	1
#define GPIO_ALT_FUNC_2			    	2
#define GPIO_ALT_FUNC_3		    		3
#define GPIO_ALT_FUNC_4		    		4
#define GPIO_ALT_FUNC_5		    		5
#define GPIO_ALT_FUNC_6		    		6
#define GPIO_ALT_FUNC_7		    		7
#define GPIO_ALT_FUNC_8		    		8
#define GPIO_ALT_FUNC_9			     	9
#define GPIO_ALT_FUNC_14		  		14
#define GPIO_ALT_FUNC_15			  	15

/*  
*   @GPIO_Drive_Sel
*/

#define GPIO_DRV_2                0
#define GPIO_DRV_4                1
#define GPIO_DRV_8                2

/*
*   GPIO Interrupt sense type
*/

#define GPIO_INTR_EDGE            0
#define GPIO_INTR_LEVEL           1

/*
*   GPIO Interrupt Event type
*/

#define GPIO_INTR_RISING_HIGH     1
#define GPIO_INTR_FALLING_LOW     0

/*
*   IRQ Intr Priority macros
*/

#define IRQ_PRIORITY_0			    	0
#define IRQ_PRIORITY_1				    1
#define IRQ_PRIORITY_2				    2
#define IRQ_PRIORITY_3				    3
#define IRQ_PRIORITY_4				    4
#define IRQ_PRIORITY_5				    5
#define IRQ_PRIORITY_6				    6
#define IRQ_PRIORITY_7				    7

/*
*   GPIO unlock macro
*/

#define GPIO_UNLOCK(x)            do{(x -> LOCK = 0x4C4F434B);(x -> CR = 0x000000FF);}while(0)       // Pass base address as argument

/*
*   GPIO AHB Bus Enable -> SYSCTL -> GPIOHBCTL
*/

#define GPIO_HBCTL_EN(x)          (SYSCTL -> GPIOHBCTL |= (1 << x))

/*
*   Some Generic Macros
*/

#define ENABLE        1  
#define DISABLE       0
#define SET           ENABLE
#define RESET         DISABLE


/********************************************* GPIO Driver APIs Decleartion *************************************************************/

/*
 *  GPIO Peripheral clock control
 */

void GPIO_PeriClockControl(GPIO_Handle_t *pGPIOHandle, uint8_t En_or_Di);

/*
 *  GPIO Init and DeInit control
 */
 
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInint(GPIOA_Type *pGPIO_Base_Address);

/*
 *  GPIO Read and Write APIs
 */

uint8_t GPIO_ReadFromInputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPort(GPIOA_Type *pGPIO_Base_Address);

void GPIO_WriteToOutputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIOA_Type *pGPIO_Base_Address, uint8_t value);

void GPIO_ToggleOutputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber);

/*
 *  GPIO Interrupt Handling APIs
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber);

/*
 *  GPIO SSI Init
 */

void GPIO_SSI_Init(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber, uint8_t PMC_Encoding, uint8_t PullUp_En);

#endif
