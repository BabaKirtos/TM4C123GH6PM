/*
		
		Date   - 12/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver API
		
*/

#include <TM4C123GH6PM.h>
#include <stdint.h>
#include <string.h>
#include "TM4C123xx_gpio_driver.h"

/*
 *  Peripheral Clock control
 */

/******************************************************************************************************
 * @fn			-	GPIO_PeriClockControl
 *
 * @brief		-	This function enables or disables clock for the given GPIO port
 *
 * @param[in]	-	GPIO base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/
void GPIO_PeriClockControl(GPIO_Handle_t *pGPIOHandle, uint8_t En_or_Di)
{
    if(En_or_Di)
    {
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_CLOCK_EN;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_CLOCK_EN;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_CLOCK_EN;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_CLOCK_EN;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_CLOCK_EN;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_CLOCK_EN;      
    }
    else
    {
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_CLOCK_DI;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_CLOCK_DI;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_CLOCK_DI;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_CLOCK_DI;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_CLOCK_DI;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_CLOCK_DI;
    }
    
    if(pGPIOHandle->PortConfig.PortSleepEn)
    {
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_SLEEP_CLOCK_EN;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_SLEEP_CLOCK_EN;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_SLEEP_CLOCK_EN;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_SLEEP_CLOCK_EN;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_SLEEP_CLOCK_EN;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_SLEEP_CLOCK_EN; 
    }
    else if(pGPIOHandle->PortConfig.PortSleepEn == DISABLE)
    {
      // Sleep Clock Diable code here
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_SLEEP_CLOCK_DI;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_SLEEP_CLOCK_DI;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_SLEEP_CLOCK_DI;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_SLEEP_CLOCK_DI;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_SLEEP_CLOCK_DI;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_SLEEP_CLOCK_DI;
    }
    
    if(pGPIOHandle->PortConfig.PortDeepSleepEn)
    {
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_DEEP_SLEEP_CLOCK_EN;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_DEEP_SLEEP_CLOCK_EN;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_DEEP_SLEEP_CLOCK_EN;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_DEEP_SLEEP_CLOCK_EN;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_DEEP_SLEEP_CLOCK_EN;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_DEEP_SLEEP_CLOCK_EN; 
    }
    else if(pGPIOHandle->PortConfig.PortDeepSleepEn == DISABLE)
    {
      // Deep Sleep Clock disable code here
      if(GPIOA_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOA_DEEP_SLEEP_CLOCK_DI;
      else if(GPIOB_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOB_DEEP_SLEEP_CLOCK_DI;
      else if(GPIOC_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOC_DEEP_SLEEP_CLOCK_DI;
      else if(GPIOD_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOD_DEEP_SLEEP_CLOCK_DI;
      else if(GPIOE_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOE_DEEP_SLEEP_CLOCK_DI;
      else if(GPIOF_APB_AHB(pGPIOHandle->pGPIO_Base_Address))
        GPIOF_DEEP_SLEEP_CLOCK_DI;
    }
}  

/*
 *  Init and De-Init control
 */

/******************************************************************************************************
 * @fn			  -	GPIO_Init
 *
 * @brief		  -	This function initializes the GPIO
 *
 * @param[in]	-	GPIO handle pointer
 *
 * @return		-	Null
 *
 * @note	  	-
 *
 ********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    //1. Configure the GPIOHCTL if Bus Typ is AHB
    if(pGPIOHandle->PortConfig.PortBusType == AHB)
    {
      uint8_t PortCode = 0;
      PortCode = GPIO_PORT_TO_CODE(pGPIOHandle->pGPIO_Base_Address);
      GPIO_HBCTL_EN(PortCode);
    }
    
    //2. Enable Peripheral Clock  
    GPIO_PeriClockControl(pGPIOHandle, ENABLE);
    
    //UNLOCK PROTOCOL IMPLEMENTED BELOW!!! PLEASE BEWARE OF CHANGING PROTECTED GPIO BITS(refer datasheet)
    GPIO_UNLOCK(pGPIOHandle->pGPIO_Base_Address);
    
    //3. Set Direction for the GPIO using pin mode
    if(pGPIOHandle->PinConfig.PinMode)
      pGPIOHandle->pGPIO_Base_Address->DIR |= (1 << pGPIOHandle->PinConfig.PinNumber);
    else if(!(pGPIOHandle->PinConfig.PinMode))
      pGPIOHandle->pGPIO_Base_Address->DIR &= (uint8_t)~(1 << pGPIOHandle->PinConfig.PinNumber);
    
    //4. Configure Digital or Analog Enable
    if(pGPIOHandle->PinConfig.PinDigitalEn)
      pGPIOHandle->pGPIO_Base_Address->DEN |= (1 << pGPIOHandle->PinConfig.PinNumber);
    else if(pGPIOHandle->PinConfig.PinAnalogEn)
      // Only valid for pins with analog circuitry 
      pGPIOHandle->pGPIO_Base_Address->AMSEL |= (1 << pGPIOHandle->PinConfig.PinNumber);    
    
    //5. Configure Alternate Functionality 
    if(pGPIOHandle->PinConfig.PinAltFuncType.Enable)
    {
      // Enable bit in AFSEL if AltFuncEn = 1
      pGPIOHandle->pGPIO_Base_Address->AFSEL |= (1 << pGPIOHandle->PinConfig.PinNumber);
      
      // Configure the GPIOPCTL register accordingly
      if(pGPIOHandle->PinConfig.PinAltFuncType.PMCx > 0)
        pGPIOHandle->pGPIO_Base_Address->PCTL |= (uint32_t)(pGPIOHandle->PinConfig.PinAltFuncType.PMCx << (4 * pGPIOHandle->PinConfig.PinNumber));
      
      // Configure the pin as ADC or DMA trigger
      else if(pGPIOHandle->PinConfig.PinAltFuncType.ADCTriggerEn)
        pGPIOHandle->pGPIO_Base_Address->ADCCTL |= (1 << pGPIOHandle->PinConfig.PinNumber);
      else if(pGPIOHandle->PinConfig.PinAltFuncType.DMATriggerEn)
        pGPIOHandle->pGPIO_Base_Address->DMACTL |= (1 << pGPIOHandle->PinConfig.PinNumber);
      
    }
    else if(!(pGPIOHandle->PinConfig.PinAltFuncType.Enable))
      pGPIOHandle->pGPIO_Base_Address->AFSEL &= (uint8_t)~(1 << pGPIOHandle->PinConfig.PinNumber);
    
    //6. Configure pin current drive strength (NOTE: for slew rate drive strength needs to be 8mA, also by default drive strength is 2mA)
    if(pGPIOHandle->PinConfig.PinDriveSel == GPIO_DRV_2)
      pGPIOHandle->pGPIO_Base_Address->DR2R |= (1 << pGPIOHandle->PinConfig.PinNumber);    
    else if(pGPIOHandle->PinConfig.PinDriveSel == GPIO_DRV_4)
      pGPIOHandle->pGPIO_Base_Address->DR4R |= (1 << pGPIOHandle->PinConfig.PinNumber);
    else if(pGPIOHandle->PinConfig.PinDriveSel == GPIO_DRV_8)
      pGPIOHandle->pGPIO_Base_Address->DR8R |= (1 << pGPIOHandle->PinConfig.PinNumber);
    
    //7. Configure GPIO output type
    if(pGPIOHandle->PinConfig.PinOutputType.OpenDrainEn)
    {
      // Enable bit in ODR register and also set the pin as digital enable
      //(NOTE: Open drain is used when GPIO is configured as output, if the pin is input then OD wont have any effect)
      pGPIOHandle->pGPIO_Base_Address->DEN |= (1 << pGPIOHandle->PinConfig.PinNumber);
      pGPIOHandle->pGPIO_Base_Address->ODR |= (1 << pGPIOHandle->PinConfig.PinNumber);
    }
    else if(pGPIOHandle->PinConfig.PinOutputType.PullUpEn)
      // Enable Pull up bit in PUR
      pGPIOHandle->pGPIO_Base_Address->PUR |= (1 << pGPIOHandle->PinConfig.PinNumber);
    else if(pGPIOHandle->PinConfig.PinOutputType.PullDownEn)
      //Enable Pull down bit in PDR
      pGPIOHandle->pGPIO_Base_Address->PDR |= (1 << pGPIOHandle->PinConfig.PinNumber);
    else if(pGPIOHandle->PinConfig.PinOutputType.SlewRateEn)
      // Enable Slew rate in SLR
      pGPIOHandle->pGPIO_Base_Address->SLR |= (1 << pGPIOHandle->PinConfig.PinNumber);
    
    //8. Interrupt Configuration 
    if(pGPIOHandle->PinConfig.PinIntrControl.InterruptEn)
    {
      //Configure the pin as input mode and digital enable
      pGPIOHandle->pGPIO_Base_Address->DIR &= (uint8_t)~(1 << pGPIOHandle->PinConfig.PinNumber);
      pGPIOHandle->pGPIO_Base_Address->DEN |= (1 << pGPIOHandle->PinConfig.PinNumber);  
      
      //Enable the interrupt bit in GPIOIM reg
      pGPIOHandle->pGPIO_Base_Address->IM |= (1 << pGPIOHandle->PinConfig.PinNumber);
      
      //Configure the type of sense(edge or level)
      if(pGPIOHandle->PinConfig.PinIntrControl.IntrSenseType == GPIO_INTR_LEVEL)
      {
        pGPIOHandle->pGPIO_Base_Address->IS |= (1 << pGPIOHandle->PinConfig.PinNumber);
      }
      else if(pGPIOHandle->PinConfig.PinIntrControl.IntrSenseType == GPIO_INTR_EDGE)
      {
        pGPIOHandle->pGPIO_Base_Address->IS &= (uint8_t) ~(1 << pGPIOHandle->PinConfig.PinNumber);
      }
      
      //Configure the GPIOIBE to detect both edges(NOTE - If set the GPIOIEV reg is ignored)
      if(pGPIOHandle->PinConfig.PinIntrControl.IntrBothEdgesEn)
      {
        pGPIOHandle->pGPIO_Base_Address->IBE |= (1 << pGPIOHandle->PinConfig.PinNumber);
      }
      else if(!(pGPIOHandle->PinConfig.PinIntrControl.IntrBothEdgesEn))
      {
        if(pGPIOHandle->PinConfig.PinIntrControl.IntrEventType == GPIO_INTR_RISING_HIGH)
          pGPIOHandle->pGPIO_Base_Address->IEV |= (1 << pGPIOHandle->PinConfig.PinNumber);
        else if(pGPIOHandle->PinConfig.PinIntrControl.IntrEventType == GPIO_INTR_FALLING_LOW)
          pGPIOHandle->pGPIO_Base_Address->IEV &= (uint8_t) ~(1 << pGPIOHandle->PinConfig.PinNumber);
      }      
    }    
}

/******************************************************************************************************
 * @fn			  -	GPIO_DeInit
 *
 * @brief		  -	This function De-initializes the GPIO
 *
 * @param[in]	-	GPIO base address
 *
 * @return		-	Null
 *
 * @note	  	-
 *
 ********************************************************************************************************/
void GPIO_DeInint(GPIOA_Type *pGPIO_Base_Address)
{
  if(GPIOA_APB_AHB(pGPIO_Base_Address))
    GPIOA_REG_RESET;
  else if(GPIOB_APB_AHB(pGPIO_Base_Address))
    GPIOB_REG_RESET;
  else if(GPIOC_APB_AHB(pGPIO_Base_Address))
    GPIOC_REG_RESET;
  else if(GPIOD_APB_AHB(pGPIO_Base_Address))
    GPIOD_REG_RESET;
  else if(GPIOE_APB_AHB(pGPIO_Base_Address))
    GPIOE_REG_RESET;
  else if(GPIOF_APB_AHB(pGPIO_Base_Address))
    GPIOF_REG_RESET;  
}

/*
 *  Data Read and Write
 */

/******************************************************************************************************
 * @fn		  	-	GPIO_ReadFromInputPin
 *
 * @brief		  -	This function enables user to read from input pin of a gpio port
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Pin Number of that GPIO
 *
 * @return		-	Return the read value from the GPIO pin (0 or 1)
 *
 * @note		  -
 *
 ********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIO_Base_Address->DATA >> PinNumber & 0x00000001); // Right shifting the IDR to the bit 0 and masking other bits.
	return value;
}
/*********************************************************************************************************
* @fn			     -	GPIO_ReadFromInputPort
 *
 * @brief		   -	This function enables user to read from input of a gpio port
 *
 * @param[in]  -	Pointer to GPIOx
 *
 * @return		 -	Return the read value from the GPIO port (0 or 1 of 16 pins)
 *
 * @note	   	-
 *
 ********************************************************************************************************/
uint8_t GPIO_ReadFromInputPort(GPIOA_Type *pGPIO_Base_Address)
{
	uint8_t value;
	value = (uint8_t)(pGPIO_Base_Address->DATA);
	return value;
}
/******************************************************************************************************
 * @fn			-	GPIO_WriteToOutputPin
 *
 * @brief		-	This function writes the given value to the GPIOx pin
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Pin Number
 *
 * @param[in]	-	Value to be written (0 o 1)
 *
 * @return		-	NULL
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_WriteToOutputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber, uint8_t value)
{
	if(value == SET)
		pGPIO_Base_Address->DATA |= (1 << PinNumber);
	else
		pGPIO_Base_Address->DATA &= ~(1 << PinNumber);
}
/*********************************************************************************************************
  * @fn			-	GPIO_WriteToOutputPort
 *
 * @brief		-	This function writes the given value to the GPIOx port
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Value to be written (0 o 1)
 *
 * @return		-	NULL
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIOA_Type *pGPIO_Base_Address, uint8_t value)
{
	pGPIO_Base_Address->DATA = value;
}
/*********************************************************************************************************
 * @fn		  	-	GPIO_ToggleOutputPin
 *
 * @brief		  -	This function toggles the output pin of GPIOx
 *
 * @param[in]	-	Pointer to the GPIO
 *
 * @param[in]	-	Pin Number
 *
 * @return		-	Null
 *
 * @note		  -
 *
 ********************************************************************************************************/
void GPIO_ToggleOutputPin(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber)
{
	pGPIO_Base_Address->DATA ^= (1 << PinNumber);
}


/*
 *  IRQ Configuration and ISR handling
 */

/******************************************************************************************************
 * @fn			  -	GPIO_IRQInterruptConfig
 *
 * @brief		  -	This function is used to configure the IRQ (Interrupt Request)
 *
 * @param[in]	-	IRQNumber (defined at @IRQNumber macro)
 *
 * @param[in]	-	Enable or Disable
 *
 * @return		-	Null
 *
 * @note		  -
 *
 ********************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
    else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//Program ISER2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
    else if(IRQNumber >= 128 && IRQNumber < 139)
		{
			//Program ISER2 register
			*NVIC_ISER4 |= (1 << (IRQNumber % 128));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
    else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 96));
		}
    else if(IRQNumber >= 128 && IRQNumber <139)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 128));
		}
	}
}
/***********************************************************************************************************
 * @fn		  	-	GPIO_IRQPriorityConfig
 *
 * @brief		  -	This function is used to configure the IRQ Priority (Interrupt Request)
 *
 * @param[in]	-	IRQPriority (defined at  macro)
 *
 * @param[in]	-	IRQNumber (defined at @IRQNumber macro)
 *
 * @return		-	Null
 *
 * @note		  -
 *
 ***********************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out IPR register to touch
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (NUM_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + iprx) |= (IRQPriority << (shift_amount));
}

/*************************************************************************************************************
 * @fn			  -	GPIO_IRQHandling
 *
 * @brief		  -	Stores the address of ISR handler in the Vector Address corresponding to the IRQ Number
 *
 * @param[in]	-	Base address and Pin Number
 *
 * @return		-	Null
 *
 * @note		  -
 *
 **************************************************************************************************************/
void GPIO_IRQHandling(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber)
{
	if((pGPIO_Base_Address->MIS) & (1 << PinNumber))
  {
    pGPIO_Base_Address->ICR |= (1 << PinNumber);    
  }
}

/*
 *    GPIO SPI configuration
 */

/*********************************************************************************************************
 * @fn		  	-	GPIO_SSI_Init
 *
 * @brief		  -	This function configures a given GPIO port and pin for SSI functionality  
 *
 * @param[in]	-	Pointer to the GPIO
 *
 * @param[in]	-	Pin Number
 *
 * @param[in]	-	PMCx encoding from @GPIOAltFuncMode
 *
 * @return		-	Null
 *
 * @note		  -
 *
 ********************************************************************************************************/
void GPIO_SSI_Init(GPIOA_Type *pGPIO_Base_Address, uint8_t PinNumber, uint8_t PMCx_Encoding, uint8_t PullUp_En)
{
	GPIO_Handle_t SSI_Pin_Config;
  
	memset(&SSI_Pin_Config,0,sizeof(SSI_Pin_Config));

	SSI_Pin_Config.pGPIO_Base_Address = pGPIO_Base_Address;
	SSI_Pin_Config.PortConfig.PortBusType = APB;
	SSI_Pin_Config.PinConfig.PinNumber = PinNumber;
	SSI_Pin_Config.PinConfig.PinDigitalEn = ENABLE;
	if(PullUp_En)
	SSI_Pin_Config.PinConfig.PinOutputType.PullUpEn = ENABLE;
	SSI_Pin_Config.PinConfig.PinAltFuncType.Enable = ENABLE;
	SSI_Pin_Config.PinConfig.PinAltFuncType.PMCx = PMCx_Encoding;

	GPIO_Init(&SSI_Pin_Config);
  
}

