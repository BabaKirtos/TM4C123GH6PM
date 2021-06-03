/*
		
		Date   - 12/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver API
		
*/

#include <TM4C123GH6PM.h>
#include <stdint.h>
#include "TM4C123xx_gpio_driver.h"
#include "TM4C123xx_spi_driver.h"

/*
 *  Peripheral Clock control
 */

/******************************************************************************************************
 * @fn		  	-	SSI_PeriClockControl
 *
 * @brief		  -	This function enables or disables clock for the given SSI port
 *
 * @param[in]	-	SSI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		  -	none
 *
 ********************************************************************************************************/
 
void SSI_PeriClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di)
{
	if(En_or_Di)
	{
		if(pSSIx == SSI0)
			SSI0_CLOCK_ENABLE;
		else if(pSSIx == SSI1)
			SSI1_CLOCK_ENABLE;
		else if(pSSIx == SSI2)
			SSI2_CLOCK_ENABLE;
		else if(pSSIx == SSI3)
			SSI3_CLOCK_ENABLE;
	}
	else
	{
		if(pSSIx == SSI0)
			SSI0_CLOCK_DISABLE;
		else if(pSSIx == SSI1)
			SSI1_CLOCK_DISABLE;
		else if(pSSIx == SSI2)
			SSI2_CLOCK_DISABLE;
		else if(pSSIx == SSI3)
			SSI3_CLOCK_DISABLE;
	}
} 

/******************************************************************************************************
 * @fn		  	-	SSI_PeriSleepClockControl
 *
 * @brief		  -	This function enables or disables sleep clock for the given SSI port
 *
 * @param[in]	-	SSI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		  -	none
 *
 ********************************************************************************************************/

void SSI_PeriSleepClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di)
{
	if(En_or_Di)
	{
		if(pSSIx == SSI0)
			SSI0_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI1)
			SSI1_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI2)
			SSI2_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI3)
			SSI3_SLEEP_CLOCK_EN;
	}
	else
	{
		if(pSSIx == SSI0)
			SSI0_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI1)
			SSI1_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI2)
			SSI2_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI3)
			SSI3_SLEEP_CLOCK_DI;
	}
}

/******************************************************************************************************
 * @fn		  	-	SSI_PeriDeepSleepClockControl
 *
 * @brief		  -	This function enables or disables sleep clock for the given SSI port
 *
 * @param[in]	-	SSI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		  -	none
 *
 ********************************************************************************************************/

void SSI_PeriDeepSleepClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di)
{
	if(En_or_Di)
	{
		if(pSSIx == SSI0)
			SSI0_DEEP_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI1)
			SSI1_DEEP_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI2)
			SSI2_DEEP_SLEEP_CLOCK_EN;
		else if(pSSIx == SSI3)
			SSI3_DEEP_SLEEP_CLOCK_EN;
	}
	else
	{
		if(pSSIx == SSI0)
			SSI0_DEEP_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI1)
			SSI1_DEEP_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI2)
			SSI2_DEEP_SLEEP_CLOCK_DI;
		else if(pSSIx == SSI3)
			SSI3_DEEP_SLEEP_CLOCK_DI;
	}
}

/*
 *  Init and De-Init control
 */
/******************************************************************************************************
 * @fn			-	SPI_Init
 *
 * @brief		-	This function initializes the SPIx
 *
 * @param[in]	-	SPIx base address
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ******************************************************************************************************/

void SSI_Init(SSI_Handle_t *pSSIHandle)
{
	
}

/******************************************************************************************************
 * @fn			  -	SSI_DeInit
 *
 * @brief		  -	This function resets the SSI module
 *
 * @param[in]	-	SSI Base address
 *
 * @return		-	Null
 *
 * @note	  	-	None
 *
 ********************************************************************************************************/

void SSI_DeInit(SSI0_Type *pSSIx)
{
	if(pSSIx == SSI0)
		SSI0_REG_RESET;
	else if(pSSIx == SSI1)
		SSI1_REG_RESET;
	else if(pSSIx == SSI2)
		SSI2_REG_RESET;
	else if(pSSIx == SSI3)
		SSI3_REG_RESET;
}
