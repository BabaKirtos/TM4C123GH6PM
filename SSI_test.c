/*
		
		Date   - 12/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver Header
		
*/

#include <TM4C123GH6PM.h>
#include <string.h>
#include "TM4C123xx_gpio_driver.h"
#include "TM4C123xx_spi_driver.h"


void GPIO_ALT_SSI_Init(void);

int main(void)
{
	GPIO_ALT_SSI_Init();
	SSI_PeriClockControl(SSI1,ENABLE);
	SSI_PeriSleepClockControl(SSI2,ENABLE);
	SSI_PeriDeepSleepClockControl(SSI3,ENABLE);
	SSI_DeInit(SSI1);
	SSI_DeInit(SSI2);
	SSI_DeInit(SSI3);
	return 0;
}


void GPIO_ALT_SSI_Init(void)
{
	GPIO_SSI_Init(GPIOB,PIN_NUMBER_4,GPIO_ALT_FUNC_2, ENABLE);     //sclk
	GPIO_SSI_Init(GPIOB,PIN_NUMBER_5,GPIO_ALT_FUNC_2, ENABLE);     //nss
	GPIO_SSI_Init(GPIOB,PIN_NUMBER_6,GPIO_ALT_FUNC_2, DISABLE);    //MISO
	GPIO_SSI_Init(GPIOB,PIN_NUMBER_7,GPIO_ALT_FUNC_2, DISABLE);    //MOSI 
  
}
