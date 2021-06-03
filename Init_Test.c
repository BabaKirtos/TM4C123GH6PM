/*
		
		Date   - 12/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver Header
		
*/

#include <TM4C123GH6PM.h>
#include <string.h>
#include "TM4C123xx_gpio_driver.h"

void GPIOF_Handler(void);
void delay(void);

int main(void)
{
  GPIO_Handle_t Switch_PF4, LED_PF2;
  
  memset(&Switch_PF4,0,sizeof(Switch_PF4));
  memset(&LED_PF2,0,sizeof(LED_PF2));
  
  // Configure switch as interrupt
  Switch_PF4.pGPIO_Base_Address = GPIOF;
  Switch_PF4.PortConfig.PortBusType = APB;
  Switch_PF4.PinConfig.PinNumber = PIN_NUMBER_4;
  Switch_PF4.PinConfig.PinMode = GPIO_MODE_INPUT;
  Switch_PF4.PinConfig.PinDigitalEn = ENABLE;
  Switch_PF4.PinConfig.PinOutputType.PullUpEn = ENABLE;
  Switch_PF4.PinConfig.PinIntrControl.InterruptEn = ENABLE;
  Switch_PF4.PinConfig.PinIntrControl.IntrSenseType = GPIO_INTR_EDGE;
  Switch_PF4.PinConfig.PinIntrControl.IntrEventType = GPIO_INTR_FALLING_LOW;
  
  // Configure LED as output
  LED_PF2.pGPIO_Base_Address = GPIOF;
  LED_PF2.PortConfig.PortBusType = APB;
  LED_PF2.PinConfig.PinNumber = PIN_NUMBER_2;
  LED_PF2.PinConfig.PinMode = GPIO_MODE_OUTPUT;
  LED_PF2.PinConfig.PinDigitalEn = ENABLE;
  
  // Call GPIO Init Functions
  GPIO_Init(&Switch_PF4);
  GPIO_Init(&LED_PF2);
  
  // Configure Interrupt on porcessor side
  GPIO_IRQInterruptConfig(GPIOF_IRQn,ENABLE);
  GPIO_IRQPriorityConfig(GPIOF_IRQn,IRQ_PRIORITY_0);  
  
  while(1);
}

void GPIOF_Handler(void)
{
  delay();
  GPIO_IRQHandling(GPIOF,PIN_NUMBER_4);
  GPIO_ToggleOutputPin(GPIOF,PIN_NUMBER_2);  
}

void delay(void)
{
  for(uint32_t i = 0; i < 500000; i++);  
}
