#include <TM4C123GH6PM.h>
#include "TM4C123xx_gpio_driver.h"

void delay(void);

int main(void)
{
  GPIO_PeriClockControl(GPIOF,ENABLE);
  GPIOF -> DIR |= (uint8_t)0x0E;
  GPIOF -> DEN |= (uint8_t)0x0E;
  while(1)
    {
      GPIOF -> DATA |= (1 << 1);
      delay();
      GPIOF -> DATA |= (1 << 2);
      delay();
      GPIOF -> DATA |= (1 << 3);
      delay();
      GPIOF -> DATA &= (uint8_t)~(1 << 1);
      delay();
      GPIOF -> DATA &= (uint8_t)~(1 << 2);
      delay();
      GPIOF -> DATA &= (uint8_t)~(1 << 3);
      
    }  
}

void delay(void)
{
  for(uint32_t i = 0; i < 2500000; i++);  
}
