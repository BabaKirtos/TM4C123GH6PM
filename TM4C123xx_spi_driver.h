/*
		
		Date   - 20/03/2021
		Author - Abhishek Roy Chowdhury
		File   - TM4C123GH6PM GPIO Driver Header
		
*/


#ifndef INC_TM4C123XX_SPI_DRIVER_H_
#define INC_TM4C123XX_SPI_DRIVER_H_

#include<TM4C123GH6PM.h>

/* ================================================================================ */
/* ================              SSI CONFIG STRUCT                 ================ */
/* ================================================================================ */

typedef struct
{
	uint8_t SSI_DeviceMode;							// Use macros defined at @SSI_DeviceMode
	uint8_t SSI_BusConfig;							// Use macros defined at @SSI_BusConfig
	uint8_t SSI_SclkSpeed;							// Use macros defined at @SSI_SclkSpeed
	uint8_t SSI_DFF;								// Use macros defined at @SSI_DFF
	uint8_t SSI_CPOL;								// Use macros defined at @SSI_CPOL
	uint8_t SSI_CPHA;								// Use macros defined at @SSI_CPHA
	uint8_t SSI_SSM;								// Use macros defined at @SSI_SSM

}SSI_Config_t;

/* ================================================================================ */
/* ================              SSI HANDLE STRUCT                 ================ */
/* ================================================================================ */

typedef struct
{
    SSI0_Type *pSSIx;				//SSI0 reg def from TM4C123GH6PM.h file
	SSI_Config_t SSIConfig;			//SSI Config structure
	uint8_t PAD;
  
}SSI_Handle_t;

/* ================================================================================ */
/* ================        SSI CLOCK AND RESET MACROS              ================ */
/* ================================================================================ */

/*
 *  SSI Clock Enable Macros
 */

#define SSI0_CLOCK_ENABLE                   (SYSCTL -> RCGC1 |= (1 << 4))
#define SSI1_CLOCK_ENABLE                   (SYSCTL -> RCGC1 |= (1 << 5))
#define SSI2_CLOCK_ENABLE                   (SYSCTL -> RCGCSSI |= (1 << 2))
#define SSI3_CLOCK_ENABLE                   (SYSCTL -> RCGCSSI |= (1 << 3))


/*
*   SSI Sleep Clock Enable Macros
*/

#define SSI0_SLEEP_CLOCK_EN                 (SYSCTL -> SCGC1 |= (1 << 4))
#define SSI1_SLEEP_CLOCK_EN                 (SYSCTL -> SCGC1 |= (1 << 5))
#define SSI2_SLEEP_CLOCK_EN                 (SYSCTL -> SCGCSSI |= (1 << 2))
#define SSI3_SLEEP_CLOCK_EN                 (SYSCTL -> SCGCSSI |= (1 << 3))


/*
*   SSI Deep Sleep Clock Enable Macros
*/

#define SSI0_DEEP_SLEEP_CLOCK_EN            (SYSCTL -> DCGC1 |= (1 << 4))
#define SSI1_DEEP_SLEEP_CLOCK_EN            (SYSCTL -> DCGC1 |= (1 << 5))
#define SSI2_DEEP_SLEEP_CLOCK_EN            (SYSCTL -> DCGCSSI |= (1 << 2))
#define SSI3_DEEP_SLEEP_CLOCK_EN            (SYSCTL -> DCGCSSI |= (1 << 3))


/*
 *  SSI Clock Disable Macros
 */

#define SSI0_CLOCK_DISABLE                  (SYSCTL -> RCGC1 &= (uint32_t)~(1 << 4))
#define SSI1_CLOCK_DISABLE                  (SYSCTL -> RCGC1 &= (uint32_t)~(1 << 5))
#define SSI2_CLOCK_DISABLE                  (SYSCTL -> RCGCSSI &= (uint32_t)~(1 << 2))
#define SSI3_CLOCK_DISABLE                  (SYSCTL -> RCGCSSI &= (uint32_t)~(1 << 3))

/*
*   SSI Sleep Clock Disable Macros
*/

#define SSI0_SLEEP_CLOCK_DI                 (SYSCTL -> SCGC1 &= (uint32_t)~(1 << 4))
#define SSI1_SLEEP_CLOCK_DI                 (SYSCTL -> SCGC1 &= (uint32_t)~(1 << 5))
#define SSI2_SLEEP_CLOCK_DI                 (SYSCTL -> SCGCSSI &= (uint32_t)~(1 << 2))
#define SSI3_SLEEP_CLOCK_DI                 (SYSCTL -> SCGCSSI &= (uint32_t)~(1 << 3))
  
/*
*   SSI Deep Sleep Clock Disable Macros
*/

#define SSI0_DEEP_SLEEP_CLOCK_DI            (SYSCTL -> DCGC1 &= (uint32_t)~(1 << 4))
#define SSI1_DEEP_SLEEP_CLOCK_DI            (SYSCTL -> DCGC1 &= (uint32_t)~(1 << 5))
#define SSI2_DEEP_SLEEP_CLOCK_DI            (SYSCTL -> DCGCSSI &= (uint32_t)~(1 << 2))
#define SSI3_DEEP_SLEEP_CLOCK_DI            (SYSCTL -> DCGCSSI &= (uint32_t)~(1 << 3))


/*
*   SSI Register reset macros
*/

#define SSI0_REG_RESET         do{(SYSCTL -> SRCR1 |= (1 << 4));(SYSCTL -> SRCR1 &= (uint32_t)~(1 << 4));}while(0)
#define SSI1_REG_RESET         do{(SYSCTL -> SRCR1 |= (1 << 5));(SYSCTL -> SRCR1 &= (uint32_t)~(1 << 5));}while(0)
#define SSI2_REG_RESET         do{(SYSCTL -> SRSSI |= (1 << 2));(SYSCTL -> SRSSI &= (uint32_t)~(1 << 2));}while(0)
#define SSI3_REG_RESET         do{(SYSCTL -> SRSSI |= (1 << 3));(SYSCTL -> SRSSI &= (uint32_t)~(1 << 3));}while(0)


/* ================================================================================ */
/* ================              	SSI APIs                       ================ */
/* ================================================================================ */

/*
 *  SSI Peripheral clock control
 */

void SSI_PeriClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di);
void SSI_PeriSleepClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di);
void SSI_PeriDeepSleepClockControl(SSI0_Type *pSSIx, uint8_t En_or_Di);

/*
 *  SSI Init and De-Init
 */

void SSI_Init(SSI_Handle_t *pSSIHandle);
void SSI_DeInit(SSI0_Type *pSSIx);



#endif
