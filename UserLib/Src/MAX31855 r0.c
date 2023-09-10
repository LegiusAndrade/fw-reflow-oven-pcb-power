/********************************************************************************
 * @file    		MAX31855.c
 * @author  		Lucas Andrade
 * @brief   		Library for IC MAX31855
 * @Created on		08/05/2022
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/
/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include <stdbool.h>

#include "MAX31855 r0.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/
#define BUFFER_MAX31855	20
#define REQUEST_TEMPERATURE_MAX31855 1000//ms
/*########### ENUMS ###########*/


typedef enum
{
	MAX31855_IDLE,
	MAX31855_READY,
	MAX31855_RECEIVING,
	MAX31855_BUFFER_COMPLETE,

} eMAX31855_SM;

/*########### STRUCT ###########*/
union
{
	struct
	{
		unsigned OC_Fault :1;
		unsigned SCG_FAULT :1;
		unsigned SCV_Fault :1;
		unsigned Reserved0 :1;
		signed InternalTemperature :12;
		unsigned Fault :1;
		unsigned Reserved1 :1;
		signed ExternalTemperature :14;
	};
	uint32_t Data;
} uData_MAX31855;

struct
{
	eMAX31855_ERRORS Error;
	eMAX31855_SM StateMachineMAX31855;
	float ExternalTemperature[BUFFER_MAX31855];
	float InternalTemperature[BUFFER_MAX31855];
	uint32_t HasDataToRead;
} sManager_MAX31855, sTempManager_MAX31855;
/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/
volatile uint16_t Cnt1ms_MAX31855 = 0,CntIndexMAX31855 = 0;
GPIO_TypeDef *GPIO_CS = { 0 };
uint16_t Pin_CS = 0;
SPI_HandleTypeDef *Hspi;
uint8_t DataSPI[BUFFER_MAX31855 * 4] = { 0 };

/********************************************************************************
 ******* PRIVATE FUNCTIONS
 *******************************************************************************/
void InitAquisition_MAX31855(void);
void HandlerAquisition_MAX31855(void);
void Handler_MAX31855(void);




void Init_MAX31855(SPI_HandleTypeDef *_hspi1, GPIO_TypeDef *_GPIO_CS, uint16_t _Pin_CS)
{
	GPIO_CS = _GPIO_CS;
	Pin_CS = _Pin_CS;
	Hspi = _hspi1;
	sManager_MAX31855.StateMachineMAX31855 = MAX31855_READY;
	Cnt1ms_MAX31855 = 0;
	InitAquisition_MAX31855();
}

void InitAquisition_MAX31855(void)
{
	CntIndexMAX31855 = 0;
	sManager_MAX31855.StateMachineMAX31855 = MAX31855_READY;
	HandlerAquisition_MAX31855();
}

uint8_t tempDataSPI[4] =
{ 0 };
void HandlerAquisition_MAX31855(void)
{
	uint32_t TimeDelay = HAL_RCC_GetHCLKFreq() <= 1E6 ? 1 : HAL_RCC_GetHCLKFreq() / 1E6; //1us
	if (CntIndexMAX31855 < BUFFER_MAX31855)
	{
		if (sManager_MAX31855.StateMachineMAX31855 == MAX31855_READY)
		{
			for (uint32_t i = 0; i < TimeDelay; i++)
				asm("NOP");
			HAL_GPIO_WritePin(GPIO_CS, Pin_CS, GPIO_PIN_RESET); // Low State for SPI Communication
			for (uint32_t i = 0; i < TimeDelay; i++)
				asm("NOP");
			HAL_SPI_Receive_DMA(Hspi, &tempDataSPI[0], 4); // DATA Transfer - Buffer Normal
			sManager_MAX31855.StateMachineMAX31855 = MAX31855_RECEIVING;
		}
	}
	else
		Handler_MAX31855();

}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(GPIO_CS, Pin_CS, GPIO_PIN_SET); // High State for SPI Communication
	for (uint16_t i = 0; i < 4; i++)
	{
		DataSPI[CntIndexMAX31855 * 4 + i] = tempDataSPI[i];
		tempDataSPI[i] = 0;
	}
	CntIndexMAX31855++;
	sManager_MAX31855.StateMachineMAX31855 = MAX31855_READY;
	HandlerAquisition_MAX31855();

}
void Handler_MAX31855(void)
{
	int16_t i = 0, j = 0;
	HAL_GPIO_WritePin(GPIO_CS, Pin_CS, GPIO_PIN_SET); // High State for SPI Communication
	for (i = 0, j = 0; i < BUFFER_MAX31855; i++, j += 4)
	{
		uData_MAX31855.Data = DataSPI[j] << 24 | DataSPI[j + 1] << 16 | DataSPI[j + 2] << 8 | DataSPI[j + 3];
		sManager_MAX31855.InternalTemperature[i] = (float) uData_MAX31855.InternalTemperature / 16.0; //Resolution 0.0625°C/bit = 1/16
		sManager_MAX31855.ExternalTemperature[i] = (float) uData_MAX31855.ExternalTemperature / 4.0; //Resolution 0.25°C/bit = 1/4
	}
	for (i = 0; i < BUFFER_MAX31855 * 4; i++)
		DataSPI[i] = 0;

	if (uData_MAX31855.Fault)
	{
		if (uData_MAX31855.OC_Fault)
			sManager_MAX31855.Error = ERROR_OPEN;
		else if (uData_MAX31855.SCG_FAULT)
			sManager_MAX31855.Error = ERROR_SHORTCIRCUIT_GND;
		else if (uData_MAX31855.SCV_Fault)
			sManager_MAX31855.Error = ERROR_SHORTCIRCUIT_VCC;
		else
			sManager_MAX31855.Error = ERROR_UNKNOWN;
	}
	else
		sManager_MAX31855.Error = ERROR_NONE_MAX31855;

	CntIndexMAX31855 = 0;
	sManager_MAX31855.StateMachineMAX31855 = MAX31855_BUFFER_COMPLETE;
}

void InterruptControl_MAX31855(void)
{
	if (sManager_MAX31855.StateMachineMAX31855 != MAX31855_IDLE)
	{
		if (Cnt1ms_MAX31855 >= REQUEST_TEMPERATURE_MAX31855)
		{
			if (sManager_MAX31855.StateMachineMAX31855 == MAX31855_BUFFER_COMPLETE)
			{
				sManager_MAX31855.HasDataToRead = true;
				Cnt1ms_MAX31855 = 0;
				sTempManager_MAX31855 = sManager_MAX31855;
				sManager_MAX31855.StateMachineMAX31855 = MAX31855_READY;
				InitAquisition_MAX31855();
			}
		}
		else
			Cnt1ms_MAX31855++;

	}
}

float GetTemperatureExternal_MAX31855(void)
{
	if (sManager_MAX31855.HasDataToRead)
	{
		sManager_MAX31855.HasDataToRead = false;
		int16_t i = 0;
		float fTemp = 0.0;
		for (i = 0; i < BUFFER_MAX31855; i++)
			fTemp += sTempManager_MAX31855.ExternalTemperature[i];
		return (fTemp / BUFFER_MAX31855);
	}
	return 0.0;
}

float GetTemperatureInternal_MAX31855(void)
{
	if (sManager_MAX31855.HasDataToRead)
	{
		sManager_MAX31855.HasDataToRead = false;
		int16_t i = 0;
		float fTemp = 0.0;
		for (i = 0; i < BUFFER_MAX31855; i++)
			fTemp += sTempManager_MAX31855.InternalTemperature[i];
		return (fTemp / BUFFER_MAX31855);
	}
	return 0.0;
}

eMAX31855_ERRORS GetError_MAX31855(void)
{
	return sTempManager_MAX31855.Error;
}

