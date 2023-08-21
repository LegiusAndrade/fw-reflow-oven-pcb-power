/********************************************************************************
 * @file    		MAX31855.h
 * @author  		Lucas Andrade
 * @brief   		Library for IC MAX31855
 * @Created on		08/05/2022
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/

#ifndef MAX31855_H_
#define MAX31855_H_

/*########### INCLUDES ###########*/
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include <limits.h>

/*########### DEFINES ###########*/
#define BUFFER_MAX31855	20
#define REQUEST_TEMPERATURE_MAX31855 1000//ms

/*########### ENUMS ###########*/
typedef enum
{
	ERROR_NONE_MAX31855 = 0,
	ERROR_SHORTCIRCUIT_VCC,
	ERROR_SHORTCIRCUIT_GND,
	ERROR_OPEN,
	ERROR_UNKNOWN,

} eMAX31855_ERRORS;

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

/*########### VARIABLES ###########*/
uint8_t DataSPI[BUFFER_MAX31855 * 4] = { 0 };
volatile uint16_t Cnt1ms_MAX31855 = 0,CntIndexMAX31855 = 0;
GPIO_TypeDef *GPIO_CS = { 0 };
uint16_t Pin_CS = 0;
SPI_HandleTypeDef *Hspi;

/*########### PROTOTYPE ###########*/
void Init_MAX31855(SPI_HandleTypeDef *_hspi1, GPIO_TypeDef *_GPIO_CS, uint16_t _Pin_CS);
void InitAquisition_MAX31855(void);
void HandlerAquisition_MAX31855(void);
void Handler_MAX31855(void);
void InterruptControl_MAX31855(void);
float GetTemperatureExternal_MAX31855(void);
float GetTemperatureInternal_MAX31855(void);
eMAX31855_ERRORS GetError_MAX31855(void);

#endif /* MAX31855_H_ */
