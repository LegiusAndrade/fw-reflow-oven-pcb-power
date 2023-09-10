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

#ifdef __cplusplus
extern "C" {
#endif

/*########### INCLUDES ###########*/
#include "limits.h"

#include "stm32g4xx.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/

typedef enum
{
	ERROR_NONE_MAX31855 = 0,
	ERROR_SHORTCIRCUIT_VCC,
	ERROR_SHORTCIRCUIT_GND,
	ERROR_OPEN,
	ERROR_UNKNOWN,

} eMAX31855_ERRORS;

/*########### PROTOTYPE ###########*/
void Init_MAX31855(SPI_HandleTypeDef *_hspi1, GPIO_TypeDef *_GPIO_CS, uint16_t _Pin_CS);

void InterruptControl_MAX31855(void);
float GetTemperatureExternal_MAX31855(void);
float GetTemperatureInternal_MAX31855(void);
eMAX31855_ERRORS GetError_MAX31855(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX31855_H_ */
