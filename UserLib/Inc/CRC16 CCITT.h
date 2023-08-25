/********************************************************************************
 * @file    		CRC16 CCITT.h
 * @author  		Lucas Andrade
 * @brief   		Library for calculate CRC16
 * @Created on		24/08/2023
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/

#ifndef INC_CRC16_CCITT_H_
#define INC_CRC16_CCITT_H_

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include <stdint.h>
#include <stddef.h>
/********************************************************************************
 ******* PROTOTYPE OF FUNCTIONS
 *******************************************************************************/
uint16_t CRC16_CCITT_Calculate(const uint8_t * data, size_t lenght);

#endif /* INC_CRC16_CCITT_H_ */
