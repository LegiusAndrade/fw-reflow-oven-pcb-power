/********************************************************************************
 * @file    		CRC16 CCITT.c
 * @author  		Lucas Andrade
 * @brief   		Library for calculate CRC16
 * @Created on		24/08/2023
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include "CRC16 CCITT.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/
#define POLY 0x1021 // CRC-16-CCITT polynomial: x^16 +x^12 +x^5 +1 // Quando representamos isso em forma binária, obtemos: 0001 0000 0010 0001

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/

/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

uint16_t CRC16_CCITT_Calculate(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xFFFF; // Initial value

	for (size_t i = 0; i < length; i++)
	{
		crc ^= (uint16_t) data[i] << 8; // "Place the current byte into the MSB of the CRC for the XOR.

		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{ // Check if the most significant bit (MSB) is set.
				crc = (crc << 1) ^ POLY;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}

	return crc & 0xFFFF;
}
