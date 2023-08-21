/********************************************************************************
 * @file    		Full Duplex USART r0.c
 * @author  		Lucas Andrade
 * @brief   		Library for communication full duplex using USART
 * @Created on		20/08/2023
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include "Full Duplex USART r0.h"
#include <stdbool.h>
#include "stm32g4xx_hal_uart.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/
union
{
	struct
	{
		unsigned Initialized :1;
		unsigned ChecksumOK :1;
		unsgined FullBuf :1;
		unsigned dummy :1;
	} Bits;
	uint16_t AllFlags;
} FDUFlags;
struct
{
	uint8_t FDUBufferLines[FDUSART_SIZE_MAX_MESSAGE];
	uint8_t FDUTimeRetransmit;
	uint8_t FDUErrorTimoutCnt;
} FDUManager[FDUSART_BUFFFER_LINES];

UART_HandleTypeDef *huart;

uint8_t FDUUnusedLine = 0;
uint8_t FDUUnusedLine = 0;

/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

size_t FullDuplexUsart_Init(UART_HandleTypeDef *_huart)
{
	huart = _huart;
	FDUFlags.AllFlags = 0;

	FDUFlags.Bits.Initialized = true;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	//Se receber completamente os dados
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
