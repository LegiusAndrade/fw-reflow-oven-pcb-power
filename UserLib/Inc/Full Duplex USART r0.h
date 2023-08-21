/********************************************************************************
 * @file    		Full Duplex USART r0.h
 * @author  		Lucas Andrade
 * @brief   		Library for communication full duplex using USART
 * @Created on		20/08/2023
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/

#ifndef FULL_DUPLEX_USART_H_
#define FULL_DUPLEX_USART_H_

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include <stddef.h>
#include <stm32g4xx_hal_uart.h>

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/
#define FDUSART_BUFFFER_LINES			15 					// Size array buffer
#define FDUSART_SIZE_MAX_MESSAGE		100					// Size in one line of the buffer
#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	25			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR									// Max error if not receive message
#define FDUSART_TX_TIMEOUT				10					// Timeout each byte sent

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/





/********************************************************************************
 ******* PROTOTYPE OF FUNCTIONS
 *******************************************************************************/

size_t FullDuplexUsart_Init(UART_HandleTypeDef *huart);

#endif

