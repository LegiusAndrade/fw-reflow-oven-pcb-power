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

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include "stdint.h"
#include "stddef.h"

#include "stm32g4xx.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/


typedef struct
{
	UART_HandleTypeDef *uart;
	uint16_t size_buffer;
} FD_CONFIG_t;


/* Typedef used to opaque pointer to FD handler */
typedef struct FD_struct_t FD_t;

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/





/********************************************************************************
 ******* PROTOTYPE OF FUNCTIONS
 *******************************************************************************/

FD_t* FDUSART_Init(const FD_CONFIG_t *config);
void FDUSART_DeInit(FD_t **fd);

size_t FDUSART_InterruptControl(FD_t *FDInstance);

size_t FDUSART_SendMessage(FD_t *FDInstance , uint8_t Cmd, uint8_t *Buf, size_t Len);
size_t FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len);

#ifdef __cplusplus
}
#endif

#endif

