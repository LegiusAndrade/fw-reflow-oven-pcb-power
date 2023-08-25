/********************************************************************************
 * @file    		Full Duplex USART r0.c
 * @author  		Lucas Andrade
 * @brief   		Library for communication full duplex using USART
 * @Created on		20/08/2023
 * @Version			V1.0 r0
 * @History
 *					r0: Initial release
 ********************************************************************************/
/* IDEA
 *
 * ID | VERSION | SEQUENCE NUMBER | MESSAGE COMMAND | SIZE MESSAGE | MESSAGE | CRC
 * ID: Este campo age como Início/Fim de Mensagem. Se você escolher, por exemplo, um valor especial que é improvável de ocorrer no meio da mensagem (como 0xFFFF em um protocolo binário), você pode usá-lo para detectar o início e o fim das mensagens.
 * VERSÃO: Campo para indicar a versão do protocolo. Útil para compatibilidade futura.
 * NUMERO SEQUÊNCIA: Um contador que aumenta a cada mensagem enviada. Isso ajuda a identificar se alguma mensagem foi perdida ou para manter a ordem das mensagens.
 * COMANDO DA MENSAGEM: Define o tipo de mensagem ou a ação que ela representa.
 * TAMANHO DA MENSAGEM: Indica o tamanho da parte da mensagem, o que permite ao receptor saber quanto espaço alocar e quando a mensagem termina.
 * MENSAGEM: Os dados que você deseja transmitir.
 * CRC: Verifica a integridade da mensagem.
 *
 * */

/********************************************************************************
 ******* INCLUDES
 *******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <stdlib.h>

#include "Full Duplex USART r0.h"

#include "CRC16 CCITT.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/
#define	MAX_UART_INSTANCES						10			// Max instances USART in same time in this library

#define FDUSART_BUFFER_LINES					15 			// Size array buffer
#define FDUSART_SIZE_MAX_MESSAGE				256			// MAX size in one line of the buffer
#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	25			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR						5			// Max error if not receive message
#define FDUSART_TX_TIMEOUT						10			// Timeout each byte sent

#define MESSAGE_ID_SEND							0xBEBE		// Protocol init message for send
#define PROTOCOL_VERSION						100			// Protocol version: 1.00
#define SIZE_HEADER								10			// 2Bytes (ID) 	 + 1Byte (VERSION) + 2Bytes (SEQUENCE NUMBER) + 1Byte (COMMAND MESSAGE) + 1Byte (SIZE MESSAGE) + 2Bytes (CRC)

typedef enum
{
	LINE_FREE = 0, LINE_READY_FOR_SEND, LINE_SENT,
} LINE_STATE_t;

typedef union
{
	struct
	{
		unsigned initialized :1;
		unsigned send_message :1;
		unsigned checksum_ok :1;
		unsigned full_buf :1;
		unsigned dummy :12;
	} bits;
	uint16_t all_flags;
} FD_FLAGS_t;

typedef struct
{
	LINE_STATE_t line_state;
	uint8_t *buffer_line;
	uint8_t time_retransmit;
	uint8_t error_timeout_cnt;
	size_t message_length;
} FD_LINK_t;

struct FD_struct_t
{
	FD_FLAGS_t flags;
	uint16_t sequence_number;
	FD_LINK_t link[FDUSART_BUFFER_LINES];
	UART_HandleTypeDef *uart;
	uint16_t size_buffer;
};

typedef struct
{
	UART_HandleTypeDef *uart;
	uint16_t sequence_number_sent[FDUSART_BUFFER_LINES];
} INSTANCES_t;

INSTANCES_t *gInstances[MAX_UART_INSTANCES];
/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/

/********************************************************************************
 ******* PROTOTYPE PRIVATE FUNCTIONS
 *******************************************************************************/
size_t _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree);
size_t _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine);
void _FDUSART_MakeToSend(FD_t *FDInstance, uint16_t NumberLine);
void _FDUSART_SetFlagFullBuf(FD_t *FDInstance, uint8_t flag_value);
size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line);

/********************************************************************************
 ******* PRIVATE FUNCTIONS
 *******************************************************************************/
size_t _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree)
{
	size_t success = false;
	uint16_t line_free_id = 0;
	for (line_free_id = 0; line_free_id < FDUSART_BUFFER_LINES; line_free_id++)
	{
		if (FDInstance->link[line_free_id].line_state == LINE_FREE) // Free line
		{
			*LineFree = line_free_id;
			success = true;
			break;
		}
	}
	return success;
}

size_t _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine)
{
	size_t success = false;
	size_t i = 0;

	FDInstance->link[NumberLine].buffer_line[0] = 0;
	FDInstance->link[NumberLine].buffer_line[1] = 0;
	FDInstance->link[NumberLine].error_timeout_cnt = 0;
	FDInstance->link[NumberLine].line_state = LINE_FREE;
	FDInstance->link[NumberLine].time_retransmit = 0;

	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		if (gInstances[i]->uart->Instance == FDInstance->uart->Instance)
		{
			break;
		}
	}

	if (i < MAX_UART_INSTANCES)
		// Found instance
		for (size_t j = 0; j < FDUSART_BUFFER_LINES; j++)
		{
			if (gInstances[i]->sequence_number_sent[j] == 0)
			{
				gInstances[i]->sequence_number_sent[j] = 0;
				success = true;
				break;
			}
		}

	return success;
}

void _FDUSART_MakeToSend(FD_t *FDInstance, uint16_t NumberLine)
{
	FDInstance->link[NumberLine].line_state = LINE_READY_FOR_SEND;
	FDInstance->link[NumberLine].time_retransmit = FDUSART_TIME_RETRANSMIT_IF_NORESPONSE;
}

void _FDUSART_SetFlagFullBuf(FD_t *FDInstance, uint8_t flag_value)
{
	FDInstance->flags.bits.full_buf = (flag_value & 0x01);
}

size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line)
{
	size_t success = false;

	HAL_UART_Transmit_DMA(FDInstance->uart, FDInstance->link[line].buffer_line, FDInstance->link[line].message_length);
	FDInstance->flags.bits.send_message = true;

	success = true;
	return success;
}
/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

FD_t* FDUSART_Init(const FD_CONFIG_t *config)
{
	assert(config != NULL);

	size_t success = false;

	FD_t *fd_return;

	fd_return = (FD_t*) malloc(sizeof(FD_t));

	if (fd_return != NULL)
	{
		fd_return->flags.all_flags = 0;
		fd_return->flags.bits.initialized = true;
		fd_return->uart = config->uart;

		if (config->size_buffer > FDUSART_SIZE_MAX_MESSAGE)
		{
			success = false;
		}
		else
		{
			fd_return->size_buffer = config->size_buffer;
			// Allocated dinamic memeory
			for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
			{
				fd_return->link[i].buffer_line = (uint8_t*) malloc(fd_return->size_buffer + SIZE_HEADER);
				if (fd_return->link[i].buffer_line == NULL)
				{
					// Fail alocated memeory
					success = false;
					break;
				}
			}
			success = true;
		}

	}
	else
	{
		fd_return = NULL;
	}

	if (!success)
	{
		if (fd_return != NULL)
		{
			free(fd_return);
			fd_return = NULL;
		}
	}
	return fd_return;
}

void FDUSART_DeInit(FD_t **fd)
{
	assert(*fd != NULL);
	for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
	{
		free((*fd)->link[i].buffer_line);
	}

	free(*fd);
	*fd = NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	size_t i = 0;
	size_t j = 0;

	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		if (gInstances[i]->uart->Instance == huart->Instance)
		{
			break;
		}
	}

	if (i >= MAX_UART_INSTANCES)
	{
		// Not found intance
		return;
	}

	for (j = 0; j < FDUSART_BUFFER_LINES; j++)
	{
		if (gInstances[i]->sequence_number_sent[j] == 0)
		{
			gInstances[i]->sequence_number_sent[j] = gInstances[i]->uart->pTxBuffPtr[3] << 8 | gInstances[i]->uart->pTxBuffPtr[4];
			break;
		}
	}

	if (j >= FDUSART_BUFFER_LINES)
	{
		//All lines used
		return;
	}
}

size_t FDUSART_InterruptControl(FD_t *FDInstance)
{
	size_t success = false;

	for (size_t line = 0; line < FDUSART_BUFFER_LINES; line++)
	{
		/* Verifying if transmition mode */
		if (!FDInstance->flags.bits.send_message)
		{
			/* Search line ready for send */
			if (FDInstance->link[line].line_state == LINE_READY_FOR_SEND)
			{
				/* Activated DMA for transmit message */
				_FDUSART_TransmitMessage(FDInstance, line);
			}
		}

		if (FDInstance->link[line].line_state == LINE_SENT)
		{ // If message sent and not received response
			FDInstance->link[line].time_retransmit--; //Decrement time for retransmit
			if (FDInstance->link[line].time_retransmit == 0)
			{
				FDInstance->link[line].error_timeout_cnt++; //Increment number of tentative
				if (FDInstance->link[line].error_timeout_cnt >= FDUSART_MAX_ERROR) //If exceded tentatives
				{
					_FDUSART_ClearLine(FDInstance, line);
				}
				else
				{
					_FDUSART_MakeToSend(FDInstance, line);
				}
			}
		}
	}


	return success;

}

size_t FDUSART_SendMessage(FD_t *FDInstance, uint8_t Cmd, uint8_t *Buf, size_t Len)
{
	uint16_t line_free_id = 0;
	size_t success = false;

	/* All lines occupied */
	if (_FDUSART_SearchLineFree(FDInstance, &line_free_id))
	{
		_FDUSART_SetFlagFullBuf(FDInstance, true);
		return success;
	}

	/* Len outside the maximum size */
	if (Len > FDInstance->size_buffer || Len > FDUSART_SIZE_MAX_MESSAGE)
	{
		return success;
	}

	/* Sequence number value */
	if (FDInstance->sequence_number == UINT16_MAX)
	{
		FDInstance->sequence_number = 1;
	}
	else
	{
		FDInstance->sequence_number++;
	}

	_FDUSART_ClearLine(FDInstance, line_free_id);

//* ID | VERSION | SEQUENCE NUMBER | MESSAGE COMMAND | SIZE MESSAGE | MESSAGE | CRC
	FDInstance->link[line_free_id].buffer_line[0] = (uint8_t) (MESSAGE_ID_SEND & 0xFF00) >> 8; // Lines writted with MESSAGE_ID
	FDInstance->link[line_free_id].buffer_line[1] = (uint8_t) (MESSAGE_ID_SEND & 0x00FF);
	FDInstance->link[line_free_id].buffer_line[2] = PROTOCOL_VERSION;
	FDInstance->link[line_free_id].buffer_line[3] = (FDInstance->sequence_number & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[4] = (FDInstance->sequence_number & 0x00FF);
	FDInstance->link[line_free_id].buffer_line[5] = Cmd;
	FDInstance->link[line_free_id].buffer_line[6] = (uint8_t) (Len & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[7] = (uint8_t) (Len & 0x00FF);
	memcpy(&FDInstance->link[line_free_id].buffer_line[8], Buf, Len);

	uint16_t crc = CRC16_CCITT_Calculate(FDInstance->link[line_free_id].buffer_line, Len + SIZE_HEADER); //Calculate CRC based into message and header

	FDInstance->link[line_free_id].buffer_line[Len + 8] = (uint8_t) (crc & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[Len + 9] = (uint8_t) (crc & 0x00FF);

	FDInstance->link[line_free_id].message_length = Len + SIZE_HEADER;

	_FDUSART_MakeToSend(FDInstance, line_free_id);

	success = true;

	return success;
}

size_t FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{
	uint16_t line_received_id = 0;

	size_t success = false;

	if (FDInstance->link[line_received_id].buffer_line[0] != (uint8_t) (MESSAGE_ID_SEND & 0xFF00) >> 8
			|| FDInstance->link[line_received_id].buffer_line[1] != (uint8_t) (MESSAGE_ID_SEND & 0x00FF))
	{
		// invalid ID
		return success;
	}

// Validate protocol version
	if (FDInstance->link[line_received_id].buffer_line[2] != PROTOCOL_VERSION)
	{
		// protocol version invalid
		return success;
	}

// Read sequence number, command and size message.
	uint16_t sequence_number = (FDInstance->link[line_received_id].buffer_line[3] << 8) | FDInstance->link[line_received_id].buffer_line[4];

	*Cmd = FDInstance->link[line_received_id].buffer_line[5];
	*Len = (FDInstance->link[line_received_id].buffer_line[6] << 8) | FDInstance->link[line_received_id].buffer_line[7];

// Validate size message
	if (*Len > FDInstance->size_buffer || *Len > FDUSART_SIZE_MAX_MESSAGE)
	{
		// size message invalid
		return success;
	}

// check CRC
	uint16_t received_crc = (FDInstance->link[line_received_id].buffer_line[*Len + 8] << 8) | FDInstance->link[line_received_id].buffer_line[*Len + 9];
	uint16_t calculated_crc = CRC16_CCITT_Calculate(FDInstance->link[line_received_id].buffer_line, *Len + SIZE_HEADER);

	if (received_crc != calculated_crc)
	{
		// invalid CRC
		return success;
	}

// Copy message for pointer
	memcpy(Buf, &FDInstance->link[line_received_id].buffer_line[8], *Len);

// Clear line
	success = _FDUSART_ClearLine(FDInstance, line_received_id);

	return success;
}

