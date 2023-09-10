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

#define FDUSART_BUFFER_LINES					20 			// Size array buffer
#define FDUSART_SIZE_MAX_MESSAGE				50			// MAX size in one line of the buffer
#define FDUSART_BUFFER_RECEIVE					20			// Size pack receive

#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	25			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR						5			// Max error if not receive message
#define FDUSART_TX_TIMEOUT						10			// Timeout each byte sent
#define FDUSART_RX_TIMEOUT						100			// Timeout receive message complete

#define MESSAGE_ID_SEND							0xBEBE		// Protocol init message for send
#define PROTOCOL_VERSION						100			// Protocol version: 1.00
#define SIZE_HEADER								11			// 2Bytes (ID) 	 + 1Byte (VERSION) + 1 Byte (TYPE MESSAGE) + 2Bytes (SEQUENCE NUMBER) + 1Byte (COMMAND MESSAGE) + 1Byte (SIZE MESSAGE) + 2Bytes (CRC)

typedef enum
{
	LINE_FREE = 0, LINE_READY_FOR_SEND, LINE_SENT,
} LINE_STATE_t;

typedef enum
{
	MESSAGE_SEND = 0x00, MESSAGE_ACK,
} TYPE_MESSAGE_t;

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
	TYPE_MESSAGE_t type_message;
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
	uint8_t buffer_received[FDUSART_SIZE_MAX_MESSAGE];
	uint16_t cnt_byte_received;
	uint16_t size_message;
	uint16_t time_receiving;

	uint8_t message_ready_for_read[FDUSART_BUFFER_RECEIVE][FDUSART_SIZE_MAX_MESSAGE];
	uint8_t cnt_messages_for_read;

	union
	{
		struct
		{
			unsigned rx_receiving :1;
			unsigned header_ok :1;
			unsigned version_protocol_ok :1;
			unsigned size_buffer_ok :1;
			unsigned checksum_ok :1;
			unsigned dummy :3;
		} bits;
		uint8_t all_flags;
	} Flags;
} FD_Receive_struct_t;

volatile FD_Receive_struct_t gInstances_Receiving[MAX_UART_INSTANCES] =
{ 0 };
volatile FD_t *gInstances[MAX_UART_INSTANCES] =
{ NULL };

uint8_t gTemp_Message_Receive[FDUSART_SIZE_MAX_MESSAGE] =
{ 0 };

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

	FDInstance->link[NumberLine].buffer_line[0] = 0;
	FDInstance->link[NumberLine].buffer_line[1] = 0;
	FDInstance->link[NumberLine].error_timeout_cnt = 0;
	FDInstance->link[NumberLine].line_state = LINE_FREE;
	FDInstance->link[NumberLine].time_retransmit = 0;

	return success;
}

int16_t _FDUSART_SearchIndexSequenceNumber(FD_t *FDInstance, uint16_t SequenceNumber)
{
	int16_t index_with_sequence_number = -1;

	for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
	{
		uint16_t sequence_number = ((uint16_t) FDInstance->link[i].buffer_line[4] << 8 | FDInstance->link[i].buffer_line[5]);
		if (sequence_number == SequenceNumber)
		{
			index_with_sequence_number = i;
			break;
		}
	}
	return index_with_sequence_number;
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

bool _FDUSART_CheckHeader(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[0] == (uint8_t) (MESSAGE_ID_SEND & 0xFF00) >> 8 && gInstances_Receiving[index].buffer_received[1] == (uint8_t) (MESSAGE_ID_SEND & 0x00FF))
	{
		return true;
	}
	return false;
}

bool _FDUSART_CheckProtocolVersion(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[2] == PROTOCOL_VERSION)
	{
		return true;
	}
	return false;
}

bool _FDUSART_CheckSizeMessage(size_t index)
{
	gInstances_Receiving[index].size_message = ((uint16_t) gInstances_Receiving[index].buffer_received[7] << 8 | gInstances_Receiving[index].buffer_received[8]);
	if (gInstances_Receiving[index].size_message <= (gInstances[index]->size_buffer - SIZE_HEADER))
	{
		return true;
	}
	return false;
}

void _FDUSART_ResetReceiving(size_t index)
{
	gInstances_Receiving[index].Flags.bits.rx_receiving = false;
	gInstances_Receiving[index].cnt_byte_received = 0;
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

	if (success)
	{
		size_t i = 0;
		// Find an empty slot for the new instance
		for (i = 0; i < MAX_UART_INSTANCES; i++)
		{
			if (gInstances[i] == NULL)
			{
				gInstances[i] = fd_return;

				HAL_UART_Receive_DMA(fd_return->uart, gTemp_Message_Receive, 1);
			}
		}
		if (i >= MAX_UART_INSTANCES)
		{ // Not slot empty
			FDUSART_DeInit(&fd_return);
			success = false;
		}
	}
	return fd_return;
}

void FDUSART_DeInit(FD_t **fd)
{
	assert(*fd != NULL);

	size_t i = 0;
	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		if (gInstances[i] && gInstances[i]->uart->Instance == (*fd)->uart->Instance)
		{
			gInstances[i] = NULL;
			break;
		}
	}

	for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
	{
		if ((*fd)->link[i].buffer_line != NULL)
		{
			free((*fd)->link[i].buffer_line);
			(*fd)->link[i].buffer_line = NULL;
		}
	}

	free(*fd);
	*fd = NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	size_t i = 0;
	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		if (gInstances[i] && gInstances[i]->uart->Instance == huart->Instance)
		{
			break;
		}
	}
	if (i >= MAX_UART_INSTANCES)
		return;

	if (gInstances_Receiving[i].Flags.bits.rx_receiving == false)
	{
		gInstances_Receiving[i].time_receiving = 0;
		gInstances_Receiving[i].Flags.bits.rx_receiving = true;
		gInstances_Receiving[i].Flags.all_flags = 0;
	}

	//Copy message received for buffer
	gInstances_Receiving[i].buffer_received[gInstances_Receiving[i].cnt_byte_received++] = gTemp_Message_Receive[0];

	/* Check header */
	if (!gInstances_Receiving[i].Flags.bits.header_ok)
	{
		if (gInstances_Receiving[i].cnt_byte_received >= 2)
		{
			if (!_FDUSART_CheckHeader(i))
			{
				_FDUSART_ResetReceiving(i);
				return;
			}
			gInstances_Receiving[i].Flags.bits.header_ok = true;
		}
	}

	/* Check version protocol */
	if (!gInstances_Receiving[i].Flags.bits.version_protocol_ok)
	{
		if (gInstances_Receiving[i].cnt_byte_received >= 3)
		{
			if (!_FDUSART_CheckProtocolVersion(i))
			{
				_FDUSART_ResetReceiving(i);
				return;
			}
			gInstances_Receiving[i].Flags.bits.version_protocol_ok = true;
		}
	}

	/* Check size message */
	if (!gInstances_Receiving[i].Flags.bits.size_buffer_ok)
	{
		if (gInstances_Receiving[i].cnt_byte_received >= 9)
		{
			if (!_FDUSART_CheckSizeMessage(i))
			{
				_FDUSART_ResetReceiving(i);
				return;
			}
			gInstances_Receiving[i].Flags.bits.size_buffer_ok = true;
		}
	}

	if (gInstances_Receiving[i].cnt_byte_received >= gInstances_Receiving[i].size_message + SIZE_HEADER)
	{
		uint16_t index_free = gInstances_Receiving[i].cnt_messages_for_read++;
		if (index_free > FDUSART_BUFFER_RECEIVE)
		{ //Buffer full
			gInstances_Receiving[i].cnt_messages_for_read = 0;
			index_free = 0;
		}
		memcpy((void*) gInstances_Receiving[i].message_ready_for_read[index_free], (void*) gInstances_Receiving[i].buffer_received, gInstances_Receiving[i].size_message + SIZE_HEADER);
		_FDUSART_ResetReceiving(i);
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	size_t i = 0;
	size_t j = 0;

	FD_t *instance = NULL;
	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		instance = (void*) gInstances[i];
		if (instance && instance->uart->Instance == huart->Instance)
		{
			// Do something with instance
			break;
		}
	}

	if (i >= MAX_UART_INSTANCES)
	{
// Not found instance
		return;
	}

	uint16_t sequence_number_sent = huart->pTxBuffPtr[4] << 8 | huart->pTxBuffPtr[5];
	uint16_t sequence_number_index = 0;

	for (j = 0; j < FDUSART_BUFFER_LINES; j++)
	{
		sequence_number_index = instance->link[j].buffer_line[4] << 8 | instance->link[j].buffer_line[5];
		if (sequence_number_sent == sequence_number_index)
		{
			// Found line sent
			break;
		}
	}

	if (j >= FDUSART_BUFFER_LINES)
	{
//All lines used
		return;
	}
	instance->link[j].line_state = LINE_SENT;

}

size_t FDUSART_InterruptControl(FD_t *FDInstance)
{
	size_t success = false;
	FD_t *instance = FDInstance;
	size_t i;

	// Transmission
	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		instance = (void*) gInstances[i];
		if (instance != NULL)
		{
			for (size_t line = 0; line < FDUSART_BUFFER_LINES; line++)
			{
				/* Verifying if transmition mode */
				if (!instance->flags.bits.send_message)
				{
					/* Search line ready for send */
					if (instance->link[line].line_state == LINE_READY_FOR_SEND)
					{
						/* Activated DMA for transmit message */
						success = _FDUSART_TransmitMessage(instance, line);
					}
				}

				if (instance->link[line].line_state == LINE_SENT)
				{ // If message sent and not received response
					instance->link[line].time_retransmit--; //Decrement time for retransmit
					if (instance->link[line].time_retransmit == 0)
					{
						instance->link[line].error_timeout_cnt++; //Increment number of tentative
						if (instance->link[line].error_timeout_cnt >= FDUSART_MAX_ERROR) //If exceded tentatives
						{
							_FDUSART_ClearLine(instance, line);
						}
						else
						{
							_FDUSART_MakeToSend(instance, line);
						}
					}
				}
			}
		}
	}

	//Reception
	for (i = 0; i < MAX_UART_INSTANCES; i++)
	{
		if (gInstances_Receiving[i].Flags.bits.rx_receiving == true)
		{ // Verify timeout receiver message
			gInstances_Receiving[i].time_receiving++;
			if (gInstances_Receiving[i].time_receiving >= FDUSART_RX_TIMEOUT)
			{
				_FDUSART_ResetReceiving(i);
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

// ID | VERSION | TYPE MESSAGE | SEQUENCE NUMBER | MESSAGE COMMAND | SIZE MESSAGE | MESSAGE | CRC
	FDInstance->link[line_free_id].buffer_line[0] = (uint8_t) (MESSAGE_ID_SEND & 0xFF00) >> 8; // Lines writted with MESSAGE_ID
	FDInstance->link[line_free_id].buffer_line[1] = (uint8_t) (MESSAGE_ID_SEND & 0x00FF);
	FDInstance->link[line_free_id].buffer_line[2] = PROTOCOL_VERSION;
	FDInstance->link[line_free_id].buffer_line[3] = FDInstance->link[line_free_id].type_message;
	FDInstance->link[line_free_id].buffer_line[4] = (FDInstance->sequence_number & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[5] = (FDInstance->sequence_number & 0x00FF);
	FDInstance->link[line_free_id].buffer_line[6] = Cmd;
	FDInstance->link[line_free_id].buffer_line[7] = (uint8_t) (Len & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[8] = (uint8_t) (Len & 0x00FF);
	memcpy(&FDInstance->link[line_free_id].buffer_line[9], Buf, Len);

	uint16_t crc = CRC16_CCITT_Calculate(FDInstance->link[line_free_id].buffer_line, Len + SIZE_HEADER); //Calculate CRC based into message and header

	FDInstance->link[line_free_id].buffer_line[Len + 9] = (uint8_t) (crc & 0xFF00) >> 8;
	FDInstance->link[line_free_id].buffer_line[Len + 10] = (uint8_t) (crc & 0x00FF);

	FDInstance->link[line_free_id].message_length = Len + SIZE_HEADER;

	FDInstance->link[line_free_id].type_message = MESSAGE_SEND;

	_FDUSART_MakeToSend(FDInstance, line_free_id);

	success = true;

	return success;
}

size_t FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{

	size_t success = false;
	size_t search_instance;

	for (search_instance = 0; search_instance < MAX_UART_INSTANCES; search_instance++)
	{
		if (gInstances[search_instance]->uart->Instance == FDInstance->uart->Instance)
		{
			break;
		}
	}

	if (search_instance >= MAX_UART_INSTANCES)
	{
		return success;
	}

	int16_t index_message_for_read = gInstances_Receiving[search_instance].cnt_messages_for_read;
	while (index_message_for_read > 0)
	{ //If exists message for read

		// Extract message details: sequence number, command, and size.
		uint16_t sequence_number = (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][4] << 8)
				| gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][5];

		*Cmd = gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][6];
		*Len = (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][7] << 8) | gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][8];

		//If new message, send ACK
		if (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][3] == MESSAGE_SEND)
		{
			gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][3] = MESSAGE_ACK;
			FDUSART_SendMessage((void*) gInstances[search_instance], *Cmd, (void*) &gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][9], *Len);
		}
		else
		{ // If the message is just an ACK, delete the message and subtract it from the message counter
			index_message_for_read = --gInstances_Receiving[search_instance].cnt_messages_for_read;
			int16_t index_sequence_number = _FDUSART_SearchIndexSequenceNumber((void*) &FDInstance, sequence_number);
			_FDUSART_ClearLine((void*) &FDInstance, index_sequence_number);
			continue;
		}

		// Validate CRC
		uint16_t received_crc = (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][*Len + 9] << 8)
				| gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][*Len + 10];
		uint16_t calculated_crc = CRC16_CCITT_Calculate((void*) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read], *Len + SIZE_HEADER);

		if (received_crc != calculated_crc)
		{
			// invalid CRC, you might want to log this error or notify the user
			index_message_for_read = --gInstances_Receiving[search_instance].cnt_messages_for_read;
			continue;
		}

		// Copy the validated message to the provided buffer
		memcpy(Buf, (void*) &gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][9], *Len);

		gInstances_Receiving[search_instance].cnt_messages_for_read--;

		success = true;
		break;
	}

	return success;
}

