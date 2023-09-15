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
#define	MAX_UART_INSTANCES						3			// Max instances USART in same time in this library

#define FDUSART_BUFFER_LINES					20 			// Size array buffer
#define FDUSART_SIZE_MAX_MESSAGE				50			// MAX size in one line of the buffer
#define FDUSART_BUFFER_RECEIVE					20			// Size pack receive

#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	25			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR						5			// Max error if not receive message
#define FDUSART_TX_TIMEOUT						10			// Timeout each byte sent
#define FDUSART_RX_TIMEOUT						100			// Timeout receive message complete

#define MESSAGE_ID_SEND							0xBEBE		// Protocol init message for send
#define PROTOCOL_VERSION						100			// Protocol version: 1.00
#define //SIZE_HEADER								11			// 2Bytes (ID) 	 + 1Byte (VERSION) + 1 Byte (TYPE MESSAGE) + 2Bytes (SEQUENCE NUMBER) + 1Byte (COMMAND MESSAGE) + 1Byte (SIZE MESSAGE) + 2Bytes (CRC)

#define PACK_STRUCT 							__attribute__((__packed__))

typedef enum
{
	LINE_FREE = 0, LINE_READY_FOR_SEND, LINE_SENT_DMA, LINE_SENT_EXTRNAL
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

typedef union PACK_STRUCT
{
	uint8_t *data;
	struct PACK_STRUCT
	{
		uint16_t header;
		uint16_t version :10;
		uint8_t type_message :2;
		uint16_t sequence_number;
		uint8_t cmd;
		uint16_t len :10;
		uint16_t crc16;
	} fields;
} FD_MSG_t;

typedef struct
{
	LINE_STATE_t line_state;
	FD_MSG_t *buffer_line;
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
static size_t _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree);
static size_t _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine);
static int16_t _FDUSART_SearchIndexSequenceNumber(FD_t *FDInstance, uint16_t SequenceNumber);
static void _FDUSART_MakeToSend(FD_t *FDInstance, uint16_t NumberLine);
static void _FDUSART_SetFlagFullBuf(FD_t *FDInstance, uint8_t flag_value);
static size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line);
static bool _FDUSART_CheckHeader(size_t index);
static bool _FDUSART_CheckProtocolVersion(size_t index);
static bool _FDUSART_CheckSizeMessage(size_t index);
static void _FDUSART_ResetReceiving(size_t index);
static bool _FDUSART_HandleMessageOrACK(FD_t *FDInstance, size_t search_instance, int16_t index_message_for_read);
static size_t _FDUSART_SearchUARTInstance(FD_t *FDInstance);
static bool _FDUSART_ExtractMessageDetails(FD_MSG_t *message, uint8_t *cmd, size_t *len, size_t *sequence_number);
static bool _FDUSART_ValidateMessageCRC(FD_MSG_t *message, size_t len);

/********************************************************************************
 ******* PRIVATE FUNCTIONS
 *******************************************************************************/
static size_t _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree)
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

static size_t _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine)
{
	size_t success = false;

	FDInstance->link[NumberLine].buffer_line[0] = 0;
	FDInstance->link[NumberLine].buffer_line[1] = 0;
	FDInstance->link[NumberLine].error_timeout_cnt = 0;
	FDInstance->link[NumberLine].line_state = LINE_FREE;
	FDInstance->link[NumberLine].time_retransmit = 0;

	return success;
}

static int16_t _FDUSART_SearchIndexSequenceNumber(FD_t *FDInstance, uint16_t SequenceNumber)
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

static void _FDUSART_MakeToSend(FD_t *FDInstance, uint16_t NumberLine)
{
	FDInstance->link[NumberLine].line_state = LINE_READY_FOR_SEND;
	FDInstance->link[NumberLine].time_retransmit = FDUSART_TIME_RETRANSMIT_IF_NORESPONSE;
}

static void _FDUSART_SetFlagFullBuf(FD_t *FDInstance, uint8_t flag_value)
{
	FDInstance->flags.bits.full_buf = (flag_value & 0x01);
}

static size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line)
{
	size_t success = false;

	HAL_UART_Transmit_DMA(FDInstance->uart, FDInstance->link[line].buffer_line, FDInstance->link[line].message_length);
	FDInstance->flags.bits.send_message = true;
	FDInstance->link[line].line_state = LINE_SENT_DMA;

	success = true;
	return success;
}

static bool _FDUSART_CheckHeader(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[0] == (uint8_t) ((MESSAGE_ID_SEND & 0xFF00) >> 8)
			&& gInstances_Receiving[index].buffer_received[1] == (uint8_t) (MESSAGE_ID_SEND & 0x00FF))
	{
		return true;
	}
	return false;
}

static bool _FDUSART_CheckProtocolVersion(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[2] == PROTOCOL_VERSION)
	{
		return true;
	}
	return false;
}

static bool _FDUSART_CheckSizeMessage(size_t index)
{
	gInstances_Receiving[index].size_message = ((uint16_t) (gInstances_Receiving[index].buffer_received[7] << 8)
			| gInstances_Receiving[index].buffer_received[8]);
	if (gInstances_Receiving[index].size_message <= (gInstances[index]->size_buffer - SIZE_HEADER))
	{
		return true;
	}
	return false;
}

static void _FDUSART_ResetReceiving(size_t index)
{
	gInstances_Receiving[index].Flags.bits.rx_receiving = false;
	gInstances_Receiving[index].cnt_byte_received = 0;
}

static size_t _FDUSART_SendACK(FD_t *FDInstance, uint8_t *Buf, size_t Len)
{
	uint16_t line_free_id = 0;
	size_t success = false;

	/* All lines occupied */
	if (!_FDUSART_SearchLineFree(FDInstance, &line_free_id))
	{
		_FDUSART_SetFlagFullBuf(FDInstance, true);
		return success;
	}

	/* Len outside the maximum size */
	if (Len > FDInstance->size_buffer || Len > FDUSART_SIZE_MAX_MESSAGE)
	{
		return success;
	}

	_FDUSART_ClearLine(FDInstance, line_free_id);

	memcpy(&FDInstance->link[line_free_id].buffer_line[0], Buf, (Len + SIZE_HEADER - 2)); //-2 of CRC

	FDInstance->link[line_free_id].buffer_line[3] = MESSAGE_ACK;

	uint16_t crc = CRC16_CCITT_Calculate(FDInstance->link[line_free_id].buffer_line, Len + SIZE_HEADER - 2); //Calculate CRC based into message and header

	FDInstance->link[line_free_id].buffer_line[Len + SIZE_HEADER - 2] = (uint8_t) (crc >> 8);
	FDInstance->link[line_free_id].buffer_line[Len + SIZE_HEADER - 1] = (uint8_t) crc;

	FDInstance->link[line_free_id].message_length = Len + SIZE_HEADER;

	_FDUSART_MakeToSend(FDInstance, line_free_id);

	success = true;

	return success;
}

static size_t _FDUSART_SearchUARTInstance(FD_t *FDInstance)
{
	size_t search_instance;
	for (search_instance = 0; search_instance < MAX_UART_INSTANCES; search_instance++)
	{
		if (gInstances[search_instance]->uart->Instance == FDInstance->uart->Instance)
			break;
	}
	return search_instance;
}

static bool _FDUSART_ExtractMessageDetails(FD_MSG_t *message, uint8_t *cmd, size_t *len, size_t *sequence_number)
{
	if (!message || !cmd || !len)
		return false;

	*cmd = message->fields.cmd;
	*len = message->fields.len;
	*sequence_number = message->fields.sequence_number;
	return true;
}

static bool _FDUSART_HandleMessageOrACK(FD_t *FDInstance, size_t search_instance, int16_t index_message_for_read)
{
	FD_MSG_t *currentMessageStruct = (FD_MSG_t*) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read];

	if (currentMessageStruct->fields.type_message == MESSAGE_SEND)
	{
		_FDUSART_SendACK(gInstances[search_instance], currentMessageStruct->data);
	}
	else
	{
		int16_t index_sequence_number = _FDUSART_SearchIndexSequenceNumber(FDInstance, currentMessageStruct->fields.sequence_number);
		_FDUSART_ClearLine(FDInstance, index_sequence_number);
		return false;
	}
	return true;
}

static bool _FDUSART_ValidateMessageCRC(FD_MSG_t *message, size_t len)
{
	if (!message)
		return false;

	uint16_t received_crc = message->fields.crc16;

	uint16_t calculated_crc = CRC16_CCITT_Calculate(message->data, len + offsetof(FD_MSG_t, fields.crc16));

	return received_crc == calculated_crc;
}

/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

FD_t* FDUSART_Init(const FD_CONFIG_t *config)
{
	assert(config != NULL);

	bool success = false;

	FD_t *fd_return;

	fd_return = (FD_t*) malloc(sizeof(FD_t));

	if (fd_return != NULL)
	{
		memset(fd_return, 0, sizeof(FD_t));  // Limpa a memória alocada.

		fd_return->flags.all_flags = 0;
		fd_return->flags.bits.initialized = true;
		fd_return->uart = config->uart;

		if (config->size_buffer <= FDUSART_SIZE_MAX_MESSAGE)
		{
			fd_return->size_buffer = config->size_buffer;
			// Allocated dinamic memeory
			for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
			{
				fd_return->link[i].buffer_line = (FD_MSG_t*) malloc(fd_return->size_buffer + SIZE_HEADER);
				if (fd_return->link[i].buffer_line == NULL)
				{
					// Free previously allocated memory
					for (size_t j = 0; j < i; j++)
					{
						free(fd_return->link[j].buffer_line);
					}
					success = false;
					break;
				}
				memset(fd_return->link[i].buffer_line, 0, fd_return->size_buffer + SIZE_HEADER);
				success = true;
			}
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
				break;
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

	for (i = 0; i < FDUSART_BUFFER_LINES; i++)
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
		memcpy((void*) gInstances_Receiving[i].message_ready_for_read[index_free], (void*) gInstances_Receiving[i].buffer_received,
				gInstances_Receiving[i].size_message + SIZE_HEADER);
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

//	uint16_t sequence_number_sent = huart->pTxBuffPtr[4] << 8 | huart->pTxBuffPtr[5];
//	uint16_t sequence_number_index = 0;
//
//	for (j = 0; j < FDUSART_BUFFER_LINES; j++)
//	{
//		sequence_number_index = instance->link[j].buffer_line[4] << 8 | instance->link[j].buffer_line[5];
//		if (sequence_number_sent == sequence_number_index)
//		{
//			// Found line sent
//			break;
//		}
//	}
//
//	if (j >= FDUSART_BUFFER_LINES)
//	{
////All lines used
//		return;
//	}

	for (j = 0; j < FDUSART_BUFFER_LINES; j++)
	{
		if (instance->link[j].line_state == LINE_SENT_DMA)
		{
			//			Found line sent
			break;
		}
	}

	instance->flags.bits.send_message = false;

	if (instance->link[j].buffer_line[3] == MESSAGE_ACK)
	{ //If response with ACK no wait
		_FDUSART_ClearLine(instance, j);
	}
	else
	{
		instance->link[j].line_state = LINE_SENT_EXTRNAL;
	}

}

/*void USART2_IRQHandler(void)
 {
 //https://community.st.com/t5/stm32-mcus-embedded-software/hal-uart-transmit-dma-issues/td-p/443614/page/2
 size_t i;
 FD_t *instance;

 // Transmission
 for (i = 0; i < MAX_UART_INSTANCES; i++)
 {
 instance = (void*) gInstances[i];
 if (instance != NULL)
 {
 HAL_UART_IRQHandler((void*) gInstances[i]->link);
 }
 }

 }*/

size_t FDUSART_InterruptControl()
{
	size_t success = false;
	FD_t *instance;
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

				if (instance->link[line].line_state == LINE_SENT_EXTRNAL || instance->link[line].line_state == LINE_SENT_DMA)
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

	// Reception
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

bool FDUSART_SendMessage(FD_t *FDInstance, uint8_t Cmd, uint8_t *Buf, size_t Len)
{
	uint16_t line_free_id = 0;
	bool success = false;

	if (!FDInstance || !Buf) // Verifica ponteiros nulos
		return success;

	/* All lines occupied */
	if (!_FDUSART_SearchLineFree(FDInstance, &line_free_id))
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
	FDInstance->sequence_number = FDInstance->sequence_number >= UINT16_MAX ? 1 : FDInstance->sequence_number + 1;

	_FDUSART_ClearLine(FDInstance, line_free_id);

	// ID | VERSION | TYPE MESSAGE | SEQUENCE NUMBER | MESSAGE COMMAND | SIZE MESSAGE | MESSAGE | CRC
	FDInstance->link[line_free_id].buffer_line->fields.header = MESSAGE_ID_SEND;
	FDInstance->link[line_free_id].buffer_line->fields.version = PROTOCOL_VERSION;
	FDInstance->link[line_free_id].buffer_line->fields.type_message = MESSAGE_SEND;
	FDInstance->link[line_free_id].buffer_line->fields.sequence_number = FDInstance->sequence_number;
	FDInstance->link[line_free_id].buffer_line->fields.cmd = Cmd;
	FDInstance->link[line_free_id].buffer_line->fields.len = Len;

	/* offsetof(FD_MSG_t, fields.len) retorna o deslocamento do membro len dentro da estrutura.
	 sizeof(FDInstance->link[line_free_id].buffer_line->fields.len) retorna o tamanho do campo len, então o ponteiro se move para o byte após o len.
	 O resultado é o endereço inicial onde você quer copiar sua mensagem. */

	uint8_t *messageStart = (uint8_t*) FDInstance->link[line_free_id].buffer_line + offsetof(FD_MSG_t, fields.len)
			+ sizeof(FDInstance->link[line_free_id].buffer_line->fields.len);
	memcpy(messageStart, Buf, Len);

	uint16_t crc = CRC16_CCITT_Calculate(&(FDInstance->link[line_free_id].buffer_line->data), offsetof(FD_MSG_t, fields.crc16)); //Calculate CRC based into message and header
	FDInstance->link[line_free_id].buffer_line->fields.crc16 = crc;

	FDInstance->link[line_free_id].message_length = sizeof(FD_MSG_t) + Len;

	_FDUSART_MakeToSend(FDInstance, line_free_id);

	success = true;

	return success;
}

bool FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{
	bool success = false;

	if (!FDInstance || !Cmd || !Buf || !Len)
		return false;

	size_t search_instance = _FDUSART_SearchUARTInstance(FDInstance);

	if (search_instance >= MAX_UART_INSTANCES)
		return false;

	int16_t index_message_for_read = gInstances_Receiving[search_instance].cnt_messages_for_read - 1;
	while (index_message_for_read > -1)
	{
		FD_MSG_t *currentMessage = (FD_MSG_t*) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read];

		uint16_t sequence_number = 0;

		if (!_FDUSART_ExtractMessageDetails(currentMessage, &sequence_number, Cmd, Len, &sequence_number))
		{
			index_message_for_read--;
			continue;
		}

		if (!_FDUSART_HandleMessageOrACK(FDInstance, search_instance, &index_message_for_read, sequence_number))
		{
			index_message_for_read--;
			continue;
		}

		if (!_FDUSART_ValidateMessageCRC(currentMessage, *Len))
		{
			index_message_for_read--;
			continue;
		}

        uint8_t* messageStart = currentMessage->data + offsetof(FD_MSG_t, fields.crc16) + sizeof(currentMessage->fields.crc16);
        memcpy(Buf, messageStart, *Len);

        gInstances_Receiving[search_instance].cnt_messages_for_read--;

		return true;

	}
	return false;
}

bool FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{

	bool success = false;
	size_t search_instance;

	// Check null pointers and valid input
	if (!FDInstance || !Cmd || !Buf || !Len)
		return success;

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

	int16_t index_message_for_read = gInstances_Receiving[search_instance].cnt_messages_for_read - 1;
	while (index_message_for_read > -1)
	{ //If exists message for read

		uint8_t *currentMessage = gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read];

		// Extract message details: sequence number, command, and size.
		uint16_t sequence_number = (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][4] << 8)
				| gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][5];

		*Cmd = gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][6];
		*Len = (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][7] << 8)
				| gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][8];

		//If new message, send ACK
		if (gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][3] == MESSAGE_SEND)
		{

			//FDUSART_SendMessage((void*) gInstances[search_instance], *Cmd, (void*) &gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][9], *Len);
			_FDUSART_SendACK((void*) gInstances[search_instance], (void*) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read],
					*Len);

		}
		else
		{ // If the message is just an ACK, delete the message and subtract it from the message counter
			index_message_for_read = --gInstances_Receiving[search_instance].cnt_messages_for_read;
			int16_t index_sequence_number = _FDUSART_SearchIndexSequenceNumber((void*) &FDInstance, sequence_number);
			_FDUSART_ClearLine((void*) &FDInstance, index_sequence_number);
			continue;
		}

		// Validate CRC
		uint16_t received_crc = (uint16_t) ((int16_t) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][*Len + SIZE_HEADER
				- 2] << 8) | (int16_t) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read][*Len + SIZE_HEADER - 1];
		uint16_t calculated_crc = CRC16_CCITT_Calculate((void*) gInstances_Receiving[search_instance].message_ready_for_read[index_message_for_read],
				*Len + SIZE_HEADER - 2);

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

