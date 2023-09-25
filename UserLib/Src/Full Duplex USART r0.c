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

#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	500			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR						5			// Max error if not receive message
#define FDUSART_TX_TIMEOUT						10			// Timeout each byte sent
#define FDUSART_RX_TIMEOUT						100			// Timeout receive message complete

#define MESSAGE_ID_SEND							0xBEBE		// Protocol init message for send
#define PROTOCOL_VERSION						100			// Protocol version: 1.00
//#define SIZE_HEADER								11			// 2Bytes (ID) 	 + 1Byte (VERSION) + 1 Byte (TYPE MESSAGE) + 2Bytes (SEQUENCE NUMBER) + 1Byte (COMMAND MESSAGE) + 1Byte (SIZE MESSAGE) + 2Bytes (CRC)

#define DELAY_EACH_MESSAGE						10 			// Delay in each message

#define HEADER_OFFSET (offsetof(FD_MSG_t, fields.header))
#define HEADER_SIZE (sizeof(((FD_MSG_t *)0)->fields.header))

#define VERSION_OFFSET (offsetof(FD_MSG_t, fields.version))
#define VERSION_SIZE (sizeof(((FD_MSG_t *)0)->fields.version))

#define LEN_OFFSET (offsetof(FD_MSG_t, fields.len))
#define LEN_SIZE (sizeof(((FD_MSG_t *)0)->fields.len))

#define FIXED_MESSAGE_SIZE (offsetof(FD_MSG_t, fields.crc16) + sizeof(((FD_MSG_t *)0)->fields.crc16))

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

typedef struct PACK_STRUCT
{
	uint16_t header;
	uint8_t version;
	uint8_t type_message;
	uint16_t sequence_number;
	uint8_t cmd;
	uint16_t len;
	uint16_t crc16;
} FD_HEADER_t;

typedef struct
{
	volatile FD_HEADER_t fields;
	volatile uint8_t message[];
} FD_MSG_t;

typedef struct
{
	LINE_STATE_t line_state;
	volatile FD_MSG_t *buffer_line;
	uint16_t time_retransmit;
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
	uint8_t buffer_received[FDUSART_SIZE_MAX_MESSAGE + HEADER_SIZE];
	uint16_t cnt_byte_received;
	uint16_t size_message;
	uint16_t time_receiving;

	uint8_t message_ready_for_read[FDUSART_BUFFER_RECEIVE][FDUSART_SIZE_MAX_MESSAGE];
	int16_t cnt_messages_for_read;

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

uint16_t countDelayEachMessage = 0;

uint16_t countRetransmission = 0;
uint16_t countMessageError = 0;

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/

/********************************************************************************
 ******* PROTOTYPE PRIVATE FUNCTIONS
 *******************************************************************************/
static bool _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree);
static bool _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine);
static int16_t _FDUSART_SearchIndexSequenceNumber(FD_t *FDInstance, uint16_t SequenceNumber);
static void _FDUSART_MakeToSend(FD_t *FDInstance, uint16_t NumberLine);
static void _FDUSART_SetFlagFullBuf(FD_t *FDInstance, uint8_t flag_value);
static size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line);
static bool _FDUSART_CheckHeader(size_t index);
static bool _FDUSART_CheckProtocolVersion(size_t index);
static bool _FDUSART_CheckSizeMessage(size_t index);
static void _FDUSART_ResetReceiving(size_t index);
static bool _FDUSART_HandleMessageOrACK(FD_t *FDInstance, FD_MSG_t *FDMessage, size_t search_instance, int16_t index_message_for_read);
static size_t _FDUSART_SearchUARTInstance(FD_t *FDInstance);
static bool _FDUSART_ExtractMessageDetails(FD_MSG_t *message, uint8_t *cmd, size_t *len, size_t *sequence_number);
static bool _FDUSART_ValidateMessageCRC(FD_MSG_t *message, size_t len);
static uint16_t _FDUSART_CalcCRCProtocol(FD_MSG_t *message);
static void _FDUSART_CopyToBuf(void *buf, void *message, size_t size, size_t *offset);
static void _FDUSART_CopyFromBuf(void *dest, uint8_t *source, size_t size, size_t *offset);
static void _FDUSART_ConstructMessage(FD_MSG_t *FDMessage, uint8_t *buf, bool ignoreCRC);
static void _FDUSART_ConstructMessageReceived(uint8_t *source, FD_MSG_t *dest);

/********************************************************************************
 ******* PRIVATE FUNCTIONS
 *******************************************************************************/
static bool _FDUSART_SearchLineFree(FD_t *FDInstance, uint16_t *LineFree)
{
	bool success = false;
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

static bool _FDUSART_ClearLine(FD_t *FDInstance, uint16_t NumberLine)
{
	bool success = false;
	if (NumberLine < FDUSART_BUFFER_LINES)
	{
		FDInstance->link[NumberLine].error_timeout_cnt = 0;
		FDInstance->link[NumberLine].line_state = LINE_FREE;
		FDInstance->link[NumberLine].time_retransmit = 0;
		success = true;
	}
	return success;
}

static int16_t _FDUSART_SearchIndexSequenceNumber(FD_t *FDInstance, uint16_t SequenceNumber)
{
	int16_t index_with_sequence_number = -1;

	for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
	{
		if (FDInstance->link[i].buffer_line->fields.sequence_number == SequenceNumber)
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

static void _FDUSART_CopyToBuf(void *buf, void *message, size_t size, size_t *offset)
{
	memcpy(buf + *offset, message, size);
	*offset += size;

	/*if (size == 1) {
	 buf[*offset] = *(uint8_t *)message;
	 *offset += size;
	 return;
	 }

	 // Big-Endian
	 uint8_t *source = (uint8_t *)message + size - 1;
	 uint8_t *destination = buf + *offset;

	 for(size_t i = 0; i < size; i++) {
	 *destination = *source;
	 destination++;
	 source--;
	 }

	 *offset += size;*/
}
static void _FDUSART_CopyFromBuf(void *dest, uint8_t *source, size_t size, size_t *offset)
{
	memcpy(dest, source + *offset, size);
	*offset += size;
}

static void _FDUSART_ConstructMessage(FD_MSG_t *FDMessage, uint8_t *buf, bool ignoreCRC)
{
	size_t offset = 0;

	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.header), sizeof(FDMessage->fields.header), &offset);
	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.version), sizeof(FDMessage->fields.version), &offset);
	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.type_message), sizeof(FDMessage->fields.type_message), &offset);
	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.sequence_number), sizeof(FDMessage->fields.sequence_number), &offset);
	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.cmd), sizeof(FDMessage->fields.cmd), &offset);
	_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.len), sizeof(FDMessage->fields.len), &offset);

	memcpy(buf + offset, (FD_MSG_t*)FDMessage->message, FDMessage->fields.len);
	offset += FDMessage->fields.len;
	if (!ignoreCRC)
		_FDUSART_CopyToBuf(buf, (FD_MSG_t*)&(FDMessage->fields.crc16), sizeof(FDMessage->fields.crc16), &offset);
}

static void _FDUSART_ConstructMessageReceived(uint8_t *source, FD_MSG_t *dest)
{
	size_t offset = 0;

	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.header), source, sizeof(dest->fields.header), &offset);
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.version), source, sizeof(dest->fields.version), &offset);
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.type_message), source, sizeof(dest->fields.type_message), &offset);
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.sequence_number), source, sizeof(dest->fields.sequence_number), &offset);
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.cmd), source, sizeof(dest->fields.cmd), &offset);
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.len), source, sizeof(dest->fields.len), &offset);
	if (dest->fields.len > FDUSART_SIZE_MAX_MESSAGE)
	{
		return;
	}
	memcpy((FD_MSG_t*)&(dest->message), source + offset, dest->fields.len);
	offset += dest->fields.len;
	_FDUSART_CopyFromBuf((FD_MSG_t*)&(dest->fields.crc16), source, sizeof(dest->fields.crc16), &offset);

}

#pragma GCC push_options
#pragma GCC optimize ("-O0")
static size_t _FDUSART_TransmitMessage(FD_t *FDInstance, size_t line)
{
	size_t success = false;

	// Crie um buffer temporário
	volatile uint8_t buffer[sizeof(FD_MSG_t) + FDInstance->link[line].buffer_line->fields.len];

	/* Critical section */
	//__disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/

	_FDUSART_ConstructMessage((void*)FDInstance->link[line].buffer_line, (uint8_t*)buffer, false);

	FDInstance->flags.bits.send_message = true;
	FDInstance->link[line].line_state = LINE_SENT_DMA;

	success = true;
	//memcpy(&teste[countsend++], buffer, sizeof(buffer));

	// Agora, envie o buffer usando HAL_UART_Transmit_DMA

	HAL_UART_Transmit_DMA(FDInstance->uart, (uint8_t *)buffer, FDInstance->link[line].message_length);

	/* Critical section */
	//__enable_irq();

	return success;
}
#pragma GCC pop_options

static bool _FDUSART_ShouldCheckHeader(size_t index)
{
	return !gInstances_Receiving[index].Flags.bits.header_ok && gInstances_Receiving[index].cnt_byte_received >= (HEADER_OFFSET + HEADER_SIZE);
}

static bool _FDUSART_ShouldCheckProtocolVersion(size_t index)
{
	return !gInstances_Receiving[index].Flags.bits.version_protocol_ok && gInstances_Receiving[index].cnt_byte_received >= (VERSION_OFFSET + VERSION_SIZE);
}

static bool _FDUSART_ShouldCheckSizeMessage(size_t index)
{
	return !gInstances_Receiving[index].Flags.bits.size_buffer_ok && gInstances_Receiving[index].cnt_byte_received >= (LEN_OFFSET + LEN_SIZE);
}

static bool _FDUSART_CheckHeader(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[HEADER_OFFSET + 1] == (uint8_t) ((MESSAGE_ID_SEND & 0xFF00) >> 8)
			&& gInstances_Receiving[index].buffer_received[HEADER_OFFSET] == (uint8_t) (MESSAGE_ID_SEND & 0x00FF))
	{
		return true;
	}
	return false;
}

static bool _FDUSART_CheckProtocolVersion(size_t index)
{
	if (gInstances_Receiving[index].buffer_received[VERSION_OFFSET] == PROTOCOL_VERSION)
	{
		return true;
	}
	return false;
}

static bool _FDUSART_CheckSizeMessage(size_t index)
{
	gInstances_Receiving[index].size_message = ((uint16_t) (gInstances_Receiving[index].buffer_received[LEN_OFFSET + 1] << 8) | gInstances_Receiving[index].buffer_received[LEN_OFFSET]);
	if (gInstances_Receiving[index].size_message <= (gInstances[index]->size_buffer - FIXED_MESSAGE_SIZE))
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

static bool _FDUSART_SendACK(FD_t *FDInstance, uint8_t *Buf, size_t Len)
{
	uint16_t line_free_id = 0;
	bool success = false;

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

	size_t header_offset = offsetof(FD_MSG_t, fields.header);
	size_t data_size = offsetof(FD_MSG_t, fields.crc16) - offsetof(FD_MSG_t, fields.header) + Len;

	memcpy(&FDInstance->link[line_free_id].buffer_line, Buf + header_offset, data_size);

	FDInstance->link[line_free_id].buffer_line->fields.type_message = (uint8_t) MESSAGE_ACK;

	uint16_t crc = _FDUSART_CalcCRCProtocol((void*)FDInstance->link[line_free_id].buffer_line); //Calculate CRC based into message and header

	FDInstance->link[line_free_id].buffer_line->fields.crc16 = crc;

	FDInstance->link[line_free_id].message_length = sizeof(((FD_MSG_t*) 0)->fields) + Len;

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

static bool _FDUSART_HandleMessageOrACK(FD_t *FDInstance, FD_MSG_t *FDMessage, size_t search_instance, int16_t index_message_for_read)
{
	FD_MSG_t *currentMessageStruct = FDMessage;

	if (currentMessageStruct->fields.type_message == MESSAGE_SEND)
	{
		_FDUSART_SendACK((void*) gInstances[search_instance], (uint8_t*) currentMessageStruct, currentMessageStruct->fields.len);
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

	//size_t totalSize = len + offsetof(FD_MSG_t, fields.crc16);

	uint16_t calculated_crc = _FDUSART_CalcCRCProtocol(message); //Calculate CRC based into message and header

	return received_crc == calculated_crc;
}

static uint16_t _FDUSART_CalcCRCProtocol(FD_MSG_t *message)
{
	// Calculando o tamanho dos campos até o campo 'crc16' (excluindo o próprio campo 'crc16')
	size_t headerSize = offsetof(FD_MSG_t, fields.crc16);

	// O tamanho total que você quer calcular o CRC é o tamanho do cabeçalho mais a quantidade de dados em 'message'
	size_t totalSize = headerSize + message->fields.len; // Supondo que 'len' indique a quantidade de bytes em 'message'

	uint8_t data_for_calc_crc[totalSize];

	// Se quiser zerar o buffer (embora possa não ser necessário se você estiver preenchendo todo o buffer posteriormente)
	memset(data_for_calc_crc, 0, totalSize);

	memcpy(data_for_calc_crc, (uint8_t*) message, headerSize);
	memcpy(data_for_calc_crc + headerSize,(FD_MSG_t*) message->message, message->fields.len); // Note que mudei "Len" para "message->fields.len"

	uint16_t crc = CRC16_CCITT_Calculate(data_for_calc_crc, totalSize);
	return crc;
}

/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

FD_t* FDUSART_Init(const FD_CONFIG_t *config)
{
	assert(config != NULL);

	bool success = false;

	FD_t *fd_return;

	if (config->size_buffer > FDUSART_SIZE_MAX_MESSAGE)
	{
		return NULL;
	}

	fd_return = (FD_t*) malloc(sizeof(FD_t));

	if (fd_return != NULL)
	{
		memset(fd_return, 0, sizeof(FD_t));  // Limpa a memória alocada.

		fd_return->flags.all_flags = 0;
		fd_return->flags.bits.initialized = true;
		fd_return->uart = config->uart;

		fd_return->size_buffer = config->size_buffer;
		// Allocated dinamic memeory
		for (size_t i = 0; i < FDUSART_BUFFER_LINES; i++)
		{
			fd_return->link[i].buffer_line = (FD_MSG_t*) malloc(sizeof(FD_MSG_t) + fd_return->size_buffer * sizeof(uint8_t));

			if (fd_return->link[i].buffer_line == NULL)
			{
				// Free previously allocated memory
				for (size_t j = 0; j < i; j++)
				{
					free((void*)fd_return->link[j].buffer_line);
				}
				success = false;
				break;
			}
			memset((void*)fd_return->link[i].buffer_line, 0, sizeof(FD_MSG_t) + fd_return->size_buffer * sizeof(uint8_t));

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
				gInstances_Receiving[i].cnt_messages_for_read = 0;

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
			free((void*)(*fd)->link[i].buffer_line);
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
	if (_FDUSART_ShouldCheckHeader(i))
	{
		if (!_FDUSART_CheckHeader(i))
		{
			_FDUSART_ResetReceiving(i);
			return;
		}
		gInstances_Receiving[i].Flags.bits.header_ok = true;
	}

	/* Check version protocol */
	if (_FDUSART_ShouldCheckProtocolVersion(i))
	{
		if (!_FDUSART_CheckProtocolVersion(i))
		{
			_FDUSART_ResetReceiving(i);
			return;
		}
		gInstances_Receiving[i].Flags.bits.version_protocol_ok = true;
	}

	/* Check size message */
	if (_FDUSART_ShouldCheckSizeMessage(i))
	{
		if (!_FDUSART_CheckSizeMessage(i))
		{
			_FDUSART_ResetReceiving(i);
			return;
		}
		gInstances_Receiving[i].Flags.bits.size_buffer_ok = true;
	}

	if (gInstances_Receiving[i].cnt_byte_received >= gInstances_Receiving[i].size_message + FIXED_MESSAGE_SIZE)
	{
		uint16_t index_free = gInstances_Receiving[i].cnt_messages_for_read++;
		if (index_free > FDUSART_BUFFER_RECEIVE)
		{
			// Buffer cheio
			gInstances_Receiving[i].cnt_messages_for_read = 0;
			index_free = 0;
		}
		memcpy((void*) gInstances_Receiving[i].message_ready_for_read[index_free], (void*) gInstances_Receiving[i].buffer_received, gInstances_Receiving[i].size_message + FIXED_MESSAGE_SIZE);
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
	if (j < FDUSART_BUFFER_LINES)
	{

		if (instance->link[j].buffer_line->fields.type_message == MESSAGE_ACK)
		{ //If response with ACK no wait
			_FDUSART_ClearLine(instance, j);
		}
		else
		{
			instance->link[j].line_state = LINE_SENT_EXTRNAL;
		}
	}

	instance->flags.bits.send_message = false;
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

 }
 */

bool FDUSART_InterruptControl()
{
	bool success = false;
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
					if (countDelayEachMessage >= DELAY_EACH_MESSAGE)
					{
						countDelayEachMessage = 0;
						/* Search line ready for send */
						if (instance->link[line].line_state == LINE_READY_FOR_SEND)
						{
							/* Activated DMA for transmit message */
							success = _FDUSART_TransmitMessage(instance, line);
						}
					}
					else
						countDelayEachMessage++;

				}

				if (instance->link[line].line_state == LINE_SENT_EXTRNAL || instance->link[line].line_state == LINE_SENT_DMA)
				{ // If message sent and not received response
					instance->link[line].time_retransmit--; //Decrement time for retransmit
					if (instance->link[line].time_retransmit == 0)
					{
						countRetransmission++;
						instance->link[line].error_timeout_cnt++; //Increment number of tentative
						if (instance->link[line].error_timeout_cnt >= FDUSART_MAX_ERROR) //If exceded tentatives
						{
							countMessageError++;
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

	uint8_t *messageStart = (void*)&(FDInstance->link[line_free_id].buffer_line->message[0]);
	memcpy(messageStart, Buf, Len);

	// Crie um buffer temporário
	uint8_t buffer[sizeof(FD_MSG_t) - sizeof(((FD_MSG_t*) 0)->fields.crc16) + Len];

	_FDUSART_ConstructMessage((void*)FDInstance->link[line_free_id].buffer_line, buffer, false);

	uint16_t crc = CRC16_CCITT_Calculate(buffer, Len + sizeof(FD_HEADER_t) - sizeof(((FD_MSG_t*) 0)->fields.crc16));

	FDInstance->link[line_free_id].buffer_line->fields.crc16 = crc;

	FDInstance->link[line_free_id].message_length = sizeof(FD_MSG_t) + Len;

	_FDUSART_MakeToSend(FDInstance, line_free_id);

	success = true;

	return success;
}

bool FDUSART_Receive_Message(FD_t *FDInstance, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{
	// Check null pointers and valid input
	if (!FDInstance || !Cmd || !Buf || !Len)
		return false;

	size_t search_instance = _FDUSART_SearchUARTInstance(FDInstance);

	if (search_instance >= MAX_UART_INSTANCES)
		return false;

	int16_t *index_message_for_read = (void*) &gInstances_Receiving[search_instance].cnt_messages_for_read;
	while (*index_message_for_read > 0)
	{
		FD_MSG_t *currentMessage = malloc(sizeof(FD_MSG_t));
		memset(currentMessage, 0, sizeof(FD_MSG_t));

		_FDUSART_ConstructMessageReceived((uint8_t*) gInstances_Receiving[search_instance].message_ready_for_read[*index_message_for_read - 1], currentMessage);

		size_t sequence_number = 0;

		if (!_FDUSART_ExtractMessageDetails(currentMessage, Cmd, Len, &sequence_number))
		{
			(*index_message_for_read)--;
			continue;
		}

		if (!_FDUSART_ValidateMessageCRC(currentMessage, *Len))
		{
			(*index_message_for_read)--;
			continue;
		}

		if (!_FDUSART_HandleMessageOrACK(FDInstance, currentMessage, search_instance, *index_message_for_read))
		{
			(*index_message_for_read)--;
			continue;
		}
		memcpy(Buf, (FD_MSG_t*)&(currentMessage->message[0]), *Len);

		return true;

	}
	return false;
}

