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

#include "stm32g4xx_hal_uart.h"

#include "Full Duplex USART r0.h"
#include "CRC16 CCITT.h"

/********************************************************************************
 ******* DEFINES
 *******************************************************************************/
#define FDUSART_BUFFER_LINES					15 			// Size array buffer
#define FDUSART_SIZE_MAX_MESSAGE				100			// Size in one line of the buffer
#define FDUSART_TIME_RETRANSMIT_IF_NORESPONSE	25			// If not receive response of message sent, retransmit message and increment in error
#define FDUSART_MAX_ERROR									// Max error if not receive message
#define FDUSART_TX_TIMEOUT						10			// Timeout each byte sent

#define MESSAGE_ID_SEND							0xBEBE		// Protocol init message for send
#define MESSAGE_ID_RECEIVED						0xCECE		// Protocol init message for received
#define PROTOCOL_VERSION						100			// Protocol version: 1.00
#define SIZE_HEADER								10			// 2Bytes (ID) 	 + 1Byte (VERSION) + 2Bytes (SEQUENCE NUMBER) + 1Byte (COMMAND MESSAGE) + 1Byte (SIZE MESSAGE) + 2Bytes (CRC)

typedef enum
{
	LINE_FREE = 0, LINE_READY_FOR_SEND, LINE_EXCEDED_TIME_SENT,
} LINE_STATE_t;

/********************************************************************************
 ******* VARIABLES
 *******************************************************************************/
typedef union
{
	struct
	{
		unsigned Initialized :1;
		unsigned ChecksumOK :1;
		unsigned FullBuf :1;
		unsigned dummy :13;
	} Bits;
	uint16_t AllFlags;
} FDUFlags_t;

struct
{
	LINE_STATE_t lineState;
	uint8_t bufferLine[FDUSART_SIZE_MAX_MESSAGE + SIZE_HEADER];
	uint8_t timeRetransmit;
	uint8_t errorTimoutCnt;
} FDUManager[FDUSART_BUFFER_LINES];

struct
{
	uint16_t FDUSequenceNumber;
	FDUFlags_t FDUFlags;
} FDUSART_System;
UART_HandleTypeDef *huart;

/********************************************************************************
 ******* PRIVATE FUNCTIONS
 *******************************************************************************/
size_t _FDUSART_SearchLineFree(uint16_t *LineFree)
{
	size_t success = false;
	uint16_t line_free_id = 0;
	for (line_free_id = 0; line_free_id < FDUSART_BUFFER_LINES; line_free_id++)
	{
		if (FDUManager[line_free_id].bufferLine[0] == 0 && FDUManager[line_free_id].bufferLine[1] == 0) // Free line
		{
			*LineFree = line_free_id;
			success = true;
			break;
		}
	}
	return success;
}


size_t _FDUSART_ClearLine(uint16_t NumberLine)
{
	size_t success = false;
	FDUManager[NumberLine].bufferLine[0] = 0;
	FDUManager[NumberLine].bufferLine[1] = 0;
	FDUManager[NumberLine].errorTimoutCnt = 0;
	FDUManager[NumberLine].lineState = LINE_FREE;
	FDUManager[NumberLine].timeRetransmit = 0;
	success = true;
	return success;
}
/********************************************************************************
 ******* FUNCTIONS
 *******************************************************************************/

size_t FDUSART_Init(UART_HandleTypeDef *_huart)
{
	size_t success = false;
	huart = _huart;
	FDUSART_System.FDUFlags.AllFlags = 0;

	FDUSART_System.FDUFlags.Bits.Initialized = true;

	success = true;

	return success;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//Se receber completamente os dados
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

size_t FDUSART_SendMessage(UART_HandleTypeDef *huart, uint8_t Cmd, uint8_t *Buf, size_t Len)
{
	uint16_t line_free_id = 0;
	size_t success = false;

	/* All lines occupied */
	if(_FDUSART_SearchLineFree(&line_free_id))
	{
		FDUSART_System.FDUFlags.Bits.FullBuf = 1;
		return success;
	}

	/* Len outside the maximum size */
	if (Len > FDUSART_SIZE_MAX_MESSAGE)
	{
		return success;
	}

	/* Sequence number value */
	if (FDUSART_System.FDUSequenceNumber == UINT16_MAX)
	{
		FDUSART_System.FDUSequenceNumber = 0;
	}
	else
	{
		FDUSART_System.FDUSequenceNumber++;
	}

	_FDUSART_ClearLine(line_free_id);

	//* ID | VERSION | SEQUENCE NUMBER | MESSAGE COMMAND | SIZE MESSAGE | MESSAGE | CRC
	FDUManager[line_free_id].bufferLine[0] = (uint8_t) (MESSAGE_ID_SEND & 0xFF00) >> 8; // Lines writted with MESSAGE_ID
	FDUManager[line_free_id].bufferLine[1] = (uint8_t) (MESSAGE_ID_SEND & 0x00FF);
	FDUManager[line_free_id].bufferLine[2] = PROTOCOL_VERSION;
	FDUManager[line_free_id].bufferLine[3] = (FDUSART_System.FDUSequenceNumber & 0xFF00) >> 8;
	FDUManager[line_free_id].bufferLine[4] = (FDUSART_System.FDUSequenceNumber & 0x00FF);
	FDUManager[line_free_id].bufferLine[5] = Cmd;
	FDUManager[line_free_id].bufferLine[6] = (uint8_t) (Len & 0xFF00) >> 8;
	FDUManager[line_free_id].bufferLine[7] = (uint8_t) (Len & 0x00FF);
	memcpy(&FDUManager[line_free_id].bufferLine[8], Buf, Len);

	uint16_t crc = CRC16_CCITT_Calculate(FDUManager[line_free_id].bufferLine, FDUSART_SIZE_MAX_MESSAGE + SIZE_HEADER); //Calculate CRC based into message and header

	FDUManager[line_free_id].bufferLine[Len + 8] = (uint8_t) (crc & 0xFF00) >> 8;
	FDUManager[line_free_id].bufferLine[Len + 9] = (uint8_t) (crc & 0x00FF);

	FDUManager[line_free_id].lineState = LINE_READY_FOR_SEND;

	success = true;

	return success;
}

size_t FDUSART_Receive_Message(UART_HandleTypeDef *huart, uint8_t *Cmd, uint8_t *Buf, size_t *Len)
{
	uint16_t line_received_id = 0;

	size_t success = false;

	if (FDUManager[line_received_id].bufferLine[0] != (uint8_t) (MESSAGE_ID_RECEIVED & 0xFF00) >> 8
			|| FDUManager[line_received_id].bufferLine[1] != (uint8_t) (MESSAGE_ID_RECEIVED & 0x00FF))
	{
		// invalid ID
		return success;
	}

	// Validate protocol version
	if (FDUManager[line_received_id].bufferLine[2] != PROTOCOL_VERSION)
	{
		// protocol version invalid
		return success;
	}

	// Read sequence number, command and size message.
	uint16_t sequence_number = (FDUManager[line_received_id].bufferLine[3] << 8) | FDUManager[line_received_id].bufferLine[4];

	*Cmd = FDUManager[line_received_id].bufferLine[5];
	*Len = (FDUManager[line_received_id].bufferLine[6] << 8) | FDUManager[line_received_id].bufferLine[7];

	// Validate size message
	if (*Len > FDUSART_SIZE_MAX_MESSAGE)
	{
		// size message invalid
		return success;
	}

	// check CRC
	uint16_t received_crc = (FDUManager[line_received_id].bufferLine[*Len + 8] << 8) | FDUManager[line_received_id].bufferLine[*Len + 9];
	uint16_t calculated_crc = CRC16_CCITT_Calculate(FDUManager[line_received_id].bufferLine, *Len + SIZE_HEADER);

	if (received_crc != calculated_crc)
	{
		// invalid CRC
		return success;
	}

	// Copy message for pointer
	memcpy(Buf, &FDUManager[line_received_id].bufferLine[8], *Len);

	// Clear line
	success = _FDUSART_ClearLine(line_received_id);

	return success;
}

