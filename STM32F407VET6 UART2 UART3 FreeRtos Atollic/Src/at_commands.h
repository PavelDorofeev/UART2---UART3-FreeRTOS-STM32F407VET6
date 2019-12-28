#ifndef __AT_COMMANDS_H
#define __AT_COMMANDS_H

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
//#include "semphr.h"
//#include "cmsis_os.h"

HAL_StatusTypeDef MasterSendATCommand(
		UART_HandleTypeDef *huart,
		uint8_t *tx,
		uint16_t toTransfer,
		uint8_t *rx,
		uint16_t *wasReadden,
		xQueueHandle *rxUart,
		TickType_t xMaxExpectedBlockTime,
		uint8_t echoON);

HAL_StatusTypeDef MastertoUart(UART_HandleTypeDef *huart,
		uint8_t *pData,
		uint16_t toTransfer);

//uint8_t getAnswer(xQueueHandle *rxUart,uint16_t toTransfer,TickType_t xMaxExpectedBlockTime,uint8_t echoON);

uint8_t MasterGetFromQueue(
		xQueueHandle *rxQueue,
		uint8_t *tx,
		uint16_t toTransfer,
		uint8_t *rx,
		TickType_t xMaxExpectedBlockTime,
		uint8_t checkEcho);

uint8_t SlaveGetFromQueue(
		xQueueHandle *rxQueue,
		uint8_t *rx,
		TickType_t xMaxExpectedBlockTime);

HAL_StatusTypeDef SlaveToUart(
		UART_HandleTypeDef *phuart,
		uint8_t *tx,
		uint16_t toTransfer);


#endif
