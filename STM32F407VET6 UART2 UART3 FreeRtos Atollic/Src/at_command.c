#include "at_commands.h"

//extern uint8_t  rx1[200];

HAL_StatusTypeDef MasterSendATCommand(
		UART_HandleTypeDef *huart,
		uint8_t *tx,
		uint16_t toTransfer,
		uint8_t *rx,
		uint16_t *wasReadden,
		xQueueHandle *rxQueue,
		TickType_t xMaxExpectedBlockTime,
		uint8_t echoON)
{
	HAL_StatusTypeDef res;

	printf("Master ->");
	for(int ii=0; ii< toTransfer
	; ii++)
	{
		printf("%.2X ",tx[ii]);
	}
	printf("\n");


	//tx[0]=0x41; // A
	//tx[1]=0x54; // T

	if (MastertoUart(huart, (uint8_t *)tx, toTransfer)!= HAL_OK)
		return 0;

	/*if(echoON==1)
	{
		if(MasterGetFromQueue(rxQueue, tx, toTransfer, rx, xMaxExpectedBlockTime, 1) != 1)
		{
			printf("error answer from slave\n");
			return 0;
		}
		tx[0]="\n";
		MastertoUart(huart, (uint8_t *)tx, 1);

		if(MasterGetFromQueue(rxQueue, tx, toTransfer=1, rx, xMaxExpectedBlockTime,1) != 1)
		{
			printf("error answer from slave 2\n");
			return 0;
		}
	}

	if(MasterGetFromQueue(rxQueue, tx, toTransfer=0, rx, xMaxExpectedBlockTime,1) != 1)
	{
		printf("error answer from AT\n");
		return 0;
	}*/

	return res;
}



uint8_t MasterGetFromQueue(
		xQueueHandle *rxQueue,
		uint8_t *tx,
		uint16_t toTransfer,
		uint8_t *rx,
		TickType_t xMaxExpectedBlockTime,
		uint8_t checkEcho)
{
	BaseType_t pdRes;
	uint8_t rr=0;
	uint8_t transferRes=1;
	uint8_t rxCount=0;

	//printf("<- ");

	if( pdRes=xQueueReceive(rxQueue , rx , xMaxExpectedBlockTime ) == pdTRUE) //portMAX_DELAY
	{
		//printf("uxQueueMessagesWaiting=%d\n",uxQueueMessagesWaiting(rxUart));
		//rx[rxCount++] = rr;

		rxCount++;

		if(toTransfer == 0) // ждем неопределенное число байтов
			return rxCount;

		//cразу проверяем эхо
		/*if(checkEcho == 1 && tx[rxCount-1] != rr) // команда эхом не вернулась
		{
			printf("wrong nn=%d\n",rxCount-1);
			transferRes=-1;
			return 0;
		}*/

		if(rxCount == toTransfer)
		{
			//transferRes=1;
			return rxCount;
		}

	}

	if(pdRes != errQUEUE_EMPTY)
	{
		printf(" !=  errQUEUE_EMPTY\n");
		return 0;
	}
	/*else if (transferRes != 1)
	{
		printf("wrong bytes \n");
		return 0;
	}*/


	//printf("rx1 rxCount=%d\n",rxCount);

	//printf("\n");

	return 0;

}


HAL_StatusTypeDef MastertoUart(
		UART_HandleTypeDef *phuart,
		uint8_t *tx,
		uint16_t toTransfer)
{
	HAL_StatusTypeDef res;

	if (res=HAL_UART_Transmit_IT(phuart, (uint8_t *)tx, toTransfer) == HAL_OK)
		;//printf("Master > HAL_OK\n");
	else if(res = HAL_ERROR)
	{
		printf("Master > HAL_ERROR\n");
		return res;
	}
	else  if(res = HAL_BUSY)
	{
		printf("Master > HAL_BUSY\n");
		return res;
	}
	return res;
}


uint8_t SlaveGetFromQueue(
		xQueueHandle *rxQueue,
		uint8_t *rx,
		TickType_t xMaxExpectedBlockTime)
{
	BaseType_t pdRes;
	uint8_t rr=0;
	uint8_t transferRes=1;
	uint8_t rxCount=0;
	//printf("Slave xQueueReceive\n");
	while( pdRes=xQueueReceive(rxQueue , &rr , xMaxExpectedBlockTime ) == pdTRUE) //portMAX_DELAY
	{
		rx[rxCount++] = rr;
		//printf("Slave xQueueReceive %d\n",rr);
	}
	if(pdRes != errQUEUE_EMPTY)
	{
		printf("Slave !=  errQUEUE_EMPTY\n");
		return 0;
	}
	else if (transferRes != 1)
	{
		printf("Slave wrong bytes \n");
		return 0;
	}

	//printf("Slave rxCount %d\n",rxCount);
	return rxCount;

}


HAL_StatusTypeDef SlaveToUart(
		UART_HandleTypeDef *phuart,
		uint8_t *tx,
		uint16_t toTransfer)
{
	HAL_StatusTypeDef res;

	if (res=HAL_UART_Transmit_IT(phuart, (uint8_t *)tx, toTransfer) == HAL_OK)
		;//printf("Slave > HAL_OK\n");
	else if(res = HAL_ERROR)
	{
		printf("Slave < HAL_ERROR\n");
		return res;
	}
	else  if(res = HAL_BUSY)
	{
		printf("Slave < HAL_BUSY\n");
		return res;
	}
	return res;
}
