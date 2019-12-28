/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "task.h"
#include "semphr.h"

#include "at_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId_t defaultTaskHandle;
osThreadId_t myTask02Handle;
/* USER CODE BEGIN PV */

UART_HandleTypeDef *phuartMaster;
UART_HandleTypeDef *phuartSlave;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument); // for v2
void StartTask02(void *argument); // for v2

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t txMaster[200];
volatile uint8_t rxMasterISR[200];
volatile uint8_t rx1Master[200];
volatile uint8_t MasterToTransfer=0;
volatile uint16_t wasReaddenMaster=0;
volatile uint8_t echoON=1;

volatile uint8_t txSlave[200];
volatile uint8_t rxSlaveISR[200];
volatile uint8_t SlaveToTransfer=0;
volatile uint8_t rx1Slave[200];

//volatile uint8_t tx1[200];

//volatile uint8_t txCmpl=0;
//volatile uint8_t rxCmpl=0;

xQueueHandle rxQueueMaster;
xQueueHandle rxQueueSlave;

volatile uint16_t toRead=1;

/* The handle of the binary semaphore. */

uint8_t* stateUART(HAL_UART_StateTypeDef State);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	phuartMaster=(UART_HandleTypeDef *)&huart2;
	phuartSlave=(UART_HandleTypeDef *)&huart3;

	printf(" start\n");
	/* USER CODE END 2 */

	osKernelInitialize(); // Initialize CMSIS-RTOS

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */

	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	//txSem = xSemaphoreCreateBinary();
	//rxSem = xSemaphoreCreateBinary();
	printf("xSemaphoreCreateBinary \n");

	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	//strout_Queue = osMailCreate(osMailQ(stroutqueue), NULL);
	//osMessageQDef(usart_Queue, QUEUE_SIZE, uint8_t);
	//USART_Queue = osMessageCreate(osMessageQ(usart_Queue), NULL);
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	const osThreadAttr_t defaultTask_attributes = {
			.name = "defaultTask",
			.priority = (osPriority_t) osPriorityNormal,
			.stack_size = 512
	};
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* definition and creation of myTask02 */
	const osThreadAttr_t myTask02_attributes = {
			.name = "myTask02",
			.priority = (osPriority_t) osPriorityNormal1,
			.stack_size = 512
	};
	myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : KEY2_Pin */
	GPIO_InitStruct.Pin = KEY2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t TxXferCount=huart->TxXferCount;
	uint16_t TxXferSize=huart->TxXferSize;

	if (huart-> Instance == USART2)
	{
		if(TxXferSize == 0 ) // какая-то ошибка
		{
			//printf(". HAL_UART_TxCpltCallback count=%d  size=%d \n",count,size);
			HAL_UART_Transmit_IT(huart, (uint8_t *)(txMaster),MasterToTransfer);
		}
		else if (TxXferSize > 0 & TxXferCount < TxXferSize)
		{
			if(TxXferCount>0)
				HAL_UART_Transmit_IT(huart, (uint8_t *)(txMaster+TxXferSize-TxXferCount),TxXferCount);
			/*else
				txCmpl=1;*/
		}
	}
	else if (huart-> Instance == USART3) // SLAVE
	{
		if(TxXferSize == 0 ) // какая-то ошибка
		{
			//printf(". HAL_UART_TxCpltCallback count=%d  size=%d \n",count,size);
			HAL_UART_Transmit_IT(huart, (uint8_t *)(txSlave),SlaveToTransfer);

		}
		else if (TxXferSize > 0 & TxXferCount < TxXferSize)
		{
			if(TxXferCount>0)
				HAL_UART_Transmit_IT(huart, (uint8_t *)(txSlave+TxXferSize-TxXferCount),TxXferCount);
			/*else
				txCmpl=1;*/

			//printf("USART2 HAL_UART_TxCpltCallback TxXferCount=%d  TxXferSize=%d \n",count,size);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// ВАЖНО , ЧТОБЫ HAL_UART_RxCpltCallback завершилась до прихода следующего байта !!!
	// точнее пока следующий байт приходит , у нас еще есть время, а вот следующий за следующим уже начнет затирать перед ним
	// По-этому все здесь делаем быстро !
	// Иначе ...


	if (huart-> Instance == USART2)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;
		//printf("HAL_UART_RxCpltCallback !");

		if(RxXferSize == 0) // похоже на сбой
		{
			printf("HAL_UART_RxCpltCallback Master????");
		}
		else if(RxXferSize > 0 && RxXferCount < RxXferSize)
		{
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR.

			for(int ii=0; ii<  (RxXferSize-RxXferCount); ii++)
			{
				xQueueSendToBackFromISR(rxQueueMaster, &rxMasterISR[ii], &xHigherPriorityTaskWoken );
			}

			/*we can switch context if necessary. */
			if( xHigherPriorityTaskWoken )
			{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //для ARM7
			}
		}
		HAL_UART_Receive_IT(huart, (uint8_t *)(rxMasterISR),1);
	}
	else if (huart-> Instance == USART3)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		if(RxXferSize == 0) // похоже на сбой
		{
			printf("HAL_UART_RxCpltCallback Slave????");
		}
		else if(RxXferSize > 0 && RxXferCount < RxXferSize)
		{
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR.

			for(int ii=0; ii<  (RxXferSize-RxXferCount); ii++)
			{
				xQueueSendToBackFromISR(rxQueueSlave, &rxSlaveISR[ii], &xHigherPriorityTaskWoken );
			}

			/*we can switch context if necessary. */
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				//taskYIELD_FROM_ISR ();
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //для ARM7
				//taskYIELD() // для AVR
			}
			//printf("HAL_UART_RxCpltCallback Slave\n");
		}
		HAL_UART_Receive_IT(huart, (uint8_t *)(rxSlaveISR),1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == USART2)
	{
		uint16_t TxXferCount=huart->TxXferCount;
		uint16_t TxXferSize=huart->TxXferSize;
		printf("> HAL_UART_ErrorCallback USART2\n");
		printf("> TxXferCount=%d TxXferSize=%d\n",TxXferCount,TxXferSize);
	}
	else if (huart-> Instance == USART3)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		printf("< HAL_UART_ErrorCallback USART3\n");
		printf("< RxXferCount=%d RxXferSize=%d\n",RxXferCount,RxXferSize);

		HAL_UART_Receive_IT(phuartSlave, (uint8_t *)(rxSlaveISR),1);
		// проблема в приемном буфере, ПЕРЕПОЛНЕНИЕ, не все вытащили ДО прихода следующей пачки данных
	}

}
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	/*if (huart-> Instance == USART2)
	{*/
	printf("HAL_UART_AbortReceiveCpltCallback \n");
	//}
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
	/*if (huart-> Instance == USART2)
	{*/
	printf("HAL_UART_AbortTransmitCpltCallback \n");
	//}
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
	/*if (huart-> Instance == USART2)
	{*/
	printf("HAL_UART_AbortCpltCallback \n");
	//}
}

uint8_t* stateUART(HAL_UART_StateTypeDef State)
{
	switch(State)
	{
	case HAL_UART_STATE_RESET: 		return "HAL_UART_STATE_RESET";
	case HAL_UART_STATE_READY: 		return "HAL_UART_STATE_READY";
	case HAL_UART_STATE_BUSY: 		return "HAL_UART_STATE_BUSY";
	case HAL_UART_STATE_BUSY_TX: 	return "HAL_UART_STATE_BUSY_TX";
	case HAL_UART_STATE_BUSY_RX: 	return "HAL_UART_STATE_BUSY_RX";
	case HAL_UART_STATE_BUSY_TX_RX: return "HAL_UART_STATE_BUSY_TX_RX";
	case HAL_UART_STATE_TIMEOUT: 	return "HAL_UART_STATE_TIMEOUT";
	case HAL_UART_STATE_ERROR: 		return "HAL_UART_STATE_ERROR";
	default : 						return "?????";
	}
	return "???";
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	HAL_StatusTypeDef res;
	uint8_t jj=0;

	rxQueueMaster = xQueueCreate( 50, sizeof( uint8_t ) );
	if( rxQueueMaster == NULL )
		printf("Queue was not created and must not be used.\n");

	printf("StartDefaultTask\n");

	for(int ii=0; ii< sizeof(txMaster); ii++)
		txMaster[ii]=ii;

	MasterToTransfer=0;

	// Обязательно сначала запускаем прием!
	if(res=HAL_UART_Receive_IT(phuartMaster, (uint8_t *)rxMasterISR, 1) == HAL_OK)
		; //printf("HAL_OK\n");
	else if(res = HAL_ERROR)
		printf("Master < HAL_ERROR\n");
	else  if(res = HAL_BUSY)
		printf("Master < HAL_BUSY\n");

	const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 50 );

	uint8_t received=0;

	for(;;)
	{
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET)
		{

			for(int ii=0; ii< 1; ii++) // это чтобы дребезга не было
				vTaskDelay(250);

			printf("\n ---- %d -----\n",jj);
			printf("Master send bytes\n");

			wasReaddenMaster=0;
			MasterToTransfer=11;

			if(jj*(MasterToTransfer+1) >= sizeof(txMaster) )
			{
				MasterToTransfer=jj*(MasterToTransfer+1) -  sizeof(txMaster); /// остаток массива
			}

			if( MasterSendATCommand (phuartMaster, (uint8_t *)(txMaster+jj*MasterToTransfer), MasterToTransfer,(uint8_t *)rx1Master, &wasReaddenMaster,rxQueueMaster, xMaxExpectedBlockTime, echoON) != 1)
				;//printf("result error\n");

			jj++;
		}
		else
		{
			received=0;
			MasterToTransfer=0;
			received = MasterGetFromQueue(rxQueueMaster, (uint8_t*)txMaster, MasterToTransfer,
					(uint8_t*)rx1Master,  xMaxExpectedBlockTime, 0);

			if(received > 0)
			{
				printf("Master (%d) <- ",received);

				for(int ii=0; ii< received ; ii++)
				{
					printf("%0.2X ",rx1Master[ii]);
				}

				printf("\n");
			}
		}

		vTaskDelay(10);
	}


	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */

	HAL_StatusTypeDef res;

	rxQueueSlave = xQueueCreate( 50, sizeof( uint8_t ) );
	if( rxQueueSlave == NULL )
		printf("rxQueueSlave Queue was not created and must not be used.\n");

	printf("StartTask02 Slave\n");

	// Обязательно сначала запускаем прием
	if(res=HAL_UART_Receive_IT(phuartSlave, (uint8_t *)rxSlaveISR, 1) == HAL_OK)
		; //printf("HAL_OK\n");
	else if(res = HAL_ERROR)
		printf("Slave HAL_ERROR\n");
	else  if(res = HAL_BUSY)
		printf("Slave HAL_BUSY\n");


	const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 250 );

	for(;;)
	{
		uint8_t received=0;
		received = SlaveGetFromQueue(rxQueueSlave, (uint8_t*)rx1Slave,  xMaxExpectedBlockTime);
		if(received > 0)
		{
			printf("Slave %d -> ",received);

			for(int ii=0; ii< received ; ii++)
			{
				printf("%0.2X ",rx1Slave[ii]);
				txSlave[ii]=rx1Slave[ii];
			}

			printf("\n");

			if (MastertoUart(phuartSlave, (uint8_t *)txSlave,received ) != HAL_OK)
				;

		}

		vTaskDelay(1);
	}
	/* USER CODE END StartTask02 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
