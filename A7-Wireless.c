/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "freeRTOS.h"
#include "semphr.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "timers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "spsgrf.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "SPIRIT_Config.h"
#include "networkTable.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart2;
#define STACK_SIZE 512
#define TIMER_PERIOD_MS 5000 // 30 seconds in milliseconds

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile SpiritFlagStatus xTxDoneFlag;
volatile SpiritFlagStatus xRxDoneFlag;
TaskHandle_t xReceiveTaskHandle = NULL;
//TimerHandle_t x30SecondTimer;
xSemaphoreHandle xMutex;


uint8_t currentState;
int initialTransmit = 0;
volatile int isAborting = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Spirit_Register_Init(void);
void updateNodesTable(void);
void transmit(uint8_t flag);
void receive(void);
void clearScreen(void);
void heartBeatTask(void);
void getSpirit1State(void);
void vTimerCallback(TimerHandle_t xTimer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  __enable_irq();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  SPSGRF_Init();
//  Spirit_Register_Init();
  SpiritPktStackSetDestinationAddress(0xFF); // Me

  initNodesArray();
  clearScreen();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* State Machine Task */

  /* Data processing task */

  /* Display UART Task */

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
      // Handle error
      while(1);
  }

  if (xTaskCreate(updateNodesTable, "updateNodes", STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL) != pdPASS){ while(1); }
  if (xTaskCreate(receive, "receive", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xReceiveTaskHandle) != pdPASS){ while(1); }
  if (xTaskCreate(heartBeatTask, "heartbeat", STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, NULL) != pdPASS){ while(1); }

//  if (xTaskCreate(transmit, "transmit", STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, &xReceiveTaskHandle) != pdPASS){ while(1); }
//  x30SecondTimer = xTimerCreate("30SecondTimer", pdMS_TO_TICKS(TIMER_PERIOD_MS), pdTRUE, (void *)0, vTimerCallback);


//  if (x30SecondTimer == NULL)
//  {
//      // The timer was not created
//      printf("Timer creation failed.\n");
//      while (1);
//  }
//  else
//  {
//      // Start the timer
//      if (xTimerStart(x30SecondTimer, 0) != pdPASS)
//      {
//          // The timer could not be started
//          printf("Timer start failed.\n");
//          while (1);
//      }
//  }
  /* Heartbeat task */

  /* Start scheduler */
  osKernelStart();


  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  receive();
//	  transmit();

//	  displayNodesTable();
//	  HAL_Delay(1000);
//	  transmit();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void getSpirit1State(void) {
    // Read the status register
//	uint8_t status;
//    StatusBytes spiritStatus = SpiritSpiReadRegisters(MC_STATE0_BASE, 1, &status);

    SpiritStatus currentStatus;
    SpiritSpiReadRegisters(MC_STATE0_BASE, 1, (uint8_t *)&currentStatus);

    //    currentStatus.MC_STATE = (spiritStatus.Status & 0x70) >> 4; // Extract the relevant bits

    // Initialize other fields if necessary
//    uint8_t stateBits = currentStatus.MC_STATE; // Assuming MC_STATE gives the state bits (consult the datasheet)

//    return currentStatus.MC_STATE;
}

void vTimerCallback(TimerHandle_t xTimer)
{
    // Call the transmit function
	isAborting = 1;
    SpiritCmdStrobeSabort();
    SpiritStatus currentStatus;
    SpiritSpiReadRegisters(MC_STATE0_BASE, 1, &currentStatus);
    transmit(3);
	isAborting = 0;

    printf("Transmit function called by timer.\n");
}


void heartBeatTask(void) {
	while(1) {
		const TickType_t xDelay = 5000 / portTICK_PERIOD_MS; // 4s delay

		vTaskDelay( xDelay );

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {


			if (xReceiveTaskHandle != NULL) {
				vTaskSuspend(xReceiveTaskHandle);
			}

			transmit(3);
//			isAborting = 0;

		//    taskEXIT_CRITICAL();

			if (xReceiveTaskHandle != NULL) {
				vTaskResume(xReceiveTaskHandle);
			}
            xSemaphoreGive(xMutex);

        }
	}
}

void updateNodesTable(void) {
	while(1) {
		displayNodesTable();
		decrementNodesTable();
	}
}

void clearScreen(void) {
	  uint16_t length;

	char *clear = "\x1B[2J\x1B[H";
	length = strlen(clear);
	HAL_UART_Transmit(&huart2, (uint8_t *)clear, length, 100);
	char *cursor = "\x1B[H";
	length = strlen(cursor);
	HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);
	char *ready = "Ready to receive. \n";
	length = strlen(ready);
	HAL_UART_Transmit(&huart2, (uint8_t *)ready, length, 100);
}

void receive(void) {

    /* Receive */

	if (initialTransmit == 0) {
    	transmit(1);
	}

	while(1) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {

			char payload[20];
			uint16_t length;
			uint8_t rxLen;

			SpiritCmdStrobeFlushRxFifo();

			xRxDoneFlag = S_RESET;

			SpiritCmdStrobeRx();

			while (!xRxDoneFlag);

			uint8_t sourceAddress;
			sourceAddress = SpiritPktStackGetReceivedSourceAddress();

			char* name = getNameFromHex(sourceAddress);

			addNode(sourceAddress, name, 110);

			char *cursor = "\x1B[1;41H";
        	length = strlen(cursor);
        	HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

		    rxLen = SPSGRF_GetRxData(payload);
		    HAL_UART_Transmit(&huart2, "Node says: ", 10, HAL_MAX_DELAY);
		    HAL_UART_Transmit(&huart2, payload, rxLen, HAL_MAX_DELAY);

            xSemaphoreGive(xMutex);

        }
	}

}
//
void transmit(uint8_t flag) {
	/* Transmit */
//	while (1) {
//	    const TickType_t xDelay = 2000 / portTICK_PERIOD_MS; // 4s delay
//
//		vTaskDelay( xDelay );

//			uint8_t flag = flag;
		    uint8_t payload[100];
		    payload[0] = flag;
			xTxDoneFlag = S_RESET;
//			getSpirit1State();
			SPSGRF_StartTx(flag, strlen(flag));
		    SpiritStatus currentStatus;
		    SpiritSpiReadRegisters(MC_STATE0_BASE, 1, (uint8_t *)&currentStatus);
			while(!xTxDoneFlag);
	//        xSemaphoreTake(xTxDoneSemaphore, portMAX_DELAY);  // Wait for semaphore

		//	xTxDoneFlag = S_RESET;

//			HAL_UART_Transmit(&huart2, "Payload Sent\r\n", 14, HAL_MAX_DELAY);
//	}


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus;

  if (GPIO_Pin != SPIRIT1_GPIO3_Pin)
  {
    return;
  }

  SpiritIrqGetStatus(&xIrqStatus);
  if (xIrqStatus.IRQ_TX_DATA_SENT)
  {
    xTxDoneFlag = S_SET;
    SpiritIrqClearStatus();

  }
  if (xIrqStatus.IRQ_RX_DATA_READY)
  {
    xRxDoneFlag = S_SET;
    SpiritIrqClearStatus();

  }
  if (xIrqStatus.IRQ_RX_DATA_DISC)
  {
      if (!isAborting) // Check if the abort process is happening
      {
          SpiritCmdStrobeRx();
      }
    SpiritIrqClearStatus();
  }
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
//      if (!isAborting) // Check if the abort process is happening
//      {
	  SpiritCmdStrobeRx();
	  SpiritIrqClearStatus();

  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
