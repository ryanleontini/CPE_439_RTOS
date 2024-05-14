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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Spirit_Register_Init(void);
void updateNodesTable(void);
void transmit(void);
void receive(void);
void clearScreen(void);
void heartBeatTask(void);

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
//  addNode(0x45, "John1", 5);
//  addNode(0x41, "John2", 5);
//  addNode(0x42, "John3", 5);
//  addNode(0x43, "John4", 5);
//  addNode(0x44, "John5", 5);
//  addNode(0x46, "John6", 23);
//  addNode(0x47, "John7", 23);
//  addNode(0x48, "John8", 23);
//  addNode(0x49, "John9", 23);
//  addNode(0x50, "John10", 23);
  clearScreen();

  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();
//
//  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();
//
//  /* State Machine Task */
//
//  /* Data processing task */
//
//  /* Display UART Task */
//  if (xTaskCreate(updateNodesTable, "HelloWorld", STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL) != pdPASS){ while(1); }
//  if (xTaskCreate(receive, "receive", STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, &xReceiveTaskHandle) != pdPASS){ while(1); }
//  if (xTaskCreate(heartBeatTask, "heartbeat", STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, NULL) != pdPASS){ while(1); }
//
//  /* Heartbeat task */
//
//  /* Start scheduler */
//  osKernelStart();


//  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  transmit();

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

void heartBeatTask(void) {
    const TickType_t xDelay = 4000 / portTICK_PERIOD_MS; // 4s delay

	vTaskDelay( xDelay );

    if (xReceiveTaskHandle != NULL) {
        vTaskSuspend(xReceiveTaskHandle);
    }

    SpiritCmdStrobeSabort();

    transmit();

    if (xReceiveTaskHandle != NULL) {
        vTaskResume(xReceiveTaskHandle);
    }

	// Abort receive for TX
//    SpiritCmdStrobeSabort();


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
	while(1) {

		char payload[20] = "Hello World!\r\n";

		uint8_t rxLen;

		xRxDoneFlag = S_RESET;
		SPSGRF_StartRx();
		while (!xRxDoneFlag);
	//	xRxDoneFlag = S_RESET;

	    uint8_t sourceAddress;
	    sourceAddress = SpiritPktStackGetReceivedSourceAddress();


	    char* name = getNameFromHex(sourceAddress);

	    addNode(sourceAddress, name, 110);

	}

}
//
void transmit(void) {
	/* Transmit */
	while(1) {
		char payload[20] = "Hello World!\r\n";
		xTxDoneFlag = S_RESET;
		SPSGRF_StartTx(payload, strlen(payload));
		while(!xTxDoneFlag);
	//	xTxDoneFlag = S_RESET;

		HAL_UART_Transmit(&huart2, "Payload Sent\r\n", 14, HAL_MAX_DELAY);
	}

}
//
//void Spirit_Register_Init(void) {
////	SpiritSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer);
//
//}

//void displayNodesTable(void) {
//    char *clearScreen = "\x1B[H\x1B[2K"; // Command to clear the screen
//    char *moveCursor = "\x1B[H"; // Command to move cursor to the top left
//
//    uint16_t clearScreenLength = strlen(clearScreen);
//    uint16_t moveCursorLength = strlen(moveCursor);
//
//    HAL_UART_Transmit(&huart2, (uint8_t *)clearScreen, clearScreenLength, 100); // Transmit with data length and timeout
//    HAL_UART_Transmit(&huart2, (uint8_t *)moveCursor, moveCursorLength, 100); // Transmit with data length and timeout
//
//}

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
  if (xIrqStatus.IRQ_RX_DATA_DISC || xIrqStatus.IRQ_RX_TIMEOUT)
  {
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
