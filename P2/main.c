

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
//#include "networkTable.h"
//#include "usernames.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart2;
#define STACK_SIZE 512
#define TIMER_PERIOD_MS 5000 // 30 seconds in milliseconds

#define MAX_MESSAGES 13
#define MAX_MESSAGE_LENGTH 250
#define MAX_USERNAME_LEN 21

#define HEARTBEAT 30000

#define MAX_USERNAME 21
#define MAX_MESSAGE 250
#define USERNAME "ryan"

#define CHATLINE 40
#define MAX_CHUNK_LENGTH 76


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

TaskHandle_t updateNodesTableHandle = NULL;
TaskHandle_t heartBeatTaskHandle = NULL;

char messageBuffer[MAX_MESSAGES][MAX_MESSAGE_LENGTH];
volatile int messageCount = 0;
volatile int messageIndex = 0; // To track the current message to be transmitted
char receivedChar;
int receivedFlag;

#define RX_BUFFER_SIZE 250
char rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxIndex = 0;

int broadcastFlag = 0;
int stepFlag = 0;

char txUSER[MAX_USERNAME];
char txMESSAGE[MAX_MESSAGE];

int transmitReadyFlag = 0;

//TimerHandle_t x30SecondTimer;
xSemaphoreHandle xMutex;


uint8_t currentState;
int initialTransmit = 0;
volatile int isAborting = 0;
int firstACK = 0;
int timeout = 0;
int heartbeatCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Spirit_Register_Init(void);
void updateNodesTable(void);
void transmit(uint8_t flag, char *username, char *message);
void receive(void);
void clearScreen(void);
void heartBeatTask(void);
void getSpirit1State(void);
void vTimerCallback(TimerHandle_t xTimer);
void initTerminal(void);
void transmitToUART (char *message);
void transmitTask(void);

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

//  initNodesArray();

  initTerminal();

//  addEntryToUsernameTable(0x01, "Alice");
//  addEntryToUsernameTable(0x02, "Bob");
//  addEntryToUsernameTable(0x03, "Charlie");
//  addEntryToUsernameTable(0x04, "David");
//  addEntryToUsernameTable(0x05, "Eve");
//  addEntryToUsernameTable(0x06, "Frank");
//  addEntryToUsernameTable(0x07, "Grace");
//  addEntryToUsernameTable(0x08, "Heidi");
//  addEntryToUsernameTable(0x09, "Ivan");
//  addEntryToUsernameTable(0x0A, "Judy");
//  addEntryToUsernameTable(0x0B, "Mallory");
//  addEntryToUsernameTable(0x0C, "Oscar");
//  addEntryToUsernameTable(0x0D, "Peggy");
//  addEntryToUsernameTable(0x0E, "Sybil");
//  addEntryToUsernameTable(0x0F, "Trent");
//  addEntryToUsernameTable(0x10, "Victor");
//  addEntryToUsernameTable(0x11, "Walter");
//  addEntryToUsernameTable(0x12, "Xavier");
//  addEntryToUsernameTable(0x13, "Yvonne");
//  addEntryToUsernameTable(0x14, "Zara");
//  addEntryToUsernameTable(0x15, "Quinn");
//  addEntryToUsernameTable(0x16, "Ruth");
//  addEntryToUsernameTable(0x17, "Sam");
//  addEntryToUsernameTable(0x18, "Tina");
//  addEntryToUsernameTable(0x19, "Uma");
//  addEntryToUsernameTable(0x1A, "Vera");
//  addEntryToUsernameTable(0x1B, "Wade");
//  addEntryToUsernameTable(0x1C, "Xena");
//  addEntryToUsernameTable(0x1D, "Yara");
//  addEntryToUsernameTable(0x1E, "Zeke");


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

  if (xTaskCreate(receive, "receive", STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xReceiveTaskHandle) != pdPASS){ while(1); }
  if (xTaskCreate(updateNodesTable, "updateNodes", STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &updateNodesTableHandle) != pdPASS){ while(1); }
  if (xTaskCreate(heartBeatTask, "heartbeat", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &heartBeatTaskHandle) != pdPASS){ while(1); }
  if (xTaskCreate(transmitTask, "transmit", STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, &xReceiveTaskHandle) != pdPASS){ while(1); }

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

void addFormattedMessageToBuffer(const char *username, const char *message, const char *colorCode) {
    char formattedMessage[MAX_MESSAGE_LENGTH];
    int messageLength = strlen(message);
    int chunkStart = 0;

    while (chunkStart < messageLength) {
        // Calculate the length of the current chunk
        int chunkLength = messageLength - chunkStart;
        if (chunkLength > MAX_CHUNK_LENGTH) {
            chunkLength = MAX_CHUNK_LENGTH;
        }

        // Format the current chunk
        snprintf(formattedMessage, sizeof(formattedMessage), "%s%-*.*s\x1B[0m", colorCode, chunkLength, chunkLength, &message[chunkStart]);

        // Add the formatted chunk to the message buffer
        snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "%s: %s", username, formattedMessage);

        // Update message buffer index
        messageIndex = (messageIndex + 1) % MAX_MESSAGES;
        if (messageCount < MAX_MESSAGES) {
            messageCount++;
        }

        // Move to the next chunk
        chunkStart += chunkLength;
    }
}


void heartBeatTask(void) {

//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(1) {
		const TickType_t xDelay = HEARTBEAT / portTICK_PERIOD_MS; // 10s delay

		vTaskDelay( xDelay );

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {


			transmit(3, USERNAME, NULL);

			heartbeatCount++;
            snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "Heartbeat %d Sent.", heartbeatCount);
            messageIndex = (messageIndex + 1) % MAX_MESSAGES;

            if (messageCount < MAX_MESSAGES) {
                messageCount++;
            }

            xSemaphoreGive(xMutex);

        }
	}
}

void receive(void) {

    /* Receive */

	if (initialTransmit == 0) {
//		uint8_t flag = 1;
//    	transmit(flag);

		uint16_t length;
		uint8_t rxLen;
		for (int i = 0; i < 10; i++) {
	    	transmit(1, USERNAME, NULL);
			SpiritCmdStrobeRx();
			while (!xRxDoneFlag);

            char payload[MAX_MESSAGE_LENGTH];
            rxLen = SPSGRF_GetRxData(payload);
            char* receivedUsername = (char*)&payload[1];

            payload[rxLen] = '\0'; // Ensure null-termination

	    	if (payload[0] == 2) {
				uint8_t sourceAddress;
				sourceAddress = SpiritPktStackGetReceivedSourceAddress();

//				char* name = getNameFromHex(sourceAddress);
//    			char* name = getUsernameFromAddress(sourceAddress);

//				addNode(sourceAddress, name);

				addEntryToUsernameTable(sourceAddress, receivedUsername);

				snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "First ACK received from: %s", receivedUsername);
				firstACK = 1;
				// Store the message in the buffer, overwriting the oldest if necessary
				messageIndex = (messageIndex + 1) % MAX_MESSAGES;

				if (messageCount < MAX_MESSAGES) {
					messageCount++;
				}
				break;
	    	}
		}

		if (!firstACK) {
			snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "No ACKs received. Starting network.");
			messageIndex = (messageIndex + 1) % MAX_MESSAGES;

			if (messageCount < MAX_MESSAGES) {
				messageCount++;
			}
		}

//	    xTaskNotifyGive(updateNodesTableHandle);
//	    xTaskNotifyGive(heartBeatTaskHandle);


	}

	while(1) {
//		if (transmitReadyFlag) {
//            xSemaphoreGive(xMutex);
//            vTaskDelay(10 / portTICK_PERIOD_MS);
//		}
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {

//			char payload[20];
			uint16_t length;
			uint8_t rxLen;
			timeout = 0;


			SpiritCmdStrobeFlushRxFifo();

			xRxDoneFlag = S_RESET;

			SpiritCmdStrobeRx();

			while (!xRxDoneFlag);

			if (timeout) {
	            xSemaphoreGive(xMutex);
	            vTaskDelay(10 / portTICK_PERIOD_MS);
				continue;
			}

			uint8_t sourceAddress;
			sourceAddress = SpiritPktStackGetReceivedSourceAddress();
//			char* name = getNameFromHex(sourceAddress);


			uint8_t destAddress = SpiritPktStackGetReceivedDestAddress();
			if (destAddress != 0xFF && destAddress != MY_ADDRESS) {
                xSemaphoreGive(xMutex);
                vTaskDelay(10 / portTICK_PERIOD_MS);
				continue;
			}

            char payload[MAX_MESSAGE_LENGTH];
            memset(payload, 0, sizeof(payload));

            rxLen = SPSGRF_GetRxData(payload);
//            payload[rxLen] = '\0'; // Ensure null-termination

            // Extract username
            char* receivedUsername = (char*)&payload[1];
            size_t usernameLength = strlen(receivedUsername);

            if (usernameLength == 0) {
                snprintf(receivedUsername, sizeof(payload) - 1, "%02X", sourceAddress);
            }

//			addNode(sourceAddress, receivedUsername);
			addEntryToUsernameTable(sourceAddress, receivedUsername);


            // Extract message
            char* receivedMessage = (char*)&payload[1 + usernameLength + 1]; // +1 for null terminator

			char *cursor = "\x1B[1;41H";
        	length = strlen(cursor);
        	HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

        	if (payload[0] == 1) {

        		// Transmit
    			uint8_t sourceAddress;
    			sourceAddress = SpiritPktStackGetReceivedSourceAddress();

//    			char* name = getNameFromHex(sourceAddress);
//    			char* name = getUsernameFromAddress(sourceAddress);

        		SpiritPktStackSetDestinationAddress(sourceAddress);

        		transmit(2, USERNAME, NULL);

        		SpiritPktStackSetDestinationAddress(0xFF);

                snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "Announcement received from: %s", receivedUsername);
                // Store the message in the buffer, overwriting the oldest if necessary
                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

                if (messageCount < MAX_MESSAGES) {
                    messageCount++;
                }

        	}
        	else if (payload[0] == 2) {
                snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "ACK received from: %s", receivedUsername);
                firstACK = 1;
                // Store the message in the buffer, overwriting the oldest if necessary
                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

                if (messageCount < MAX_MESSAGES) {
                    messageCount++;
                }
            }
        	else if (payload[0] == 3) {
                snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "Heartbeat received from: %s", receivedUsername);
                // Store the message in the buffer, overwriting the oldest if necessary

//                const char *txMESSAGE = "This is a very long message that needs to be split into multiple lines to ensure that the UART does not break and the message remains readable.";
//                const char *colorCode = "\x1B[32m\x1B[47m";  // Green text on a white background
////
//                addFormattedMessageToBuffer(receivedUsername, txMESSAGE, colorCode);

                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

                if (messageCount < MAX_MESSAGES) {
                    messageCount++;
                }
            }
        	else if (payload[0] == 4) {
				if (destAddress == 0xFF) {
	                const char *colorCode = "\x1B[32m\x1B[47m";  // Green text on a white background

//	                addFormattedMessageToBuffer(receivedUsername, receivedMessage, colorCode);
					snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "\x1B[32m\x1B[47m%s: Broadcast: %s\x1B[0m", receivedUsername, receivedMessage);

//	                 Store the message in the buffer, overwriting the oldest if necessary
	                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

	                if (messageCount < MAX_MESSAGES) {
	                    messageCount++;
	                }
	            /* It's my address */
				} else {
	                const char *colorCode = "\x1B[34m\x1B[47m";  // blue text on a white background

//	                addFormattedMessageToBuffer(receivedUsername, receivedMessage, colorCode);
					snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "\x1B[34m\x1B[47m%s: Message: %s\x1B[0m", receivedUsername, receivedMessage);
	                // Store the message in the buffer, overwriting the oldest if necessary
	                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

	                if (messageCount < MAX_MESSAGES) {
	                    messageCount++;
	                }
				}
            }
        	else {
        		// Need to stop printing on timeouts.
        		if (!timeout) {
                    snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "%s: Message: %s", receivedUsername, payload);
                    messageIndex = (messageIndex + 1) % MAX_MESSAGES;

                    if (messageCount < MAX_MESSAGES) {
                        messageCount++;
                    }
        		}
        	}

            xSemaphoreGive(xMutex);
            vTaskDelay(10 / portTICK_PERIOD_MS);

        }
	}

}

void transmitUARTBuf(void) {
    char cursorPosition[10];
    snprintf(cursorPosition, sizeof(cursorPosition), "\x1B[40;0H");
    HAL_UART_Transmit(&huart2, (uint8_t *)cursorPosition, strlen(cursorPosition), HAL_MAX_DELAY);
	transmitToUART(rxBuffer);
}

void updateNodesTable(void) {
//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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
}

void clearLeft(void) {
    uint16_t length;
    char row[20]; // Buffer for the formatted row position string
    char *column = "\x1B[40X";

    for (int i = 1; i <= 24; ++i) { // Assuming a 24-row terminal for this example
        // Format the row position string
        snprintf(row, sizeof(row), "\x1B[%d;1H", i);
        length = strlen(row);
        HAL_UART_Transmit(&huart2, (uint8_t *)row, length, 100);

        // Transmit the clear column command
        length = strlen(column);
        HAL_UART_Transmit(&huart2, (uint8_t *)column, length, 100);
    }
}

void cursor(void) {
    uint16_t length;

	char *cursor = "\x1B[1;41H";
	length = strlen(cursor);
	HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);
}

void transmitMessages(void) {
    uint16_t length;
    char cursor[20]; // Buffer for cursor position string

    // Start from row 35 and clear 13 rows upwards
    int startRow = 35;
    clearText(startRow - 13, 14);

    for (int i = 0; i < messageCount; i++) {
        // Calculate cursor position for the current message
        snprintf(cursor, sizeof(cursor), "\x1B[%d;1H", startRow - i);
        length = strlen(cursor);
        HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

        int index = (messageIndex - 1 - i + MAX_MESSAGES) % MAX_MESSAGES;

        HAL_UART_Transmit(&huart2, (uint8_t *)messageBuffer[index], strlen(messageBuffer[index]), HAL_MAX_DELAY);
    }
}

void USART2_IRQHandler(void) {
   if (USART2->ISR & (USART_ISR_RXNE)) {  // Check if receive not empty
       receivedChar = USART2->RDR;  // Read received character
	   receivedFlag = 1;

       if (receivedChar == '\r' || receivedChar == '\n') {
           rxBuffer[rxIndex] = '\0'; // Null-terminate the string

           if (stepFlag == 0) {
        	   if (rxBuffer[0] == 'b') {
        		   broadcastFlag = 1;
            	   stepFlag = 1;
                   clearInput(CHATLINE);
                   transmitUserCommand("Please enter a message:");
        	   }
        	   else if (rxBuffer[0] == 'm') {
        		   broadcastFlag = 0;
            	   stepFlag = 2;
                   clearInput(CHATLINE);
//                   transmitUserCommand("Please enter your username:");
                   transmitUserCommand("Please enter a username to send to:");

        	   }
        	   else {
                   clearInput(CHATLINE);
                   transmitUserCommand("Invalid command. Please try again. Enter b (broadcast) or m (message):");
            	   stepFlag = 0;

        	   }

           }
           /* Message entered */
           else if (stepFlag == 1) {

               strncpy(txMESSAGE, rxBuffer, MAX_MESSAGE - 1);
               txMESSAGE[strlen(rxBuffer)] = '\0'; // Ensure null termination
        	   /* Need to error handle rxBuffer being bigger */

        	   stepFlag = 0;
               rxIndex = 0; // Reset index for new input
               clearInput(CHATLINE);
               transmitUserCommand("Please enter b (broadcast) or m (message):");

               /* Run TX Task */
               transmitReadyFlag = 1;
           }
           /* Message entered */
           else if (stepFlag == 2) {

               strncpy(txUSER, rxBuffer, MAX_USERNAME - 1);
               txUSER[strlen(rxBuffer)] = '\0'; // Ensure null termination
               stepFlag = 1;


               clearInput(CHATLINE);
               transmitUserCommand("Please enter a message:");
           }

           // Process the received command
           rxIndex = 0; // Reset index for new input
       } else {
           // Add the character to the buffer if there is space
           if (rxIndex < RX_BUFFER_SIZE - 1) {
               rxBuffer[rxIndex++] = receivedChar;
           }
       }

       char cursorPosition[10];
       snprintf(cursorPosition, sizeof(cursorPosition), "\x1B[40;%dH", rxIndex);
       HAL_UART_Transmit(&huart2, (uint8_t *)cursorPosition, strlen(cursorPosition), HAL_MAX_DELAY);
	   USART2->TDR = receivedChar;  // Echo the received character to terminal
       USART2->ISR &= ~(USART_ISR_RXNE);  // Clear the RXNE flag
   }
}

void clearInput(int line) {
    char cursorPosition[10];
    snprintf(cursorPosition, sizeof(cursorPosition), "\x1B[%d;H", line);
    HAL_UART_Transmit(&huart2, (uint8_t *)cursorPosition, strlen(cursorPosition), HAL_MAX_DELAY);

    cursorPosition[10];
    snprintf(cursorPosition, sizeof(cursorPosition), "\x1b[K");
    HAL_UART_Transmit(&huart2, (uint8_t *)cursorPosition, strlen(cursorPosition), HAL_MAX_DELAY);
}

void transmitTask(void) {
	while (1) {
		if (transmitReadyFlag) {
		    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		    	/* If broadcast */
		    	if (broadcastFlag) {
		    		SpiritPktStackSetDestinationAddress(0xFF); // Broadcast
					snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "\x1B[32m\x1B[47m%s: Broadcast: %s\x1B[0m", USERNAME, txMESSAGE);

//	                const char *colorCode = "\x1B[32m\x1B[47m";  // Green text on a white background

//	                addFormattedMessageToBuffer(USERNAME, txMESSAGE, colorCode);

		    		transmit(4, USERNAME, txMESSAGE);
					broadcastFlag = 0;
		    	} else {
		    		/* Get hex from username */
		            uint8_t txAddr = getAddressFromUsername(txUSER);

		    		SpiritPktStackSetDestinationAddress(txAddr); // User
					snprintf(messageBuffer[messageIndex], MAX_MESSAGE_LENGTH, "\x1B[34m\x1B[47m%s: Message: %s\x1B[0m", USERNAME, txMESSAGE);

//	                const char *colorCode = "\x1B[34m\x1B[47m";  // Green text on a white background

//	                addFormattedMessageToBuffer(USERNAME, txMESSAGE, colorCode);

		    		transmit(4, USERNAME, txMESSAGE);
					SpiritPktStackSetDestinationAddress(0xFF);
		    	}

                messageIndex = (messageIndex + 1) % MAX_MESSAGES;

                if (messageCount < MAX_MESSAGES) {
                    messageCount++;
                }
                transmitReadyFlag = 0;
		        xSemaphoreGive(xMutex);
		    }
		}
	}
}

void transmit(uint8_t flag, char *username, char *message) {

    char user[MAX_USERNAME];
    strncpy(user, username, MAX_USERNAME - 1);
    user[MAX_USERNAME - 1] = '\0'; // Ensure null termination

	uint8_t payload[250] = {0};
	payload[0] = flag;

    strncpy((char *)&payload[1], user, sizeof(payload) - 2);
    payload[sizeof(payload) - 1] = '\0';

    uint8_t txLen = 1 + strlen((char *)&payload[1]) + 1; // Flag + username + null terminator

    if (message != NULL) {
        // Ensure null termination for the username within the payload
        size_t usernameLength = strlen((char *)&payload[1]);
        strncpy((char *)&payload[1 + usernameLength + 1], message, sizeof(payload) - (1 + usernameLength + 1) - 1);
        txLen += strlen((char *)&payload[1 + usernameLength + 1]) + 1; // Add the length of the message and its null terminator
    }

	xTxDoneFlag = S_RESET;

	isAborting = 1;
	SpiritCmdStrobeSabort();
	SPSGRF_StartTx(payload, txLen);
	isAborting = 0;

	while(!xTxDoneFlag);
	isAborting = 1;
	SpiritCmdStrobeSabort();
	isAborting = 0;

	// Clear message payload.
    memset(txMESSAGE, 0, sizeof(txMESSAGE));

}

void initTerminal(void) {
    char *resetAttributes = "\x1B[0m";

    char *cursorHome = "\x1B[H";
    char *cursorSecondRow = "\x1B[2H";
    char *cursorThirdRow = "\x1B[3H";

    char *terminalHome = "\x1B[20H";
    char *terminalSecondRow = "\x1B[21H";
    char *terminalThirdRow = "\x1B[22H";

    char *messageHome = "\x1B[36H";
    char *messageSecondRow = "\x1B[37H";
    char *messageThirdRow = "\x1B[38H";

	clearScreen();
	transmitToUART(resetAttributes);
	/* Users Online */
    char *header = "---------------------------------------------------------------------------------------------";
    char *message = "Username             | Addr | HB   | Username             | Addr | HB   |";

	transmitToUART(cursorHome);
	transmitToUART(header);
	transmitToUART(cursorSecondRow);
	transmitToUART(message);
	transmitToUART(cursorThirdRow);
	transmitToUART(header);

	/* Terminal */
    char *termMessage = "Console";

	transmitToUART(terminalHome);
	transmitToUART(header);
	transmitToUART(terminalSecondRow);
	transmitToUART(termMessage);
	transmitToUART(terminalThirdRow);
	transmitToUART(header);

	/* Inbox */
    char *inboxMessage = "Chat";

	transmitToUART(messageHome);
	transmitToUART(header);
	transmitToUART(messageSecondRow);
	transmitToUART(inboxMessage);
	transmitToUART(messageThirdRow);
	transmitToUART(header);

    char *inputMessageHome = "\x1B[39H";
    char *inputMessage = "Please enter b (broadcast) or m (message):";

	transmitToUART(inputMessageHome);
	transmitToUART(inputMessage);

}

void transmitToUART (char *message) {
    uint16_t length;
    length = strlen(message);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, length, 100);
}

void transmitUserCommand(char *message) {
    char *inputMessageHome = "\x1B[39H";
    clearInput(39);
	transmitToUART(inputMessageHome);
	transmitToUART(message);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;  // Variable to check if a context switch is required


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
	  xRxDoneFlag = S_SET;
      SpiritIrqClearStatus();
  }
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
	  xRxDoneFlag = S_SET;
	  timeout = 1;

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
