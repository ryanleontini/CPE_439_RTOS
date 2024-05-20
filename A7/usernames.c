/*
 * usernames.c
 *
 *  Created on: May 19, 2024
 *      Author: Ryan
 */

#include "usernames.h"
//#include "networkTable.h"
#include "usart.h"
#include "cmsis_os.h"

#define TIME_UNTIL_DEATH 110


#define MAX_USERNAME 50
//#define MAX_ENTRIES 256
#define MAX_NODES 256
#define MAX_USERNAME 21

extern UART_HandleTypeDef huart2;

typedef struct {
    uint8_t address;
    char name[MAX_USERNAME];
    int lastHeartbeat;
} NameToHex;

NameToHex usernameTable[MAX_NODES];
int usernameTableCount = 0;

void addEntryToUsernameTable(uint8_t address, const char *name) {
    if ((address > 245 || address < 9 )|| name == NULL ) {
        printf("Invalid entry!\n");
        return; // Invalid address or name
    }

    usernameTable[address].address = address;
    strncpy(usernameTable[address].name, name, MAX_USERNAME - 1);
    usernameTable[address].name[MAX_USERNAME - 1] = '\0'; // Ensure null termination
    usernameTable[address].lastHeartbeat = 0;
}

void removeEntryFromUsernameTable(uint8_t address) {
    if (usernameTable[address].name[0] != '\0') {  // Check if the slot is used
        usernameTable[address].name[0] = '\0';  // Mark the entry as unused
        return 0;  // Success
    }
//    return -1
}

uint8_t getAddressFromUsername(const char *name) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (strncmp(usernameTable[i].name, name, MAX_USERNAME) == 0) {
            return usernameTable[i].address;
        }
    }
    return 0; // Return 0 if the username is not found
}

const char* getUsernameFromAddress(uint8_t address) {
    if (address > 255) {
        return NULL; // Invalid address
    }

    if (usernameTable[address].name[0] != '\0') {
        return usernameTable[address].name;
    }
    return NULL; // Return NULL if the address is not found
}

void displayNodesTable(void) {
    char buffer[100]; // Temporary buffer for string formatting
    uint16_t length;
    char cursor[20];
    int rowsPrinted = 0; // Counter to keep track of rows printed
    int columnOffset = 0; // Offset for columns
    int column = 0;

    // Move cursor to the initial top-left position
    snprintf(cursor, sizeof(cursor), "\x1B[4;1H");
    length = strlen(cursor);
    HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

    for (int i = 0; i < MAX_NODES; i++) {
        // Check if the node name is not empty
        if (usernameTable[i].name[0] != '\0') {
            snprintf(buffer, sizeof(buffer), "%-20s | 0x%02X | %4d |\r\n",
                     usernameTable[i].name, usernameTable[i].address, usernameTable[i].lastHeartbeat);
            length = strlen(buffer);

            // Calculate the cursor position
            snprintf(cursor, sizeof(cursor), "\x1B[%d;%dH", 4 + rowsPrinted, 1 + columnOffset);
            length = strlen(cursor);
            HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);

            rowsPrinted++;

            // If 17 rows are printed, reset row counter and increase column offset
            if (rowsPrinted == 16) {
                rowsPrinted = 0;
//                column = 1;
                columnOffset += 37; // Move to the next column
            }
        }
    }

    transmitMessages();
}


void decrementNodesTable(void) {
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS; // 1000ms delay

    int nodeRemoved = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        if (usernameTable[i].name[0] != '\0') {  // Only display used entries
        	if (usernameTable[i].lastHeartbeat < TIME_UNTIL_DEATH) {
            	usernameTable[i].lastHeartbeat++;
        	}
        	else {
        		removeEntryFromUsernameTable(usernameTable[i].address);
        		nodeRemoved = 1;
        	}
        }
    }
    if (nodeRemoved) {
    	clearText(3, 17);
    }
	vTaskDelay( xDelay ); // Delay for 1000ms
}
