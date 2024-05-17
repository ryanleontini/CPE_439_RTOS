/*
 * networkTable.c
 *
 *  Created on: May 11, 2024
 *      Author: Ryan
 */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "networkTable.h"
#include "usart.h"
#include "cmsis_os.h"


extern UART_HandleTypeDef huart2;

#define TOTAL_NODES 17
#define MAX_NODES 256
#define TIME_UNTIL_DEATH 20

typedef struct {
    uint8_t hexValue;
    const char* name;
} HexToName;

HexToName lookupTable[] = {
    {178, "C_BAE"},        // 0xB2
    {163, "D_CALDERA"},    // 0xA3
    {107, "L_CAPUTI"},     // 0x6B
    {106, "W_COLBURN"},    // 0x6A
    {127, "D_CURIEL"},     // 0x7F
    {27,  "N_DELAPENA"},   // 0x1B
    {119, "A_DOSANJH"},    // 0x77
    {135, "H_EVANS"},      // 0x87
    {123, "M_FESLER"},     // 0x7B
    {29,  "J_GALICINAO"},  // 0x1D
    {201, "T_GREEN"},      // 0xC9
    {61,  "A_GROTE"},      // 0x3D
    {133, "M_HERRERA"},    // 0x85
    {125, "B_KENNEDY"},    // 0x7D
    {22,  "J_KRAMMER"},    // 0x16
    {152, "R_LEONTINI"},   // 0x98
    {11,  "J_MANESH"},     // 0x0B
    {48,  "S_MARTIN"},     // 0x30
    {15,  "N_MASTEN"},     // 0x0F
    {140, "L_MCCARTHY"},   // 0x8C
    {199, "P_MULPURU"},    // 0xC7
    {62,  "M_NOON"},       // 0x3E
    {38,  "J_PARK"},       // 0x26
    {128, "L_PEDROZA"},    // 0x80
    {73,  "D_PETERS"},     // 0x49
    {19,  "M_PROVINCE"},   // 0x13
    {121, "A_RAJESH"},     // 0x79
    {162, "J_RAMIREZ"},    // 0xA2
    {193, "D_ROBERDS"},    // 0xC1
    {111, "D_ROLAND"},     // 0x6F
    {176, "D_SANDALL"},    // 0xB0
    {96,  "S_SELTZER"},    // 0x60
    {63,  "J_SHAFFER"},    // 0x3F
    {160, "M_WONG"},       // 0xA0
    {105, "A_TAYLOR"}		// 0x69
};


const char* getNameFromHex(uint8_t hexValue) {
    int tableSize = sizeof(lookupTable) / sizeof(lookupTable[0]);
    for (int i = 0; i < tableSize; i++) {
        if (lookupTable[i].hexValue == hexValue) {
            return lookupTable[i].name;
        }
    }
    return "Unknown";
}


typedef struct {
    char name[MAX_NAME_LEN];
    uint8_t address[MAX_ADDRESS_LEN];
    uint32_t lastHeartbeat; // Seconds since last heartbeat
} Node;

Node nodes[MAX_NODES];

void initNodesArray() {
    for (int i = 0; i < MAX_NODES; i++) {
        nodes[i].name[0] = '\0';
    }
}

int addNode(uint8_t address, const char *name) {
//    if (nodes[address].name[0] == '\0') {  // Check if the slot is unused
        strncpy(nodes[address].name, name, MAX_NAME_LEN - 1);
        nodes[address].name[MAX_NAME_LEN - 1] = '\0';
        sprintf(nodes[address].address, "%X", address);  // Format address as hex string
        nodes[address].lastHeartbeat = 0;
        return 0;  // Success
//    }
//    return -1;  // Node already exists
}

int removeNode(uint8_t address) {
    if (nodes[address].name[0] != '\0') {  // Check if the slot is used
        nodes[address].name[0] = '\0';  // Mark the entry as unused
        return 0;  // Success
    }
    return -1;  // Node not found or already empty
}

void decrementNodesTable(void) {
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS; // 1000ms delay

    int nodeRemoved = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].name[0] != '\0') {  // Only display used entries
        	if (nodes[i].lastHeartbeat < TIME_UNTIL_DEATH) {
            	nodes[i].lastHeartbeat++;
        	}
        	else {
        		removeNode(i);
        		nodeRemoved = 1;
        	}
        }
    }
    if (nodeRemoved) {
    	clearText(3, 17);
    }
	vTaskDelay( xDelay ); // Delay for 1000ms
}

void displayNodesTable(void) {
    char buffer[100]; // Temporary buffer for string formatting
    uint16_t length;
    char cursor[20];

    // Move cursor to top left
    snprintf(cursor, sizeof(cursor), "\x1B[4H");
    length = strlen(cursor);
    HAL_UART_Transmit(&huart2, (uint8_t *)cursor, length, 100);

    for (int i = 0; i < MAX_NODES; i++) {
        // Check if the node name is not empty
        if (nodes[i].name[0] != '\0') {
            snprintf(buffer, sizeof(buffer), "%-20s | %-4s | Online   | %4d\r\n",
                     nodes[i].name, nodes[i].address, nodes[i].lastHeartbeat);
            length = strlen(buffer);
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, length, 100);
        }
    }

    transmitMessages();
}

void clearText(int startingRow, int numRows) {
    uint16_t length;
    char row[20]; // Buffer for the formatted row position string
    char *column = "\x1B[K";

    for (int i = 1; i < numRows; ++i) {
        // Format the row position string
        snprintf(row, sizeof(row), "\x1B[%d;1H", startingRow + i);
        length = strlen(row);
        HAL_UART_Transmit(&huart2, (uint8_t *)row, length, 100);

        // Transmit the clear column command
        length = strlen(column);
        HAL_UART_Transmit(&huart2, (uint8_t *)column, length, 100);
    }
}
