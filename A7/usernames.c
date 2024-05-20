/*
 * usernames.c
 *
 *  Created on: May 19, 2024
 *      Author: Ryan
 */

#include "usernames.h"
#include "networkTable.h"

#define MAX_USERNAME 50
#define MAX_ENTRIES 100

typedef struct {
    uint8_t address;
    char name[MAX_USERNAME];
} NameToHex;

NameToHex usernameTable[MAX_ENTRIES];
int usernameTableCount = 0;

void addEntryToUsernameTable(uint8_t address, const char *name) {
    if (usernameTableCount < MAX_ENTRIES) {
        usernameTable[usernameTableCount].address = address;
        strncpy(usernameTable[usernameTableCount].name, name, MAX_USERNAME - 1);
        usernameTable[usernameTableCount].name[MAX_USERNAME - 1] = '\0'; // Ensure null termination
        usernameTableCount++;
    } else {
        printf("Lookup table is full!\n");
    }
}

void removeEntryFromUsernameTable(uint8_t address) {
    for (int i = 0; i < usernameTableCount; i++) {
        if (usernameTable[i].address == address) {
            for (int j = i; j < usernameTableCount - 1; j++) {
                usernameTable[j] = usernameTable[j + 1];
            }
            usernameTableCount--;
            return;
        }
    }
}


uint8_t getAddressFromUsername(const char *name) {
    for (int i = 0; i < usernameTableCount; i++) {
        if (strncmp(usernameTable[i].name, name, MAX_USERNAME) == 0) {
            return usernameTable[i].address;
        }
    }
    return 0; // Return 0 if the username is not found
}
