/*
 * usernames.h
 *
 *  Created on: May 19, 2024
 *      Author: Ryan
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#ifndef INC_USERNAMES_H_
#define INC_USERNAMES_H_

void addEntryToUsernameTable(uint8_t address, const char *name);
void removeEntryFromUsernameTable(uint8_t address);
uint8_t getAddressFromUsername(const char *name);

#endif /* INC_USERNAMES_H_ */

