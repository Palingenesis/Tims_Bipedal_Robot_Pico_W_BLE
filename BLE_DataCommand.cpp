/*

	By Tim Jackson.1960

	Based on the Libray SerialCommand
		An Arduino library to tokenize and parse commands received over a serial port.
		Steven Cogswell  <steven.cogswell@gmail.com> http://awtfy.com

	Give creadit where due.

	Notes!
		Commands sent to the BLE by a client must end with a New Line (\n)

*/

#include "BLE_DataCommand.h"
#include <string.h>

BLE_DataCommand::BLE_DataCommand() {
	strncpy(delim, "#", MAXDELIMETER);
	term = '\n';
	numCommand = 0;
	clearBuffer();
}

void BLE_DataCommand::clearBuffer() {
	for (int i = 0; i < BLECOMMANDBUFFER; i++) {
		buffer[i] = '\0';
	}
	bufPos = 0;
}

char* BLE_DataCommand::next() {
	char* nextToken;
	nextToken = strtok_r(NULL, delim, &last);
	return nextToken;
}

void BLE_DataCommand::processBLEData(uint8_t* bleBuffer, size_t length) {

#ifdef BLECOMMANDDEBUG
	Serial.print("Processing BLE Data in processBLEData with data: ");
	Serial.println((char*)bleBuffer);
#endif

	for (size_t i = 0; i < length; i++) {
		inChar = bleBuffer[i];

#ifdef BLECOMMANDDEBUG
		Serial.print("Character received: ");
		Serial.print((char)inChar);	/*	Print each character received	*/
		Serial.print("\t term = ");
		Serial.println(term,HEX);
#endif

		if (inChar == term) {
			bufPos = 0;
			token = strtok_r(buffer, delim, &last);
			if (token == NULL) return;

			bool matched = false;
			for (int j = 0; j < numCommand; j++) {
				if (strncmp(token, CommandList[j].command, BLECOMMANDBUFFER) == 0) {

#ifdef BLECOMMANDDEBUG
					Serial.print("token: ");
					Serial.println(token);	/*	Check token	*/
					Serial.print("Executing command: ");
					Serial.println(CommandList[j].command);
#endif

					(*CommandList[j].function)();
					clearBuffer();
					matched = true;
					break;
				}
			}
			if (!matched && defaultHandler != nullptr) {

#ifdef BLECOMMANDDEBUG
				Serial.print("Executing default handler");
#endif

				(*defaultHandler)();
				clearBuffer();
			}
		}
		if (isprint(inChar)) {
			buffer[bufPos++] = inChar;
			buffer[bufPos] = '\0';
			if (bufPos > BLECOMMANDBUFFER - 1) bufPos = 0;
		}
	}
#ifdef BLECOMMANDDEBUG
	Serial.print("BLE_DataCommand::processBLEData Received: ");
	Serial.println((char*)buffer);	/*	Cast buffer to char* for proper printing	*/
#endif
}
#ifdef BLECOMMANDDEBUG
#endif

void BLE_DataCommand::addCommand(const char* command, void (*function)()) {
	if (numCommand < MAXSERIALCOMMANDS) {
		strncpy(CommandList[numCommand].command, command, BLECOMMANDBUFFER);
		CommandList[numCommand].function = function;
		numCommand++;
	}
	else {
#ifdef BLECOMMANDDEBUG
		Serial.println("Too many handlers - recompile changing MAXSERIALCOMMANDS");
#endif 
	}
}

void BLE_DataCommand::addDefaultHandler(void (*function)()) {
	defaultHandler = function;
}
