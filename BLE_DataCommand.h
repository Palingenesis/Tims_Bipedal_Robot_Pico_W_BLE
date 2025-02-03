/*

    By Tim Jackson.1960

    Based on the Libray SerialCommand
        An Arduino library to tokenize and parse commands received over a serial port.
        Steven Cogswell  <steven.cogswell@gmail.com> http://awtfy.com

    Give creadit where due.

    Notes!
        Commands sent to the BLE by a client must end with a New Line (\n)

*/

#ifndef BLE_DataCommand_h
#define BLE_DataCommand_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#define BLECOMMANDDEBUG

#define BLECOMMANDBUFFER 35
#define MAXSERIALCOMMANDS 16
#define MAXDELIMETER 2


class BLE_DataCommand
{
public:
    BLE_DataCommand();

    void clearBuffer();
    char* next();
    void processBLEData(uint8_t* bleBuffer, size_t length);
    void addCommand(const char*, void(*)());
    void addDefaultHandler(void (*function)());

private:
    char inChar;
    char buffer[BLECOMMANDBUFFER];
    int  bufPos;
    char delim[MAXDELIMETER];
    char term;
    char* token;
    char* last;
    typedef struct _callback {
        char command[BLECOMMANDBUFFER];
        void (*function)();
    } BLE_DataCommandCallback;
    int numCommand;
    BLE_DataCommandCallback CommandList[MAXSERIALCOMMANDS];
    void (*defaultHandler)();
};

#endif	/*	BLE_DataCommand_h	*/	

