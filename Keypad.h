// Keypad.h

#ifndef _KEYPAD_h
#define _KEYPAD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


//dependencies
#include <Wire.h>


#define KEY_REPEAT_PERIOD1    50  //.5 sec when called every 10mS
#define KEY_REPEAT_PERIOD2    20  //.2 sec when called every 10mS
#define KEY_REPEAT_LONG		  150 //1.5 sec long repeat for MODE	

//define key-codes for these virtual keys on the keypad.  Only KEY_ESTOP and KEY_MODE defines need to be visible globally.
#define KEY_ESTOP	26
#define KEY_MODE	25
#define KEY_ESTOP_PAIR 24
#define KEY_MODE_PAIR 23
#define KEY_SINGLE_LOCO_ESTOP 18

const char ASCIImapping[] = "D#0*C987B654A321";


 /*key holds the current valid key. key flag is raised on every press or repeat press if held*/
struct KEYPAD {
	uint8_t group1;
	uint8_t group2;
	uint8_t resA;
	uint8_t resB;
	uint8_t resC;
	uint8_t key;
	char    keyASCII;
	bool    keyFlag;
	bool    keyHeld;
	bool    requestKeyUp;  //callback for a keyup event
	uint8_t keyTimer;
};


extern KEYPAD keypad;

void keyScan(void);

#endif
