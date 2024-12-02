
// Railcom.h
//2024-08-28 JOssowski
//Provides debug info over the websocket and also decodes incoming railcom data

#ifndef _RAILCOM_h
#define _RAILCOM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif




namespace nsRailcom {

	void railcomInit();
	void railcomLoop(void);
	void railcomLoopDebug(void);
	void readRailcom(uint16_t addr, bool useLongAddr);
	

	/*local scope*/
	static bool decodeRailcom(uint8_t inByte, uint8_t* dataOut, bool ignoreControlChars);
	static bool decodeRailcom(uint8_t* inByte, bool ignoreControlChars);
	
}

#endif

