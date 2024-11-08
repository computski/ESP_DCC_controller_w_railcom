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

}

#endif

