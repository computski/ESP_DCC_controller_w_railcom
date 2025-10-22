// DCCEXprocessor.h
// 2025-10-03
// This is an adjunct processing module to WiThrottle, which manages TCP clients
// that module will call out to this model to process any DCCEX protcol clients
// the INO main loop calls nsWiThrottle::broadcastChanges() which in turn calls out to
// this module

// DCCEXprocessor supports JRMI Decoder Pro, connection must be over TCP/IP as the serial port is used for Railcom messages
// DCCEX itself does not support Railcom
//https://dcc-ex.com/reference/software/command-summary-consolidated.html#gsc.tab=0



#ifndef _DCCEXPROCESSOR_h
#define _DCCEXPROCESSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <ESPAsyncTCP.h>  //Github me-no-dev/ESPAsyncTCP
#include <string>   //required if you wish to compile in arduino IDE, this is the std::string library





namespace nsDCCEXprocessor {

	//duplicate of the CLIENTMESSAGE structure from the nsWiThrottle namespace
	//this one will hold DCCEX protocol messages
	struct CLIENTMESSAGE
	{
		std::string msg;   //safer inside a vector, will avoid memory leaks
		AsyncClient* toClient;  //pointer to specific client, nullptr=all clients
	};
	

	
	void handleDCCEX(void* arg, AsyncClient* client, void* data, size_t len);
	void tokenProcessor(char* msg, AsyncClient* client);
	void tokenProcessor(char* msg, AsyncClient* client, bool oldVersion);
	void buildBroadcastQueue(bool clearQueue);
	void sendToClient(AsyncClient* client);
	void broadcastSMreadResult(uint16_t cvReg, int16_t cvVal);

	//internal scope
	static void queueMessage(std::string s, AsyncClient* client);
	static void setPower(bool powerOn);
	
}

	




#endif

