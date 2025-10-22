// LocoNetprocessor.h

#ifndef _LOCONETPROCESSOR_h
#define _LOCONETPROCESSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <ESPAsyncTCP.h>  //Github me-no-dev/ESPAsyncTCP
#include <string>   //required if you wish to compile in arduino IDE, this is the std::string library





namespace nsLOCONETprocessor {

	//duplicate of the CLIENTMESSAGE structure from the nsWiThrottle namespace
	//this one will hold DCCEX protocol messages
	struct CLIENTMESSAGE
	{
		std::string msg;   //safer inside a vector, will avoid memory leaks
		AsyncClient* toClient;  //pointer to specific client, nullptr=all clients
	};



	void handleLocoNet(void* arg, AsyncClient* client, void* data, size_t len);
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

