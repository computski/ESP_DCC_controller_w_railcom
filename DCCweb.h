
// DCCweb.h
//2024-04-25 updated to use LittleFS and ArduinoJson 7.0.4
//2024-08-28 sendJson() made visible for railcom


#ifndef _DCCWEB_h
#define _DCCWEB_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Global.h"
#include <ArduinoJson.h>   //from arduino library manager.  Using version 7.0.4
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <WebSockets.h>  //from arduino library manager. Markus Sattler v2.1
#include <WebSocketsServer.h>
#include <ESP8266WiFi.h>



namespace nsDCCweb {

	void startWebServices();
	void loopWebServices(void);
	void broadcastPower(void);
	void broadcastSMreadResult(uint16_t cvReg, int16_t cvVal);
	void broadcastPOMreadResult(uint16_t cvReg, int16_t cvVal, char addrType, uint16_t addr);
	void broadcastChanges(void);
	void sendJson(JsonObject& out);   //needs to be visible to railcom routines
	void sendJson(JsonDocument out);


	static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
	static void DCCwebWS(JsonDocument doc);
	static bool changeToTurnout(uint8_t slot, uint16_t addr, const char* name);
	static bool changeToTurnout(uint8_t slot, const char* addr, const char* name);
	static bool changeToSlot(uint8_t slot, uint16_t address, bool useLong, bool use128, const char* name);
	static bool changeToSlot(uint8_t slot, const char* addr, bool useLong, bool use128, const char* name);
	static void setPower(bool powerOn);
	static bool cBool(const char* v);
}

#endif
