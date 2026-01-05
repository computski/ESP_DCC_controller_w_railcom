/*
	Name:       ESP_DCC_Controller.ino
	updated:	2025-12-18
	Author:     Julian Ossowski
	Target:		WeMOS D1 (R1 or mini)
	Note:		This device is 4M flash with 1M spiffs. The spiffs hold the webserver files
				and must be uploaded separately via vMicro publish server feature
	Serial:		default speed on the serial is 115200
	IDE:		vMicro inside MS Visual studio.  Should also compile in the Arduino IDE.

	Hardware note: the ESP is a 3v3 device, the 1602 LCD and its backpack can run on 5v, as can the INA current monitor.
	It is recommended to run the I2C bus as 5v.  The ESP inputs are 5v tolerant and will work with a 5v I2C bus.
	

 Important: Tested and works with these library versions
 Adafruit INA219 library 1.0.3 works
 2024-04-25 switch to ArduinoJSON library 7.0.4


 These libraries need to be downloaded and put in the arduino libraries folder
 ESPAsyncTCP  https://github.com/me-no-dev/ESPAsyncTCP
 NewLiquidCrystal https://github.com/marcmerlin/NewLiquidCrystal

 These can be loaded through the arduino library manager
 WebSockets
 Adafruit INA219
 ArduinoJson
*/




#include "LocoNetprocessor.h"  //added 2025-12
#include <splash.h>
#include <Adafruit_SSD1306.h> //added 2024-11-26 for latest adafruit INA support
#include <Adafruit_NeoPixel.h> //added 2024-11-26 for latest adafruit INA support
#include <SocketIOclient.h>  //added 2024-11-26, also updated library to ver 2.6.1 from 2.4.0
#include <WebSockets.h>
#include <WebSockets4WebServer.h>
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>
#include <WebSocketsVersion.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Wire.h>
#include "Global.h"
#include "DCCcore.h"
#include "DCClayer1.h"
#include "DCCweb.h"
#include "WiThrottle.h"


//PIN ASSIGNMENTS - see global.h
//found that NMI in DCClayer1 conflicts with the WSP826WebServer. 
//Had to switch to regular ints.  There is no conflict with Websockets

uint8_t secCount;

void setup() {
	Serial.begin(115200);
	Serial.println(F("\n\nBoot DCC ESP"));
	trace(
		Serial.println(F("trace enabled"));
		_Pragma("GCC warning \" TRACE IS ENABLED\"")
			)
		//2021-10-19 the unit can operate in DCC mode, or in DC mode pwm which supports a single loco, loco 3 with 28 speed steps
		//enable or disable the DC block as required in Global.h  DCC and DC are mutually exclusive

#ifdef	DC_PINS
//If DC_PINS is defined, this overrides DCC and we will create a DC system.  Entirel optional. If you want 
//a DCC system, then comment out or delete the DC_PINS definition in Global.h for the board you are using
DC_PINS

#elif defined DCC_PINS
//we expect to find DCC_PINS defined
DCC_PINS
#else
//need to define at least DCC_PINS, else we throw a compile time error.
#error "DCC_PINS or DC_PINS must be defined.  Neither is."
#endif


//restore settings from EEPROM
dccGetSettings();

//2025-08-15 now boot
DCCcoreBoot();

nsJogWheel::jogInit();

#ifdef _DCCWEB_h		
	nsDCCweb::startWebServices();
#endif

/*2024-08-28 start railcom protocol*/
railcomInit();


	/*2020-04-02 start WiThrottle protocol*/
#ifdef _WITHROTTLE_h
	nsWiThrottle::startThrottle();
#endif

	   


} //end boot




void loop() {

#ifdef _DCCWEB_h
	nsDCCweb::loopWebServices();   // constantly check for websocket events

	/*2020-05-03 new approach to broadcasting changes over all channels.
	A change can occur on any comms channel, might be WiThrottle, Websockets or the local hardware UI
	Flags are set in the loco and turnout objects to denote the need to broadcast.
	These flags are cleared by the local UI updater as the last in sequence*/

	nsDCCweb::broadcastChanges();
#endif


#ifdef _WITHROTTLE_h
	nsWiThrottle::broadcastChanges(false);
#endif

	//broadcast turnout changes to line and clear the flags
	updateLocalMachine();

	//call DCCcore once per loop. We no longer use the return value
	DCCcore();


	//2024-08-28 process incoming railcom data
	railcomLoop();



	if (quarterSecFlag) {
		/*isQuarterSecFlag will return true and also clear the flag*/
		quarterSecFlag = false;


		//2021-01-29 call processTimeout every 250mS to give better resolution over timeouts
#ifdef _WITHROTTLE_h
		nsWiThrottle::processTimeout();
#endif

		secCount++;
		if (secCount >= 8) {
			//measure 2 secs
			secCount = 0;
		
		
			//send power status out to web sockets
#ifdef _DCCWEB_h
			nsDCCweb::broadcastPower();
#endif


#ifdef _WITHROTTLE_h
			/*transmit power status to WiThrottles*/
			//2020-11-28 no need to do this so frequently
			//nsWiThrottle::broadcastWiPower();
#endif
		}//end one sec count


	}//qtr sec

}
