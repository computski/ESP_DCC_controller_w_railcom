// 
// 
// 
#include "Global.h"
#include "Railcom.h"
//dependencies
#include "DCCweb.h"

/*debug notes
* we see bytes if JP1 is in place.  if JP1 is removed, the bytes stop.
* Its probable that noise will exist during the RC cutout that serial will interpret as data
* There is no way to flush/reset the receive buffer in Arduino other than to read all data.   There are supposedly datagrams in the railcom protocol
* so we should be able to extract valid data from what might appear to be an async stream of random stuff.
*
*/




using namespace nsRailcom;




/// <summary>
/// Initialise UART for railcom, start listening for incoming data
/// </summary>
void nsRailcom::railcomInit() {
	Serial.println(F("\n\nEnable railcom"));
	Serial.flush();

	

	Serial.end();
	//railcom uses 250kbaud
	Serial.begin(250000);




}

/// <summary>
/// Call once per program loop
/// </summary>
void nsRailcom::railcomLoop(void) {
	static uint8_t byteNew;
	static uint8_t byteOld;


	//check for serial.available and if so decode it and send it over the websocket

	/*
		JsonDocument out;
		out["type"] = "railcom";
		out["payload"] = "test";
		nsDCCweb::sendJson(out);
	*/

		if (Serial.available() > 0) {
			// read the incoming byte:
			byteNew = Serial.read();
			
			if ((byteOld == 0xAC) && (byteNew == 0x95)) {
				//saw 0xAC95 sequence

				JsonDocument out;
				out["type"] = "railcom";
				out["payload"] = "0xAC95";
				nsDCCweb::sendJson(out);

			}
			byteOld = byteNew;


			

			// say what you got:
			//Serial.print("I received: ");
			//Serial.println(incomingByte, DEC);
		}




}