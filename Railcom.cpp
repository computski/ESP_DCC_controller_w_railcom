// 
// 
// 
#include "Global.h"
#include "Railcom.h"
//dependencies
#include "DCCweb.h"
#include "DCClayer1.h"

/*debug notes
* we see bytes if JP1 is in place.  if JP1 is removed, the bytes stop.
* Its probable that noise will exist during the RC cutout that serial will interpret as data
* There is no way to flush/reset the receive buffer in Arduino other than to read all data.   There are supposedly datagrams in the railcom protocol
* so we should be able to extract valid data from what might appear to be an async stream of random stuff.
*
*
* DCCpacket.railcomCutoutActive is a flag generated by DCClayer1, it is true during the railcom cutout and so allows the main routine to 'gate' the serial data
* against the loco who's packet we are processing.  We need to be aware of when we update DCCpacket with new data vs when we read the serial port.
*
*
* Rules
* we can read/write to an address that may not even be in the roster.  for this reason we don't index LOCO
* if reading from a non roster loco, the only way we will see rc data from that loco is to keep sending it POM read packets for 0.5sec, because the regular loco packets won't
* every transmit to the target loco address
* we timeout after 80 packets, this is approx 0.5s
*
* 
* it appears we read the cv twice, possibly because we send 4 dcc-read-pom commands so normally you'd initiate read, expect a value, return that over a WS but 
* there might be a second repeat value come over, and you want to decode it, but also ignore it.... or is it sufficient to just enable 'look for id0?'
* we seem to have fixed the weird problem i saw before where reading a new cv generates the value of the prior cv read.
* 
* Bugs 2024-11-17.  cannot read cv5. we see ok over WS, but then nothing, no timeout, no WS messages.  on the line we can see the decder responds with
* AAAC which is valid, but still we see no response.  expected value is 64d
* I found that if I reversed the track polarity, I can reliably read cv5.   This is confusing because I could see it on the PicoScope but perhaps its just enough
* out of spec that the system cannot read it.  But this does not explain why it hangs and never sends a WS response.
* 
* wierd with the L298, the ESU cv8 consistently reads as 134 (wrong) or, if polarity is reversed it reads 151 (correct).  The PICO scope correctly decodes as A969
* but the ESP must be reading it differently.  but how?  the edges look about the same, and its consistently reading two 4/8 codes to get 134.
* 
*/




using namespace nsRailcom;

static void buildHexString(char* hex);
static JsonDocument rcOut;

#define nDEBUG_RC



const uint8_t decode[] = {
0b10101100,0b10101010,0b10101001,0b10100101,0b10100011,0b10100110,0b10011100,0b10011010,0b10011001,0b10010101,0b10010011,0b10010110,0b10001110,0b10001101,0b10001011,0b10110001,
0b10110010,0b10110100,0b10111000,0b01110100,0b01110010,0b01101100,0b01101010,0b01101001,0b01100101,0b01100011,0b01100110,0b01011100,0b01011010,0b01011001,0b01010101,0b01010011,
0b01010110,0b01001110,0b01001101,0b01001011,0b01000111,0b01110001,0b11101000,0b11100100,0b11100010,0b11010001,0b11001001,0b11000101,0b11011000,0b11010100,0b11010010,0b11001010,
0b11000110,0b11001100,0b01111000,0b00010111,0b00011011,0b00011101,0b00011110,0b00101110,0b00110110,0b00111010,0b00100111,0b00101011,0b00101101,0b00110101,0b00111001,0b00110011,
0b00001111,0b11110000,0b11100001 };
#define RC_NACK 0x40
#define RC_ACK 0x41
#define RC_BUSY 0x42

#define RC_BUFF_LEN 16

struct RC_MSG {
	uint8_t state;
	uint16_t locoAddr;
	bool    useLongAddr;
	uint8_t payload;
	bool	isValid;
#ifdef DEBUG_RC
	uint8_t buffer[RC_BUFF_LEN];
	uint8_t bufferIdx;
#endif
}rc_msg;

enum RC_STATE
{
	RC_EXPECT_ID0,
	RC_EXPECT_BYTE,
	RC_SUCCESS,
	RC_TIMEOUT,
};



/// <summary>
/// Initialise UART for railcom, start listening for incoming data
/// </summary>
void nsRailcom::railcomInit() {
	Serial.println(F("\n\nEnable railcom"));
	Serial.flush();

	readRailcom(0, false);

	Serial.end();
	//railcom uses 250kbaud
	Serial.begin(250000);


	rc_msg.state = RC_EXPECT_ID0;

	
}

//2024-11-26 bug. It appears the serial routine is getting overloaded and crashing the ESP.  if you unplug the serial jump, it is stable
//at the moment you plug it back in, the ESP will crash.  Just touching R5 outside leg is enough to crash the ESP.  Touching the resistor adds a ton
//more edges to the comparator signal and I think this is overloading the serial routine.
//but surely the serial chip will just go into overflow/ frame error etc and it should be able to recover.  maybe the arduino handler causes it to raise an int
//and this is overloading the system?
// https://forum.arduino.cc/t/serial-port-input-causes-arduino-to-crash/653103
//TEST: do nothing ignore serial.  touch R5 and you see lots of bogus info but it does not crash the ESP.  which means that either the act of reading it
//or my processing thereafter causes a crash



/// <summary>
/// Debug use. Continuously monitors the serial port for 2-byte datagrams and outputs these over a websocket when seen.
/// Use: ensure you disable StartRailcom with an immediate return clause.
/// </summary>
void nsRailcom::railcomLoopDebug(void) {
	
	//DEBUG USE
	uint8_t byteNew;
	uint8_t	byteCount = 0;

	/* this test works, no crash.  so its the processing itself that must be causing some sort of blocking/timeout
	while (Serial.available() > 0) {
		byteNew = Serial.read();
		if (byteCount++ > 20) return;
	}
	return;
	*/


	//nope, code below crashes it if you touch R5.  bizarre. lets disable the buffering
	while (Serial.available() > 0) {
		//saftey valve,  process max 20 bytes before giving control back to main loop
		if (byteCount++ > 20) return;

		// read the incoming byte:
		byteNew = Serial.read();

#ifdef DEBUG_RC
		//buffer was causing a crash, fix with ++rc_msg.bufferIdx not post increment
		if (++rc_msg.bufferIdx == RC_BUFF_LEN) rc_msg.bufferIdx = 0;
		rc_msg.buffer[rc_msg.bufferIdx] = byteNew;
#endif
			switch (rc_msg.state) {
			case RC_EXPECT_BYTE:
				if (decodeRailcom(&byteNew,true)) {
					rc_msg.payload += (byteNew & 0b00111111);
					//repeated calls seem to crash the ESP
					//https://arduinojson.org/v6/issues/memory-leak/
					
					//we might work around this for POM reads by gating the request with the timeout

					//The ESU decoder definately outputs junk values before finally sending the correct CV

					rcOut["type"] = "railcom";
					char hex[RC_BUFF_LEN + 1];
					//char hex[22];
				buildHexString(hex);
					rcOut["hex"] = hex;
					rcOut["payload"] = rc_msg.payload;
					rcOut["flag"] = DCCpacket.railcomCutoutActive;
					nsDCCweb::sendJson(rcOut);
					rcOut.clear();
					
				}
				rc_msg.state = RC_EXPECT_ID0; 
				break;

			case RC_SUCCESS:
			break;
			

			default:
				if (decodeRailcom(&byteNew,true)) {
					if ((byteNew & 0b00111100) == 0) {
						//found ID0
						rc_msg.payload = byteNew << 6;
						rc_msg.state = RC_EXPECT_BYTE;
						break;
					}
				}
				rc_msg.state = RC_EXPECT_ID0;
				break;
				
			}
	
		
	}
	
}

/// <summary>
/// Call once per program loop
/// </summary>
void nsRailcom::railcomLoop(void) {
#ifdef DEBUG_RC
	railcomLoopDebug();
	return;
#endif 

	uint8_t byteNew;
	uint8_t	byteCount = 0;

	while (Serial.available() > 0) {
		// read the incoming byte:
		byteNew = Serial.read();

		switch (rc_msg.state) {
		case RC_EXPECT_BYTE:
			if (decodeRailcom(&byteNew,true)) {
				rc_msg.payload += (byteNew & 0b00111111);
				rcOut["type"] = "dccUI";
				rcOut["cmd"] = "railcom";
				rcOut["payload"] = rc_msg.payload;
				rcOut["count"] = DCCpacket.railcomPacketCount;
				rcOut["flag"] = DCCpacket.railcomCutoutActive;
				nsDCCweb::sendJson(rcOut);
				rcOut.clear();
				rc_msg.isValid = true;
			}
			//stop looking for incoming messages
			//2024-11-26.  Revision.  DO keep looking for incoming messages because the ESU decoder I have sends junk values before it finally sends the correct one
			//no idea why.  it does not implement the RC protocol per spec

			//rc_msg.state = RC_SUCCESS;
			rc_msg.state = RC_EXPECT_ID0;
			break;

		case RC_SUCCESS:
		case RC_TIMEOUT:
			break;


		default:
			if (decodeRailcom(&byteNew,true)) {
				if ((byteNew & 0b00111100) == 0) {
					//found ID0
					rc_msg.payload = byteNew << 6;
					rc_msg.state = RC_EXPECT_BYTE;
					break;
				}
			}
			rc_msg.state = RC_EXPECT_ID0;
			break;

		}

		//saftey valve,  process max 20 bytes before giving control back to main loop
		if (++byteCount > 20) break;
	}


	//timed out?
	
	if (DCCpacket.railcomPacketCount == 0) {
		switch (rc_msg.state) {
		case RC_SUCCESS:
		case RC_TIMEOUT:
			break;
		
		default:
			//send this message ONCE
			//2024-11-26 if we receive multiple returns from the decoder, we will timeout and at this point do not send the ??? payload
			if (rc_msg.isValid == false) {
				JsonDocument out;
				out["type"] = "dccUI";
				out["cmd"] = "railcom";
				out["payload"] = "???";
				nsDCCweb::sendJson(out);
			}
			rc_msg.state = RC_TIMEOUT;
		}
	}


}









/*we need a railcomPacketEngine
resets on bad data
buffers incoming valid bytes
is aware of ACK/ NACK
is aware of which loco we are reading
and holds last valid read from this
is also reset from external trigger, i.e. when read is initiated

don't intend to use it to read loco ids from ch1

*/

/// <summary>
/// Reset railcom reader, and look for incoming data for locoIndex
/// </summary>
/// <param name="locoIndex"></param>
void nsRailcom::readRailcom(uint16_t addr, bool useLongAddr) {
	rc_msg.locoAddr = addr;
	rc_msg.useLongAddr = useLongAddr;
#ifdef DEBUG_RC
	return;
#endif 
	rc_msg.state = RC_EXPECT_ID0;  
	rc_msg.isValid = false;
	DCCpacket.railcomPacketCount = 80;

	
}


bool nsRailcom::decodeRailcom(uint8_t inByte, uint8_t* dataOut, bool ignoreControlChars) {
	for (int i = 0; i <= RC_BUSY; i++) {
		if (inByte == decode[i]) {
			//valid
			*dataOut = i;
			return true;
		}
		if (ignoreControlChars && (i >= 0x3F)) return false;
	}
	//didn't find a match
	*dataOut = 0;
	return false;
}

/// <summary>
/// decode inbound data against the 4/8 decode table, overwriting inByte with its decoded value
/// </summary>
/// <param name="inByte">4/8 coded serial data inbound, overwritten with decoded value</param>
/// <param name="ignoreControlChars">ignore ACK, NACK, BUSY and only return data values</param>
/// <returns>true if decode successful</returns>
bool nsRailcom::decodeRailcom(uint8_t *inByte,bool ignoreControlChars) {
	for (int i = 0; i <= RC_BUSY; i++) {
		if (*inByte == decode[i]) {
			//valid
			*inByte = i;
			return true;
		}
		if (ignoreControlChars && (i >= 0x3F)) return false;
	}
	//didn't find a match
	
	return false;

}


/// <summary>
/// debug. build a hex string of all received characters up to bufferIdx
/// </summary>
/// <returns>pointer to hexstring</returns>
void buildHexString(char* hex){
	memset(hex, '\0', sizeof(hex));
	uint8_t x = 0;

char* ptr = hex;
#ifdef DEBUG_RC

	for (uint8_t n = rc_msg.bufferIdx+1; n < RC_BUFF_LEN; n++) {
		if (x > sizeof(hex)) return;
		sprintf(ptr, "%02X", rc_msg.buffer[n]);
		ptr += 2;
		x += 2;
	}

	for (uint8_t n = 0;n<= rc_msg.bufferIdx; n++) {
		if (x > sizeof(hex)) return;
		sprintf(ptr, "%02X", rc_msg.buffer[n]);
		ptr += 2;
		x += 2;
		
	}
#endif
}



