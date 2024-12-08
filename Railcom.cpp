// 
// 
// 
#include "Global.h"
#include "Railcom.h"
//dependencies
#include "DCCweb.h"
#include "DCClayer1.h"
#include "DCCcore.h"

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

//we need a callback to DCCcore, the railcom value has to update itself there somehow...



using namespace nsRailcom;

static void buildHexString(char* hex);
static JsonDocument rcOut;

#define nDEBUG_RC










/// <summary>
/// Debug. Continuously monitors the serial port for 2-byte datagrams and outputs these over a websocket when seen.
/// Use: ensure you disable StartRailcom with an immediate return clause.
/// </summary>
void nsRailcom::railcomLoopDebug(void) {
	
	//DEBUG USE
	uint8_t byteNew;
	uint8_t	byteCount = 0;

	
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

					rcOut["type"] = "railcom";
					char hex[RC_BUFF_LEN + 1];
					//char hex[22];
				buildHexString(hex);
					rcOut["hex"] = hex;
					rcOut["payload"] = rc_msg.payload;
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



