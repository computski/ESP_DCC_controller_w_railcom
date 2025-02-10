// DCClayer1.h
// 2021-10-14 modified to support DC pwm operation, i.e. non DCC. In this mode it responds only to loco 3
// 2021-12-17 simplified dcc_init() and dc_init()
// DC mode is selected in the .INO setup routine
// 2023-09-24 added Railcom cutout
// 2024-12-12 added Railcom decoder into layer 1

//2025-01-13 BUG/ LIMITATION.  If a loco is not in the roster, then reading from it via POM is less reliable and often requires several read attempts.
//The reason for this is the POM command is repeated only twice (I think) and subsequently there are no more railcom cutout slots which address the target loco.
//Decoders respond to cv requests in railcom channel 2 which is a request-response format and requires the loco to specifically be addressed.
//To fix this we'd need to send a few loco specific packets, perhaps a stop command.  But we'd need to specially inject these into the DCCbuffer.  Or perhaps we can repeat the
//read command more times if we are aware that the loco is not in the roster.

#ifndef _DCCLAYER1_h
#define _DCCLAYER1_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



/*longest DCC packet is 6 bytes when using POM and long address*/
/*2021-10-19 trackPower now controlled from DCClayer1*/
	struct DCCBUFFER {
		uint8_t data[6];
		uint8_t packetLen;
		bool  clearToSend;  
		bool  longPreamble;
		bool  msTickFlag; 
		bool  trackPower;
		bool  fastTickFlag;
		bool  doCutout;
		bool  pinSyncInputTriggered;  //if PIN_RAILCOM_SYNC_INPUT used, this flag goes high if the input was triggered (active low)
		uint8_t railcomPacketCount;  //used for railcom timeout
	};

	/*2023-08-30 doCutout will add a railcom cutout at end of packet if true*/
	/*2021-11-25 fastTickFlag added, this runs at 1mS and is used for analog detection of ACK pulse in service mode*/

	/*Note: during the railcom cutout, the loco decoder will send data to this unit. This data will be captured in the serial buffer, but due to the overheads
	in the arduino stack, the serial data becomes avaialble after, and asyncrhonously to, the actual cutout that invoked it*/


	extern volatile DCCBUFFER DCCpacket;
	
	//void IRAM_ATTR dcc_init(uint32_t pin_dcc, uint32_t pin_enable, bool phase, bool enableAsHigh, bool enableActiveDuringCutout);
	void IRAM_ATTR dcc_init(uint32_t pin_dcc, uint32_t pin_enable, bool phase);


	//railcom related
	void railcomInit();
	void railcomLoop(void);
	void railcomRead(uint16_t addr, bool useLongAddr, uint8_t reg);

	static bool decodeRailcom(uint8_t inByte, uint8_t* dataOut, bool ignoreControlChars);
	static bool decodeRailcom(uint8_t* inByte, bool ignoreControlChars);

	/*2024-12-12 a note on railcom.  The railcom spec says serial data should be asserted by the decoder around 30uS into the railcom cutout.  The arduino stack does not allow
	* us to directly manipulate the serial port.  It reads incoming serial into a buffer but there are variable latencies on when this data appears, so it is not possible for the
	* software to know if the data in the buffer came from a specific railcom cutout.  Furthermore, we are constantly reading the buffer analysing and mostly ignoring
	* its content.  There is quite a lot of junk data from noise appearing in the buffer and this can actually upset the decoding of valid data if a valid 4/8 byte is read but actually
	* is junk data and not part of a message.   We can improve this situation by gating pin 7 of the 6N137 with the railcom sync signal.  This means we only present serial data to be 
	* ESP serial port during the cutout periods.  This greatly improves the reliabilty of reads.
	* D8 GPIO15 is used as the railcom sync pin and normally this would be estop (pull to ground). However, it must be low at boot.  Not a problem because pin 7 on the 6n137 has 
	* a weak pullup, so we need to put a stronger pull-down on this, say 2k2.
	* 
	* We have two options
	* PIN_RAILCOM_SYNC
	* D8 GPIO15 will be a totem pole output.  We need 2k2 to ground on this pin, because the on-module 12k pulldown is not enough to overcome the 6n137 Enable pin's WPU which is
	* in the 5k-7k range, and remember it pulls to 5v as the opto chip is given a 5v supply.
	* In this mode, D8 will go high when we want to gate-in the railcom serial data 
	*  
	* PIN_RAILCOM_SYNC_INPUT
	* A pushbutton is connected via 680R to D8 and pulls it to 3v3 when active.
	* D8 drives an NPN 2N3904 via a 1k base resistor.  The collector is connected to the 6N137 Enable pin.  Emitter to ground.
	* During railcom blanking period, D8 is driven high. This masks out the opto and the pushbutton.
	* D8 is driven low and is held low for the entire RC cutout except at the end, where it is breifly switched to a regular INPUT
	* and is read.  It will read high if the pushbutton is active.  When D8 is active low, the pushbutton has no effect and cannot corrupt incoming serial.
	* 
	* pinSyncInputTriggered is a flag and is set if we saw active high input on PIN_RAILCOM_SYNC_INPUT at the end of the railcom cutout. This flag must be reset in software.
	* 
	*/


#endif
