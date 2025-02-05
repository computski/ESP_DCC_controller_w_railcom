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
	
	void IRAM_ATTR dcc_init(uint32_t pin_dcc, uint32_t pin_enable, bool phase, bool enableAsHigh, bool enableActiveDuringCutout);


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
	* a weak pullup, so we need to put a stronger pull-down on this, say 1k and we should be good..
	* if PIN_ESTOP is left undefined, then it can be assigned via PIN_RAILCOM_SYNC
	* Can it be used for both?  Answer is yes, because outside of a railcom cutout, we can breiefly re-purpose the pin as an input, read it and if we see low, trigger estop.
	* pulling it low will need to be via a resistor so it does not damage the pin when used as a railcom sync output.  and also it won't damage the 6n137.
	* But that said, we would need to use a PNP pulldown off GPIO15 and into the 6n317 enable, because we'd need to repurpose the GP to be WPU and a 1k res direcly on the pin to ground
	* would probably override this, hence the need for a PNP isolator.  Actually 2k2 works, 5k1 does not.  so maybe internal WPU and 6n WPU would be enough to overcome this? the mini itself has a 12k
	* pulldown and this does not get in the way of the ESP WPU.
	* 
	* pinSyncInputTriggered is a flag and is set if we saw active low input on PIN_RAILCOM_SYNC_INPUT at the end of the railcom cutout. This flag must be reset in software.
	* this allows us to dual-purpose that pin for both opto-isolator gating into the UART, and as an input
	* Note: i might flow this trigger through to the keypad struct, this way it becomes a virtual key
	* 
	*/


#endif
