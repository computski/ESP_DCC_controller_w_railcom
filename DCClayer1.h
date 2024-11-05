// DCClayer1.h
// 2021-10-14 modified to support DC pwm operation, i.e. non DCC. In this mode it responds only to loco 3
// 2021-12-17 simplified dcc_init() and dc_init()
// DC mode is selected in the .INO setup routine
// 2023-09-24 added Railcom cutout

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
		bool  railcomCutoutActive;  
	};

	/*2023-08-30 doCutout will add a railcom cutout at end of packet if true*/
	/*2024-11-05 added railcomCutoutActive flag to signal back to main routine that the cutout is in progress*/

	/*2021-11-25 fastTickFlag added, this runs at 1mS and is used for analog detection of ACK pulse in service mode*/

	extern volatile DCCBUFFER DCCpacket;
	

	void IRAM_ATTR dcc_init(uint32_t pin_dcc, uint32_t pin_enable, bool phase, bool invertEnable);
	void IRAM_ATTR dcc_init_LMD18200(uint32_t pin_pwm, uint32_t pin_dir, uint32_t pin_brake);
	void IRAM_ATTR dc_init(uint32_t pin_pwm, uint32_t pin_dir, bool phase, bool invert);

#endif
