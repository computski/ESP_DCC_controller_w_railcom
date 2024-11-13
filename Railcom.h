// Railcom.h
//2024-08-28 JOssowski
//Provides debug info over the websocket and also decodes incoming railcom data

#ifndef _RAILCOM_h
#define _RAILCOM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif




namespace nsRailcom {
static const uint8_t decode[] = {
0b10101100,0b10101010,0b10101001,0b10100101,0b10100011,0b10100110,0b10011100,0b10011010,0b10011001,0b10010101,0b10010011,0b10010110,0b10001110,0b10001101,0b10001011,0b10110001,
0b10110010,0b10110100,0b10111000,0b01110100,0b01110010,0b01101100,0b01101010,0b01101001,0b01100101,0b01100011,0b01100110,0b01011100,0b01011010,0b01011001,0b01010101,0b01010011,
0b01010110,0b01001110,0b01001101,0b01001011,0b01000111,0b01110001,0b11101000,0b11100100,0b11100010,0b11010001,0b11001001,0b11000101,0b11011000,0b11010100,0b11010010,0b11001010,
0b11000110,0b11001100,0b01111000,0b00010111,0b00011011,0b00011101,0b00011110,0b00101110,0b00110110,0b00111010,0b00100111,0b00101011,0b00101101,0b00110101,0b00111001,0b00110011,
0b00001111,0b11110000,0b11100001 };
#define RC_NACK 0x40
#define RC_ACK 0x41
#define RC_BUSY 0x42

	enum RC_STATE
	{
		RC_EXPECT_ID0,   //equivalent to expect ID=0
		RC_EXPECT_B1,
		RC_SUCCESS,
		RC_FAIL,
		RC_TIMEOUT,
		RC_IDLE
	};

	struct RC_MSG {
		uint8_t state;
		uint16_t locoAddr;
		bool    useLongAddr;
		uint8_t	data_ID0;
		uint8_t data_1;
	}static rc_msg;
	

	void railcomInit();
	void railcomLoop(void);
	void readRailcom(uint16_t addr, bool useLongAddr, uint16_t cvReg);

	/*local scope, plus the static structs declared above*/
	static bool decodeRailcom(uint8_t inByte, uint8_t* dataOut);

}

#endif
