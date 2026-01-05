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

//https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
// 
// 2 byte message opcodes
#define OPC_IDLE	0x85
#define OPC_GPON	0x83
#define PC_GPOFF	0x82
#define OPC_BUSY	0x81

//4 byte message opcodes
#define	OPC_LOCO_ADR	0xBF
#define OPC_SW_ACK		0xBD
#define OPC_SW_STATE	0xBC
#define OPC_RQ_SL_DATA	0xBB
#define OPC_MOVE_SLOTS	0xBA
#define OPC_LINK_SLOTS	0xB9
#define OPC_UNLINK_SLOTS	0xB8
#define OPC_CONSIST_FUNC	0xB6
#define OPC_SLOT_STAT1	0xB5
#define OPC_LONG_ACK	0xB4
#define OPC_INPUT_REP	0xB2	
#define OPC_SW_REP		0xB1
#define OPC_SW_REQ		0xB0
#define OPC_LOCO_SND	0xA2
#define OPC_LOCO_DIRF	0xA1
#define OPC_LOCO_SPD	0xA0

//variable length message opcodes
#define OPC_WR_SL_DATA 0xEF
#define OPC_SL_RD_DATA 0xE7










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
//	void buildBroadcastQueue(bool clearQueue);
	void sendToClient(AsyncClient* client);
//	void broadcastSMreadResult(uint16_t cvReg, int16_t cvVal);
	void handlerLocoNet(bool ack, uint16_t cvReg, uint8_t cvVal);

	//internal scope
	static void queueMessage(std::string s, AsyncClient* client);
	static void setPower(bool powerOn);
	static std::string FN_OPC_SL_RD_DATA(int8_t locoSlot);
	//static std::string echoRequest(std::vector<std::uint8_t> tokens);	
	static void writeDIRF_SPD(uint8_t* dirf, uint8_t* spd, void* loc);
	static void* getSytemSlotPtr(uint8_t locoNetSlot);
	static void writeProgrammerTaskFinalReply(void);
}


/*
09:56:36.912: [EF 0E 7C 2F 00 01 54 00 00 01 00 7F 7F 19]  Byte Read on Main Track (Ops Mode): Decoder address 212: CV2.
09:56:36.918: [E7 0E 7C 2F 00 01 54 05 00 01 00 00 00 14]  Programming Response: Byte Read on Main Track (Ops Mode) Was Successful: Decoder address 212: CV2 value 0 (0x00, 00000000b).
final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
pcmd=2f = 0b10 1111  <w/r> <byte/bit> <ty1> <ty0> <ops/sm> <res> <res>
pstat=0 and right now code just echos that
hopsa=01
lopsa=54

in ops mode, ty0 is always 0, ty1 =0 no feedback, 1=feedback   but this makes no sense for reads because the spec does not provide for ACK on read.  But, if we see AC in the response codes
from the loco then the data would be valid.


*/


#endif

