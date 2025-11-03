// 
// 
// 

#include "LocoNetprocessor.h"
#include "Global.h"
#include "DCCcore.h"

/*LocoNet
* 
Note: Decoder Pro does not see to recognise it has lost TCP connection and does not attempt a re-initialise of LocoNet.  You have to manually restart DP.

DP does not seem to do anything with the data i am transmitting to it.  It transmits SEND 00 11 etc which is not to spec (no mention of send) so maybe its expecting
some preface word to the returning hex?
https://loconetovertcp.sourceforge.net/Protocol/LoconetOverTcp.html  yeah

this says you first echo the command then transmit SENT OK then
transmit your response.  Yup, now I am seeing stuff in the Monitor LocoNet window

https://loconetovertcp.sourceforge.net/Protocol/SD_blocking_request.svg
https://digi100.synology.me/Startseite.html


*/




using namespace nsLOCONETprocessor;

static std::vector<CLIENTMESSAGE> messages;



void nsLOCONETprocessor::handleLocoNet(void* arg, AsyncClient* client, void* data, size_t len) {
	trace(Serial.printf("\nLOCOnet %s \n", client->remoteIP().toString().c_str());)

		//malloc gives more efficient memory usage than a fixed buffer - remember to use free
		char* buffer;
	buffer = (char*)malloc(len + 1);
	//incoming *data was void, need to cast to const char
	strncpy(buffer, (const char*)data, len);
	buffer[len] = '\0';  //terminate with a null

	//it appears incoming messages may include SEND-space as a prefix and cr multiple times over in 
	//a single message.  Use SEND-space as a token separator.  Look ahead through buffer to find all tokens
	char* ptr = strstr(buffer, "SEND ");
	char* ptrNext;

	while (ptr != nullptr) {
		ptrNext = strstr(ptr + 5, "SEND ");

		if (ptrNext != nullptr) {
			//sneaky; temporarily put a null terminator at ptrNext position
			//so that we don't send the entire rest of string
			ptrNext[0] = '\0';
			tokenProcessor(ptr+5, client);  //send part after "SEND "
			ptrNext[0] = 'S';  //revert to S
		}
		else {
			//last token
			tokenProcessor(ptr+5, client);  //send part after "SEND "
			break;
		}
		ptr = ptrNext;
	}
		
	free(buffer);
}


/// <summary>
/// Loconet token processor. The LocoNet over TCP protocol requries that every incoming loconet SEND XX YY message
/// be echoed out as RECEIVE XX YY followed by SENT OK.  Then the processor can actually process the message content.
/// </summary>
/// <param name="msg">A single SEND message</param>
/// <param name="client">TCP client</param>
void nsLOCONETprocessor::tokenProcessor(char* msg, AsyncClient* client) {
	//its easier to break on spaces using strtok and build a vector of ints
	//and use strtoul just for conversion of a single token rather than iteration of msg
	//https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
	
	std::vector<std::uint8_t> tokens;
	#define BUFSIZE 25
	char buf[BUFSIZE];

	trace(Serial.printf("L: %s\r\n", msg);)
	
//Loconet over TCP requires that we echo the message prefixed by RECEIVE
//and follow this with SENT OK
	std::string m;   
	m.append("RECEIVE ");
	m.append(msg);
	m.append("\nSENT OK\n");
	

	char* tokenSplit = strtok(msg, " ");
	while (tokenSplit != NULL) {
		tokens.push_back(strtoul(tokenSplit, nullptr, 16));
		tokenSplit = strtok(NULL, " ");
	}

	uint8_t checksum;
	//XOR all tokens, should give 0xFF
	for (auto t : tokens)
	{
		checksum ^= t;
	}
	if (checksum != 0xFF) { 
		trace(Serial.printf("chk fail");)
		//if checksum fails, ignore the message
		return;
	}
	
	queueMessage(m, client);
	m.clear();

	/*
	SEND BB 79 01 3C
	SEND BB 00 00 44

	SEND B0 78 27 10
SEND B0 79 27 11
SEND B0 7A 27 12

SEND BF 00 03 43

SEND EF 0E 7C 23 00 00 00 00 00 1C 00 7F 7F 5D
SEND 82 7D


I see a string of BB and B0 initially


	*/
	

	//Two token messages
	if (tokens.size() == 2) {
		switch (tokens[0]) {
		case OPC_IDLE:
			//	FORCE IDLE state, B'cast emerg. STOP
			
			break;
			
		case OPC_GPON:
			power.trackPower = true;
			Serial.println("ON");
			break;

		case PC_GPOFF:
			power.trackPower = false;
			Serial.println("OFF");
		}
		return;
	}

	//Four token messages
	if (tokens.size() == 4) {

		
		switch (tokens[0]) {
		case OPC_SW_REQ:
			// Command a turnout.
			// Immediate response of <0xB4><30><00> if command failed, otherwise no response
			break;

		case OPC_RQ_SL_DATA:
			//request slot data/status block
			Serial.println("RQ");
			queueMessage(FN_OPC_SL_RD_DATA(tokens[1]), client);
			break;

		case OPC_LOCO_ADR:
			//Request loco address and status, if not found master puts address in a free slot.
			//The Wiki page contains an error, token[1] is ADR2 and token[2] ADR
		{//scope block because we declare variables within
			Serial.println("OPC_LOCO_ADR");

			//all bytes within a message are 7bit values (except first OPCODE byte)
			//the address is ADR2 as <7 upward> and ADR as <0-7>


			/* this is not working
			OPC_LOCO_ADR
			findLoc addr=L464 S/L=1
			Out RECEIVE BF 03 50 13

			SENT OK
			RECEIVE E7 0E 01 83 C1 00 20 04 00 D0 FF 3F 20 41 
			RECEIVE E7 0E 01 83 50 00 20 04 00 03 00 00 00 E3  better..but addr is wrong


			
			Monitor LocoNet says [BF 03 50 13]  Request slot for loco address 464. And this gets repeated but it does not respond to the RECEIVE message.
			The XOR check is good.
			
			The throttle must then examine the SLOT READ DATA bytes to work out how to process the
 Master response. If the STATUS1 byte shows the SLOT to be COMMON, IDLE or NEW the throttle
 may change the SLOT to IN_USE by performing a NULL MOVE instruction ,(opcode
<BA>,<slotX>,<slotX>,<chk> )  on this SLOT. This activation mechanism is used to guarantee proper
 SLOT usage interlocking in a multi-user asynchronous environment.
	
			I don't see any BA requests.
			



			[B0 7A 27 12]  Interrogate LocoNet Turnouts/Sensors with bits a/c/b of 1/1/0; addresses...
	49-56, 113-120, 177-184, 241-248, 305-312, 369-376, 433-440, 497-504,
	561-568, 625-632, 689-696, 753-760, 817-824, 881-888, 945-952, 1009-1016,
	1073-1080, 1137-1144, 1201-1208, 1265-1272, 1329-1336, 1393-1400, 1457-1464, 1521-1528,
	1585-1592, 1649-1656, 1713-1720, 1777-1784, 1841-1848, 1905-1912, 1969-1976, 2033-2040.
			*/





			
			if (tokens[1] == 0) {
				//dealing with a short address
				snprintf(buf, BUFSIZE, "S%d", tokens[2]);
			}
			else {
				//dealing with long address
				//token[1] will be ADR2 <7+> msb and token[2] is ADR <0-6> lsb, but we need to decode these values back to 
				uint16_t addr = (tokens[1] <<7) + tokens[2];
				snprintf(buf, BUFSIZE, "L%d", addr);
			}

			//find slot containing the loco.  LocoNet slots 0-120 are for locos.  We shall just use the index of our loco array
			int8_t locoNetSlot = findLoco(buf, nullptr, false);
			if (locoNetSlot == -1) {
				//no free slots, send OPC_LONG_ACK 0xB4 0xBF 0x00 0xF4
				queueMessage("RECEIVE 0xB4 0xBF 0x00 0xF4", client);
				return;
			}

			//have a slot.  Respond with OPC_SL_RD_DATA
			//0xE7 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2 CHK
			queueMessage(FN_OPC_SL_RD_DATA(locoNetSlot),client);
			break;
		}//end scope block

		case OPC_MOVE_SLOTS:
			//a NULL MOVE (i.e. source slot= dest slot) from the throttle is a request to take ownership of that loco
			//we respond with OPC_SL_RD_DATA

			
		case OPC_WR_SL_DATA:
			//this is how the throttle sends messages to locos


			break;
		}

	
		return;
	
	}//end 4 token

	//variable token messages



}

//std::vector<std::uint8_t> tokens;
std::string nsLOCONETprocessor::echoRequest(std::vector<std::uint8_t> tokens) {

	return "X";
}


/// <summary>
/// generate a string containing OPC_SL_RD_DATA response
/// </summary>
/// <param name="locoSlot">slot position in loco[]</param>
/// <param name="CV17">low address</param>
/// <param name="CV18">high address</param>
/// <returns>OPC_SL_RD_DATA hex string</returns>
std::string nsLOCONETprocessor::FN_OPC_SL_RD_DATA(int8_t locoSlot) {
	/*note: i thought i had a means to flag a loco that is in use by a Wit. where is this, as its not on the loco object?
	* maybe its done by comparing WiT client info?  after all the only other input source is the HUI
	* 
	* locoNet has an inUse flag.
	*/

	std::string m;
	//default failure message OPC_LONG_ACK, ACK1=0x00

	m.append("RECEIVE B4 BF 00 F4\n"); 
	if (locoSlot < 0) return m;
	if (locoSlot >= MAX_LOCO) return m;
	m.clear();	
	m.append("RECEIVE ");
	
	
	//response 0xE7 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2 CHK

	//RECEIVE E7 0E 01 83 C1 00 20 04 00 D0 FF 3F 20 41 was garbage
	//RECEIVE E7 0E 01 83 50 00 20 04 00 03 00 00 00 E3 correct 50 ADR and 03 ADR2  addr=464
	//RECEIVE E7 0E 00 83 03 00 20 04 00 00 00 00 00 B2   addr = 03

	//but still the client is not sending 

	/*
	The throttle must then examine the SLOT READ DATA bytes to work out how to process the
 Master response. If the STATUS1 byte shows the SLOT to be COMMON, IDLE or NEW the throttle
 may change the SLOT to IN_USE by performing a NULL MOVE instruction ,(opcode
<BA>,<slotX>,<slotX>,<chk> )  on this SLOT. This activation mechanism is used to guarantee proper
 SLOT usage interlocking in a multi-user asynchronous environment.
	
	NOW it is working. Am seeing <BA> stuff.


	*/
			


	uint8_t returnArray[13] = {0};   //force initialisation to zeros 
	char buf[5];

	returnArray[0] = OPC_SL_RD_DATA;
	returnArray[1] = 0x0E;
	returnArray[2] = locoSlot;
	returnArray[3] = 0b00000011;  //more work to do, especially on D4,5
	
	//loconet does not have a concept of short/long addresses, instead it takes a 14 bit address
	//and splits this over two 7 bit bytes.

	returnArray[4] = loco[locoSlot].address & 0x7F;  //lower 7 bits for ADR
	returnArray[9] = loco[locoSlot].address >> 7;  //shift bits 8+ to zero for ADR2

	/*
	if (loco[locoSlot].useLongAddress) {
		returnArray[9] = loco[locoSlot].address ^ 256;  //cv18
		returnArray[4] = (loco[locoSlot].address >> 8) + 192;  //cv17
	}
	else {
		//pass lower 8 bits as ADR and 0 for ADR2
		returnArray[4] = loco[locoSlot].address & 0x7F;  //cv17
	}
	*/

	returnArray[5] = loco[locoSlot].speedStep;
	uint8_t DIRF = loco[locoSlot].forward ? 0b00100000 : 0x00;
	DIRF += (loco[locoSlot].function >> 1) & 0b1111;
	DIRF += (loco[locoSlot].function << 4) & 0b00010000;
	returnArray[6] = DIRF;
	returnArray[7] = power.trackPower ? 0b00000101 : 0b00000100;
	//set <1> if all locos are at rest (aftermath of eStop)
	bool Allstop = true;
	for (auto l : loco) {
		if (l.address == 0) continue;
		if (l.speed != 0) Allstop = false;
	}
	if (!Allstop) returnArray[7] += 0b10;

	returnArray[8] = 0x00; //no idea what SS2 is
	//SND ID1 and ID2 are all 0x00;
	uint8_t checkSum = 0xFF;
	for (int i = 0;i < 13;i++) {
		snprintf(buf, 5, "%02X ", returnArray[i]);
		m.append(buf);
		checkSum ^= returnArray[i];
	}
	snprintf(buf, 5, "%02X ", checkSum);
	m.append(buf);
	m.append("\n");

	return m;

}




void nsLOCONETprocessor::buildBroadcastQueue(bool clearQueue) {
	if (clearQueue) {
		messages.clear();
		return;
	}
}

/// <summary>
/// Transmit queued messages to a specific client. This will also transmit any broadcast messages
/// that have been queued up.  It will iterate the message queue.
/// </summary>
/// <param name="client">specific client to send to, cannot be nullptr</param>
void nsLOCONETprocessor::sendToClient(AsyncClient* client) {
	if (messages.empty()) return;
	if (client == nullptr) return;

	std::string outBoundMessage;
	outBoundMessage.clear();

	//build the outBoundMessage from client specific messages plus any broadcast messages
	for (auto m : messages) {
		if ((m.toClient == nullptr) || (m.toClient == client)) {
			outBoundMessage.append(m.msg);
		}
	}

	//find specific client messages or nullptr broadast, and send
	//const char *data = s.c_str(); will cause a crash if you use it to call client->add
	//need to copy the data to a new array

	if (outBoundMessage.size() > 0) {
		char* data = new char[outBoundMessage.size() + 1];
		copy(outBoundMessage.begin(), outBoundMessage.end(), data);
		data[outBoundMessage.size()] = '\0';

		//send over TCP/IP
		if (client->space() > sizeof(data) && client->canSend()) {
			client->add(data, strlen(data));
			client->send();
			trace(Serial.printf("Out %s\r\n",data);)
		}

		//we used new to create *data.  delete now else you create a memory leak
		delete data;

		//2025-11-02 now delete any client-specific messages (leave null client ones in the messages vector)
		//means they are only transmitted once
		for (auto it = messages.begin(); it != messages.end(); ) {
			if (it->toClient == client) {
				it = messages.erase(it); // Erase and update iterator
			}
			else {
				++it; // Move to the next element
			}
		}
	}


}
void nsLOCONETprocessor::broadcastSMreadResult(uint16_t cvReg, int16_t cvVal) {}


/// <summary>
/// queue a message for a specific client, or all
/// </summary>
/// <param name="s">standard string containing message</param>
/// <param name="client">spcific client, or all if nullptr</param>
void nsLOCONETprocessor::queueMessage(std::string s, AsyncClient* client) {
	CLIENTMESSAGE m;
	m.toClient = client;
	m.msg = s;
	messages.push_back(m);
}

void nsLOCONETprocessor::setPower(bool powerOn);