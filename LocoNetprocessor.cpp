// 
// 
// 

#include "LocoNetprocessor.h"
#include "Global.h"
#include "DCCcore.h"

/*TO DO
* loss of loconet client should cause all slots to go to zero speed, and and Status to Common.


servic mode  - initiate write
L: EF 0E 7C 6B 00 00 00 00 00 00 03 7F 7F 0A
Out RECEIVE EF 0E 7C 6B 00 00 00 00 00 00 03 7F 7F 0A

POM - initiate
L: EF 0E 7C 6B 00 00 00 00 00 00 03 7F 7F 0A
Out RECEIVE EF 0E 7C 6B 00 00 00 00 00 00 03 7F 7F 0A
 <0xEF>,<0E>,<7C>,<PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>

 Service mode - initiate read, note that JMRI won't send anything if you ask to read when POM
 diffrence is PCMD is 2B = 0010 1011
 L: EF 0E 7C 2B 00 00 00 00 00 00 03 7F 7F 4A

 I had expected differing PCMD. 0x6B=0110 1011
 D7-0
 D6 1=w 0 =r
 D5 1= byte mode, 0=bit
 D4 ty1
 D3 ty 0
 D2 1 =ops on mainlines, 0=prog track
 D1 =res
 D0 =res

 EF 0E 7C 6B 00 00 00 00 00 1C 06 7F 7F 13   - this is a ready from CV29 (i.e. 1C=128d coded)



*/



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

https://www.digitrax.com/tsd/glossary/c/
Note on InUse.
loco slots don't have a flag for this
WiT throttles are associated with a locoSlot and manage steal on this basis.
Loconet-TCP is effectively another throttle to WiT but it might control multiple loco slots.  I think the easiest way to control this is add
a flag to the loco struct

Note: i am not sure if HUI changes to a slot speed will be reflected back to loconet and be picked up by the WiT hanging off the JMRI server.  YES it is

Note: locoNet locos are in slots 1-127, so we need to offset by one to make use of loco[].  Slot 0 is used for dispatch which we don't need.
to release a loco, locoNet writes 0x13 to STATUS1. 0b0001 0011. loco not consisted and common.

Note: using locoNet will overwrite the roster that is stored in the ESP-DCC.

Note: LocoNet asks for control of a loco address. If the Master responds that the loco is in use, then the client is supposed not to ask to use it!  JRMI seems to
go ahead and flag that loco for inUse a second time.  Not true.  this is an aberration of ED.  e.g. ED asks for loco 464 and you see this executed on LocoNet.
if you use ED to request 464 a second time then no loconet traffic is generated.  ED is just doubling the 464 loco within its own app.

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
	
	LOCO* locoPtr=nullptr;  //pointer to the actual system loco slot (offset by 1 from LocoNet slot)
	

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


		case OPC_RQ_SL_DATA:
			//request slot data/status block
			//tokens[2], if non zero is a request for expanded slot format.  I don't have the specification for this.
			queueMessage(FN_OPC_SL_RD_DATA(tokens[1]), client);
			break;

		case OPC_LOCO_ADR:
			/*Request loco address and status, if not found master puts address in a free slot.
			The Wiki page contains an error, token[1] is ADR2 and token[2] ADR
			The throttle must then examine the SLOT READ DATA bytes to work out how to process the Master response.
			If the STATUS1 byte shows the SLOT to be COMMON, IDLE or NEW the throttle
			may change the SLOT to IN_USE by performing a NULL MOVE instruction <BA>,<slotX>,<slotX>,<chk> on this SLOT.
			This activation mechanism is used to guarantee proper SLOT usage interlocking in a multi-user asynchronous environment.
		*/

		{//scope block because we declare variables within

			/*
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
				uint16_t addr = (tokens[1] << 7) + tokens[2];
				snprintf(buf, BUFSIZE, "L%d", addr);
			}

			//find slot containing the loco.  LocoNet slots 0-120 are for locos.  We shall just use the index of our loco array
			int8_t systemSlot = findLoco(buf, nullptr, false);

			if (systemSlot == -1) {
				//no free slots, send OPC_LONG_ACK 0xB4 0xBF 0x00 0xF4
				queueMessage("RECEIVE 0xB4 0xBF 0x00 0xF4", client);
				return;
			}

			//have a slot.  Respond with OPC_SL_RD_DATA
			//BUT if we just allocated or bumped a slot then loco[] object itself won't have its address set.  We need to do this now
			locoPtr = &loco[systemSlot];
			locoPtr->useLongAddress = buf[0] == 'L' ? true : false;
			locoPtr->address = atoi(buf + 1);

			//0xE7 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2 CHK
			//locoNetSlot is indexed 1 above the systemSlot
			queueMessage(FN_OPC_SL_RD_DATA(systemSlot + 1), client);
			break;
		}//end scope block

		case OPC_MOVE_SLOTS:
			//a NULL MOVE (i.e. source slot= dest slot) from the throttle is a request to take ownership of that loco
			//we respond with OPC_SL_RD_DATA.  There is no fail response
			if (tokens[1] == tokens[2]) {
				//if ((tokens[1] < 1) || (tokens[1] > MAX_LOCO)) return;
				//locoPtr = &loco[tokens[1] - 1];
				locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
				if (locoPtr) {
					//valid locoNet slots are 1 to 127 but we need to translate this to 0 to MAX_LOCO-1
					locoPtr->LocoNetSTATUS1 |= 0b110000;  //set <5,4> of STATUS1. IN_USE slot refreshed
					queueMessage(FN_OPC_SL_RD_DATA(tokens[1]), client);
				}
			}
			return;

		case OPC_SLOT_STAT1:
			//[B5 01 13 58]  Write slot 1 with status value 19 (0x13) - Loco is Not Consisted, Common and operating in 128 speed step mode.
			//typically this is how a loco is released
			
			//if ((tokens[1] < 1) || (tokens[1] > MAX_LOCO)) return;
			//locoPtr = &loco[tokens[1] - 1];

			locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
			if (locoPtr) {
				locoPtr->LocoNetSTATUS1 = tokens[2];
			}
			return;

		case OPC_LOCO_SND:
			//set slot sound function.
			locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
			if (locoPtr) {
				//SND is x,x,x,x,F8,7,6,5 
				locoPtr->function &= 0xFFF0;
				locoPtr->function |= (tokens[2] << 4);
				locoPtr->functionFlag = true;
			}
			
			break;

		case OPC_LOCO_DIRF:
			//Set slot direction and function 0-4 state.  DIRF encoded as 0,0,DIR,F0,F4,F3,F2,F1
			//if ((tokens[1] < 1) || (tokens[1] > MAX_LOCO)) return;
			//locoPtr = &loco[tokens[1] - 1];

			locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
			if (locoPtr) writeDIRF_SPD(&tokens[2], nullptr, locoPtr);
			return;
		
		case OPC_LOCO_SPD:
				//set slot speed.  LocoNet uses 0=stop, 1=eStop and 0x02-0x7F as the actual speed, so presumably it tops out at 28 for older locos and 127 for modern ones
				//the DCC ESP system stores speedStep as a value between 0 and 28 or 0 and 127 to represent a speed.
			//if ((tokens[1] < 1) || (tokens[1] > MAX_LOCO)) return;
			//locoPtr = &loco[tokens[1] - 1];
			locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
			if (locoPtr) writeDIRF_SPD(nullptr, &tokens[2], locoPtr);
			return;


			//Next are turnout related commands

		case OPC_INPUT_REP:
			break;

		case OPC_SW_REP:
		case OPC_SW_REQ:
			// Command a turnout.
			// Immediate response of <0xB4><30><00> if command failed, otherwise no response
			break;


		}
		return;
	
	}//end 4 token

	//variable token messages
	switch (tokens[0]) {

	case OPC_WR_SL_DATA:
		//this is how the throttle sends messages to locos, 3 forms.  Response is OPC_LONG_ACK
		//0xEF 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2
		//0xEF 0x0E 0x7C PCD 0 HOPSA LOPSA TRK CVH CVL DATA7 0 0
		//0xEF 0x0E 0x7B fast clock stuff
		
		if ((tokens[2] != 0x7C) && (tokens[2] != 0x7B )){
		//write slot data form of command
			//if ((tokens[2] < 1) || (tokens[2] > MAX_LOCO)) return;
				//locoPtr = &loco[tokens[1] - 1];

				locoPtr = (LOCO*)getSsytemSlotPtr(tokens[1]);
				if (!locoPtr) return;
				locoPtr->LocoNetSTATUS1 = tokens[3];
				//it seems odd that LocoNet wants to write to the loco address, when it previously had to request the slot whilst giving an address
				//i.e. it makes no sense to change this address during this command
				writeDIRF_SPD(&tokens[6], &tokens[5], locoPtr);
				//why would client write to TRK?   there is a separate command to turn power on/off

				//write to SND, i.e. F8-4
				locoPtr->function &= 0xFFF0;
				locoPtr->function |= (tokens[2] << 4);
				locoPtr->functionFlag = true;

				return;
		}

		if (tokens[2] == 0x7C) {
			/*The programmer track is accessed as Special slot #124 ( $7C, 0x7C).  It is a full asynchronous shared system resource.
			To start Programmer task, write to slot 124 (0x7C). There will be an immediate LACK acknowledge that
			indicates what programming will be allowed. If a valid programming task is started, then at the final
			(asynchronous) programming completion, a Slot read <E7> from slot 124 will be sent.  This is the final
			task status reply

			 This OPC leads to immediate LACK codes:
				<B4>,<7F>,<7F>,<0x4B>  Function NOT implemented, no reply.
				<B4>,<7F>,<0>,<0x34> Programmer BUSY , task aborted, no reply.
				<B4>,<7F>,<1>,<0x35> Task accepted , <E7> reply at completion.
				<B4>,<7F>,<0x40>,<0x74> Task accepted blind NO <E7> reply at completion.

			Any Slot RD from the master will also contain the Programmer Busy status in bit 3 of the <TRK> byte.
			OK - so i need to link this into the SM and POM state engines
			*/

			//programming mode on slot 124.  The LocoNet spec does not support reading values on the Main (i.e. TY0/1 do not allow for this). Bummer.

			//debug respond with LACK B4 7F 01 35  Task accepted , <E7> reply at completion.
			//or LACK B4 7F 40 74
			queueMessage("RECEIVE 0xB4 0x7F 0x01 0x35\n", client);
			//queueMessage("RECEIVE 0xB4 0x7F 0x40 0x74\n", client);

			/*incoming task message is
			* <0xEF>,<0E>,<7C>,<PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
			* i.e. same as returned data except PSTAT=0 and possibly hops lopsa are also zero if LocoNet think this is the prog track
			* 
			* TO DO.
			* if we are not in SM then enter SM.  but how do we know to exit SM, because this could be dangerous if the loco was not configured correctly.  That said
			* this danger would also exist before you entered SM.  i.e. enter it first, then put untested loco on the track
			* 
			* 
			*/

			//enter service mode if we have not done so already.  The only way to exit SM is to cycle track power
			//if (!power.serviceMode) writeServiceCommand(0, 0, false, true, false);
			
			//what kind of request do we have?
			//NOTE: we have to echo most of the inbound prog bytes, but do this asynchonously.  will need a callback from the SM routine in DCCcore.
			//NOTE2: if there is a current trip, all we can do is set trackpower=off.  Loconet does not provide a means to flag the cause as a trip nor indicate what
			//the trip current level was.
			
			//DEBuG, just echo that we have read a CV value
			//D6 of PCMD is write/nRead so for reads we will return 0x88 = 0b1000 1000
			if ((tokens[3] & 0b01000000) == 0) {
				//read
				Serial.println("SMr");

				tokens[0] = 0xE7;  //response code
				//D7 is in <1> of CVH
				tokens[8] |= 0b10;
				tokens[10] = 0b1000; //D6-0
				tokens[4] = 0; //PSTAT
				tokens[11] = 0;
				tokens[12] = 0;


				m.clear();
				m.append("RECEIVE ");

			uint8_t checkSum = 0xFF;
			
			for (int i = 0;i < 13;i++) {
				snprintf(buf, 5, "%02X ",tokens[i]);
				m.append(buf);
				checkSum ^= tokens[i];
			}
			snprintf(buf, 5, "%02X\n", checkSum);
			m.append(buf);
			queueMessage(m, client);

			}

			/*PROBLEM
			* L: EF 0E 7C 2B 00 00 00 00 00 69 00 7F 7F 20

			SMr
			Out RECEIVE EF 0E 7C 2B 00 00 00 00 00 69 00 7F 7F 20  0x2b=001 01 0  11  byte read, direct read, SM

			SENT OK
			RECEIVE 0xB4 0x7F 0x01 0x35
			RECEIVE EF 0E 7C 2B 00 00 00 00 02 69 08 7F 7F 2A

			DP says timeout error talking to programmer, even though it sends the correct messages back...
			unless maybe it expects those two 0x7Fs to be zero.
			
			
			NOPE; still get a timeout.  makes no sense, the Master is responding.
			* <0xEF>,<0E>,<7C>,<PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
			Out RECEIVE EF 0E 7C 2B 00 00 00 00 00 00 03 7F 7F 4A  this is cv1, expected value 3
			SENT OK
			RECEIVE 0xB4 0x7F 0x01 0x35
			RECEIVE EF 0E 7C 2B 00 00 00 00 02 00 08 00 00 43


			Try a mainline read. You cannot, DP has greyed out the read.
			https://loconetovertcp.sourceforge.net/Protocol/SD_blocking_request.svg

			Last gasp, lets not send the LACK message. makes not difference.  give up.  DP does not implement this correctly.
			https://groups.io/g/jmriusers/topic/decoderpro_cannot_read_write/32973168

			[EF 0E 7C 2B 00 00 00 00 00 00 7F 7F 7F 36]  Byte Read in Direct Mode on Service Track: CV1.  traffic monitor shows command to Master
			[EF 0E 7C 2B 00 00 00 00 02 00 08 00 00 43]  Byte Read in Direct Mode on Service Track: CV1.  and the Master response, but DP says timeout.

			https://www.jmri.org/help/en/html/hardware/loconet/DCS240.shtml

			https://www.jmri.org/help/en/html/hardware/loconet/LocoNetSim.shtml

			*/







			//usage reg,val,read,enterSM,exitSM
			//writeServiceCommand(0, 0, false, true, false);

			//so we must always send a LACK to every request to indicate we are processing it
			//and then we asynchronously return the data


			/*final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
			
			*/





		}

		return;
	

 


	}


}

/// <summary>
/// Find system slot corresponding to locoNet slot
/// </summary>
/// <param name="locoNetSlot">loconet slot number</param>
/// <returns>pointer to system slot or nullptr if fail</returns>
void* nsLOCONETprocessor:: getSsytemSlotPtr(uint8_t locoNetSlot) {
	//Note: have to declare as void* and later cast to (LOCO*) else compilation fails
	if ((locoNetSlot < 1) || (locoNetSlot > MAX_LOCO)) return nullptr;
	return &loco[locoNetSlot-1];
}




//std::vector<std::uint8_t> tokens;
std::string nsLOCONETprocessor::echoRequest(std::vector<std::uint8_t> tokens) {

	return "X";
}


/// <summary>
/// generate a string containing OPC_SL_RD_DATA response
/// </summary>
/// <param name="locoSlot">loconet slot ref</param>
/// <param name="CV17">low address</param>
/// <param name="CV18">high address</param>
/// <returns>OPC_SL_RD_DATA hex string</returns>
std::string nsLOCONETprocessor::FN_OPC_SL_RD_DATA(int8_t locoNetSlot) {
	
	std::string m;
	//default failure message OPC_LONG_ACK, ACK1=0x00

	//locoNet slots are 1-127 but we cap at MAX_LOCO
	m.append("RECEIVE B4 BF 00 F4\n"); 
	if (locoNetSlot < 1) return m;
	if (locoNetSlot > MAX_LOCO) return m;
	m.clear();	
	m.append("RECEIVE ");

	//find the correct system slot
	LOCO loc = loco[locoNetSlot - 1];

	//response 0xE7 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2 CHK
	uint8_t returnArray[13] = {0};   //force initialisation to zeros 
	char buf[5];

	returnArray[0] = OPC_SL_RD_DATA;
	returnArray[1] = 0x0E;
	returnArray[2] = locoNetSlot;  //the locoNet slot ref
	returnArray[3] = loc.LocoNetSTATUS1;  //STAT1 
	//<7> slot purge, <6,5> consist where 00 is no consist, <5,4> busy/active <2,1,0> 011=128 speed steps
	
	//loconet does not have a concept of short/long addresses, instead it takes a 
	//14 bit address and splits this over two 7 bit bytes.
	returnArray[4] = loc.address & 0x7F;  //lower 7 bits for ADR
	returnArray[9] = loc.address >> 7;  //shift bits 7+ to zero for ADR2
	returnArray[5] = loc.speedStep;
	uint8_t DIRF = loc.forward ? 0b00100000 : 0x00;
	DIRF += (loc.function >> 1) & 0b1111;
	DIRF += (loc.function << 4) & 0b00010000;
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

/// <summary>
/// write DIR and/or SPD to a system slot
/// </summary>
/// <param name="dirf">DIRF byte or nullptr</param>
/// <param name="spd">SPD byte or nullptr</param>
/// <param name="loc">pointer to the system loco slot</param>
void nsLOCONETprocessor::writeDIRF_SPD( uint8_t* dirf, uint8_t* spd, void* loc) {
	//cast loc to LOCO
	LOCO* l = (LOCO*)loc;
	if (loc == nullptr) return;
	if (spd != nullptr) {
		switch (*spd) {

		case 1:
			//locoPtr->eStopTimer = 5;   //flag an estop somehow...
		case 0:
			l->speed = 0;
			l->speedStep = 0;
			break;

		default:
			l->speedStep = *spd - 1;
		}

		//now calculate as a float percentile
		if (l->use128) {
			l->speed = l->speedStep / 127.0;
		}
		else {
			l->speed = l->speedStep / 27.0;
		}

		l->changeFlag = true;

	}
	
	if (dirf != nullptr) {
		//Set slot direction and function 0-4 state.  DIRF encoded as 0,0,DIR,F0,F4,F3,F2,F1
		l->function &= 0xFFE0;
		l->function |= (*dirf << 1) & 0b11110; //F4-F1
		l->function |= (*dirf >> 4) & 0b1; //F0
		l->functionFlag = true;
		
		l->forward = (*dirf & 0b100000) != 0 ? true :false; //assumes DIR=1 means forward
		l->directionFlag = true;
	}

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