// 
// 
// 

#include "LocoNetprocessor.h"
#include "Global.h"
#include "DCCcore.h"
#include "WiThrottle.h"  //for debug over tcp

/*TO DO
* loss of loconet client should cause all slots to go to zero speed, and and Status to Common.
*
* broadcast messages?  There is NO need to send broadcasts from LocoNet because this is required for multi-throttles, and this is handled by DP.
* 
* 
* Notes on DP.
* you can use DP to host a throttle controller.  this allows you to use ED and also open throttles on the laptop.  Note though, that on ED, if you select a loco from the server roster
* it will not display any function buttons.  This is a bug in DP, where it fails to send details of available functions to ED.  You will find that if instead you select a loco address
* directly by entering it into ED, then all the functions will appear on the throttle.
*
*
* debug approach.  The ESP is on .129 port 2560.  I can connect to this with Hercules TCP client.  If we then send to 'all clients' as ASCII debug this should work.
* 
* POM, I need to reprogram this to expect NACK, ACK, BUSY via a callback.  These are only expected if a callback routine has been set.
*/



/*LocoNet support 2025-12
 
Note: JRMI Decoder Pro (DP) does not seem to recognise it has lost the TCP connection and does not attempt a re-initialise of LocoNet.  You have to manually restart DP.

The base protocol for loconet is found in the loconet personal edition https://www.digitrax.com/support/loconet/loconetpersonaledition.pdf
JRMI DP uses loconet over TCP which has additional message prefixes https://loconetovertcp.sourceforge.net/Protocol/LoconetOverTcp.html
in short, you first echo the command then transmit SENT OK and then transmit your response. https://loconetovertcp.sourceforge.net/Protocol/SD_blocking_request.svg
Useful glossary from digitrax https://www.digitrax.com/tsd/glossary/c/

The protocol has a series of system memory 'slots' these are not loco addresses.  slot 124 is a special slot to control programming
A slot is set up to hold a loco and then commands are sent to that slot and the controller will repeately transmit the slot content to the track
This is pretty much how my original project works anyway, with the Loco[] array.

IMPORTANT: this ESP project was originally designed as a stand-alone DCC control and implements a WiThrottle server.  If you link this controller to DP over LocoNetoverTCP then
the laptop needs to run the WiThrottle server and it in turn will issue slot commands over loconet to the controller which will transmit to the track.  i.e. Engine Driver now connects to 
the laptop and not to the ESP.

This module uses the existing ESP WiThrottle port to monitor for incoming LocoNet messages.  It recognises the incoming messages as loconet and changes the handler that
receives them to direct the messages to the LocoNetprocesssor.  This routine also makes calls to DCC core for programmer support, and asynchronous callbacks are made from DCC core
to this module to handle incoming ACK pulses/ Service Mode read/writes as well as those for Program on Main.

Note that DP does not support reading cvs on the main, even though notionally the DP GUI appears to support this, you will find the app refuses to process such requests and so no
commands are sent over loconet to this module.

LocoNet locos are in slots 1-127, so we need to offset by one to make use of the loco[] array.  Slot 0 is used for dispatch which we don't need.
to release a loco, locoNet writes 0x13 to STATUS1. 0b0001 0011. loco not consisted and common.

Using locoNet will overwrite the roster that is stored in the ESP-DCC.  The roster itself is managed in DP, no longer stored on the ESP.  The ESP will restore its locally stored roster on boot.

Note: LocoNet asks for control of a loco address. If the Master responds that the loco is in use, then the client is supposed not to ask to use it!  JRMI seems to
go ahead and flag that loco for inUse a second time.  Not true.  this is an aberration of ED.  e.g. ED asks for loco 464 and you see this executed on LocoNet.
if you use ED to request 464 a second time then no loconet traffic is generated.  ED is just doubling the 464 loco within its own app.

This system does not support DCCeX because I consider it an inferior protocol to LocoNet.

Important: this module is an adjunct to WiThrottle.  In WiThrottle, there is a routine broadcastChanges() which is called regularly from the main loop.  This routine has a local 
sendToClient() routine which addess messages to the outbound queue.

Calls to write/read the prog track and to write/read POM result in an async callback to callbackLocoNet() this in turn generates a locoNet message to confirm the operation 
was a success or not.  Note that DP does not define how to enter/exit Service Mode (prog track).  This code will enter SM if an SM command is received.  It will only exit SM if
the track power is cycled, but this could be dangerous because say you have a mis wired decoder, you initialise SM then put the loco on the track.  it causes a trip and then you 
exit SM.  This would put full power on the miswired decoder....well ok, but then how are we to exit SM?   Ditto track is not in SM mode until you send the first SM command, so a miswired
decoder could detonate.  Perhaps the DP designers' assumption was the controller has a permanent SM output.

Note: DP does not support POM-read via Railcom for entire pages of CVs, but it does support individual CV reads on POM under the CVs tab.

*/



using namespace nsLOCONETprocessor;

static std::vector<CLIENTMESSAGE> messages;


//use this struct to hold decoder programming messages and because the response is async we need to hold a pointer
//to the client that launched the original programmer task.
struct PROGDETAIL {
	uint8_t PCMD;
	uint8_t PSTAT;
	uint16_t address;
	uint16_t cv;
	uint8_t cvData;
	AsyncClient* client;
};

static PROGDETAIL slot124Message;


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
/// <param name="msg">A single incoming SEND message</param>
/// <param name="client">TCP client</param>
void nsLOCONETprocessor::tokenProcessor(char* msg, AsyncClient* client) {
	//its easier to break on spaces using strtok and build a vector of ints
	//and use strtoul just for conversion of a single token rather than iteration of msg
	//https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
	
	LOCO* locoPtr=nullptr;  //pointer to the actual system loco slot (offset by 1 from LocoNet slot)
	
	std::vector<std::uint8_t> tokens;
	#define BUFSIZE 25
	char buf[BUFSIZE];

	trace(Serial.printf("In: %s\r\n", msg);)
	
//Loconet over TCP requires that we echo the message prefixed by RECEIVE
//and follow this with SENT OK
	std::string m;   
	m.append("RECEIVE ");
	m.append(msg);
	m.append("\nSENT OK\n");
	
	char* tokenSplit = strtok(msg, " ");
	while (tokenSplit != NULL) {
		//tokens.push_back(strtoul(tokenSplit, nullptr, 16));
		//emplace does not create copies of things and is more efficient
		tokens.emplace_back(strtoul(tokenSplit, nullptr, 16));
		tokenSplit = strtok(NULL, " ");
	}

	uint8_t checksum;
	//XOR all tokens, should give 0xFF
	for (auto t : tokens) {	checksum ^= t;}

	//if checksum fails, ignore the message
	if (checksum != 0xFF) { return;	}
	
	queueMessage(m, client);
	m.clear();
			
	//process TWO-TOKEN messages
	if (tokens.size() == 2) {
		switch (tokens[0]) {
		case OPC_IDLE:
			//	FORCE IDLE state, B'cast emerg. STOP
			
			break;
			
		case OPC_GPON:
			power.trackPower = true;
			nsWiThrottle::queueMessage("pwr on\n","DEBUG");
			break;

		case PC_GPOFF:
			power.trackPower = false;
			nsWiThrottle::queueMessage("pwr off\n","DEBUG");
			//exit service mode if we are in this
			writeServiceCommand(0, 0, false, false, true, nullptr);
			nsWiThrottle::queueMessage("exit sm\n", "DEBUG");
		}
		return;
	}

	//FOUR-TOKEN messages
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
			If the STATUS1 byte shows the SLOT to be COMMON, IDLE or NEW the throttle may change
			the SLOT to IN_USE by performing a NULL MOVE instruction <BA>,<slotX>,<slotX>,<chk> on this SLOT.
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
				//token[1] will be ADR2 <7+> msb and token[2] is ADR <0-6> lsb 
				uint16_t addr = (tokens[1] << 7) + tokens[2];
				snprintf(buf, BUFSIZE, "L%d", addr);
			}

			//find slot containing the loco.  LocoNet slots 1-120 are for locos.  Use the index of loco array + 1 as the loconet slot
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
				locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
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

			locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
			if (locoPtr) {
				locoPtr->LocoNetSTATUS1 = tokens[2];
			}
			return;

		case OPC_LOCO_SND:
			//set slot sound function.  This is really just F5-8
			locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
			if (locoPtr) {
				//SND is x,x,x,x,F8,7,6,5 
				locoPtr->function &= 0xFFF0;
				locoPtr->function |= (tokens[2] << 4);
				locoPtr->functionFlag = true;
			}
			
			break;

		case OPC_LOCO_DIRF:
			//Set slot direction and function 0-4 state.  DIRF encoded as 0,0,DIR,F0,F4,F3,F2,F1
			locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
			if (locoPtr) writeDIRF_SPD(&tokens[2], nullptr, locoPtr);
			return;

		case OPC_LOCO_SPD:
			//set slot speed.  LocoNet uses 0=stop, 1=eStop and 0x02-0x7F as the actual speed, so presumably it tops out at 28 for older locos and 127 for modern ones
			//the DCC ESP system stores speedStep as a value between 0 and 28 or 0 and 127 to represent a speed.
			locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
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

	//VARIABLE-LENGTH token messages
	switch (tokens[0]) {

	case OPC_WR_SL_DATA:
		//this is how the throttle sends messages to locos, 3 forms exist.  Response is OPC_LONG_ACK
		//0xEF 0x0E SLOT# STAT1 ADR SPD DIRF TRK SS2 ADR2 SND ID1 ID2  regular speed command to loco
		//0xEF 0x0E 0x7C PCD 0 HOPSA LOPSA TRK CVH CVL DATA7 0 0  programming command
		//0xEF 0x0E 0x7B fast clock command

		if ((tokens[2] != 0x7C) && (tokens[2] != 0x7B)) {
			//write slot data form of command

			locoPtr = (LOCO*)getSytemSlotPtr(tokens[1]);
			if (!locoPtr) return;
			locoPtr->LocoNetSTATUS1 = tokens[3];
			//the command contains the loco address, even though the slot was set up with this address. i.e. ignore the address
			//the command contains a TRK byte, but there is a separate command to turn power on/off. i.e. ignore TRK
						
			writeDIRF_SPD(&tokens[6], &tokens[5], locoPtr);
			
			//write to SND, i.e. F8-4
			locoPtr->function &= 0xFFF0;
			locoPtr->function |= (tokens[2] << 4);
			locoPtr->functionFlag = true;
			return;
		}

		if (tokens[2] == 0x7C) {
			/*PROGRAMMER TASK
			Programmer track is accessed as Special slot 124 (0x7C).It is a full asynchronous shared system resource.
			To start Programmer task, write to slot 124 (0x7C).There will be an immediate LACK acknowledge that
			indicates what programming will be allowed.If a valid programming task is started, then at the final
			(asynchronous) programming completion, a Slot read <E7> from slot 124 will be sent.This is the final
			task status reply.

		
			This OPC leads to immediate LACK codes :
			<B4>, <7F>, <7F>, <0x4B>  Function NOT implemented, no reply.
			<B4>, <7F>, <0>, <0x34> Programmer BUSY, task aborted, no reply.
			<B4>, <7F>, <1>, <0x35> Task accepted, <E7> reply at completion.
			<B4>, <7F>, <0x40>, <0x74> Task accepted blind NO <E7> reply at completion.
			Any Slot RD from the master will also contain the Programmer Busy status in bit 3 of the <TRK> byte.
			*/
			
										
			slot124Message.client = client;

			//*incoming task <0xEF>,<0E>,<7C>,<PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
			slot124Message.address = (tokens[5] << 7) + tokens[6];
			slot124Message.cv = (tokens[8] >> 3) + (tokens[8] & 0b1); // CVH is <0,0,CV9,CV8 - 0,0, D7,CV7> bonkers format
			slot124Message.cv = (slot124Message.cv << 7) + tokens[9];
			slot124Message.cvData = tokens[10]; //data7  
			slot124Message.cvData += (tokens[8] & 0b10) == 0?0:0x80;//D7 from CVH
			slot124Message.PCMD = tokens[3];
									
			bool doRead;
			switch (tokens[3] & 0b00111100) {
			case 0b00101000: //direct, SM, byte
			case 0b00101100: //ops byte with ACK
			case 0b00100100: //ops byte with no-ACK
				//enter service mode, this is a call to DCCcore.cpp
				writeServiceCommand(0, 0, true, true, false, nullptr);
				break;
			default:
				//operation not supported
				queueMessage("RECEIVE 0xB4 0x7F 0x7F 0x4B\n", client);
				return;
			}
			doRead = (tokens[3] & 0b01000000) == 0 ? true : false;

			if ((tokens[3] & 0b100) == 0){
				//SERVICE MODE
				//LocoNet encodes CV1 as a zero value in HOPSA LOPSA, we need to add 1 before calling writeServiceCommand

				if (ServiceModeBusy()) {  //this is a function call to DCCcore
					queueMessage("RECEIVE 0xB4 0x7F 0x00 0x34\n", client);  //LACK and busy
					nsWiThrottle::queueMessage("busy\n", "DEBUG");
					return;
				}

				if (writeServiceCommand(slot124Message.cv+1, slot124Message.cvData, doRead, false, false, &handlerLocoNet)) {
					//read requested, E7 response is via an asynchronous callback
					queueMessage("RECEIVE 0xB4 0x7F 0x01 0x35\n", client);  //LACK and will respond with E7
					
					snprintf(buf, BUFSIZ, "cv %d\n", slot124Message.cv+1);
					nsWiThrottle::queueMessage(buf, "DEBUG");
				}
				else {
					//some other error, e.g. not in service mode or sm is busy
					queueMessage("RECEIVE 0xB4 0x7F 0x00 0x34\n", client);  //LACK and busy
				}
				return;

			}
			else
			{//POM MODE
				nsWiThrottle::queueMessage("wsc pom\n","DEBUG");

				//loconet cvs are zero based
				if ((slot124Message.PCMD & 0b1000) == 0) {
					//operation not asking for feedback, blind accept, no E7 on completion
					//queueMessage("RECEIVE 0xB4 0x7F 0x40 0x74\n", client);

					//https://github.com/JMRI/JMRI/issues/5128 this is odd, apparently B4 6F 40 64 is LACK accept blind
					//queueMessage("RECEIVE 0xB4 0x6F 0x40 0x64\n", client); but this is not recognised by DP either


					setPOMfromLoconet(slot124Message.PCMD, slot124Message.address, slot124Message.cv, slot124Message.cvData, nullptr);
					nsWiThrottle::queueMessage("wsc nfb\n", "DEBUG");
					slot124Message.PSTAT = 0x00; //no write ack from decoder 0b10, because you didn't ask for one!
					writeProgrammerTaskFinalReply();
				
					//There is a bug in DP. It issues a Byte Write, no feedback command but does not accept a LACK no E7 in response.
					//Instead it expects an E7 response and bitches if this carries no ACK, even though this is what it asked for.
					//So we can either fake an ACK or actually test for one.


				}else if (setPOMfromLoconet(slot124Message.PCMD, slot124Message.address, slot124Message.cv, slot124Message.cvData, &handlerLocoNet)) {
					queueMessage("RECEIVE 0xB4 0x7F 0x01 0x35\n", client);  //LACK and will respond with E7
				}
				else{
					//operation not supported
					queueMessage("RECEIVE 0xB4 0x7F 0x7F 0x4B\n", client);
				}

			
				/*final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>*/
				
				return;
			}

			return;
			

		}  //END 0x7C block.

	return;
	

	}


}


/// <summary>
/// Writes an <E7> response back to Loconet client which initiated the programmer operation
/// </summary>
void nsLOCONETprocessor::writeProgrammerTaskFinalReply(void) {
	/*final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>*/
	if (slot124Message.client == nullptr) return;
	char buf[5];
	std::string m;
	m.append("RECEIVE E7 0E 7C ");
	uint8_t checksum = 0xFF ^ 0xE7 ^ 0x0E ^ 0x7C;

	snprintf(buf, 5, "%02X ", slot124Message.PCMD);
	m.append(buf);
	checksum ^= slot124Message.PCMD;
	snprintf(buf, 5, "%02X ", slot124Message.PSTAT);
	m.append(buf);
	checksum ^= slot124Message.PSTAT;
	//HOPSA is 6 msb of an 11 bit address, LOPSA is 7 lsb of address, .
	snprintf(buf, 5, "%02X ", slot124Message.address >> 7);
	m.append(buf);
	checksum ^= (slot124Message.address >> 7);
	snprintf(buf, 5, "%02X ", slot124Message.address & 0x7F);
	m.append(buf);
	checksum ^= (slot124Message.address & 0x7F);
	//TRK is 0b101 for now, <1,0> are estop, power-on
	uint8_t trk = 0b100;
	trk += power.trackPower ? 1 : 0;
	snprintf(buf, 5, "%02X ", trk);
	m.append(buf);
	checksum ^= trk;

	//CVH is  <0,0,CV9,CV8 - 0,0, D7,CV7>
	uint8_t cvh = (slot124Message.cv >> 7);  //preserve <cv9,8,7> as lsb
	cvh = cvh << 3; //<cv 9,8,7 0 0 0>
	cvh += (cvh & 0b1000) == 0 ? 0 : 1;  //copy cv7 to 0 bit posn
	cvh &= 0b00110001; //clear bits 3,2,1
	cvh += (slot124Message.cvData & 0x80) == 0 ? 0 : 1;//add in D7 databit
	snprintf(buf, 5, "%02X ", cvh);
	m.append(buf);
	checksum ^= cvh;
	
	snprintf(buf, 5, "%02X ", slot124Message.cv & 0x7F);  //<CVL>
	m.append(buf);
	checksum ^= (slot124Message.cv & 0x7F);
	snprintf(buf, 5, "%02X ", slot124Message.cvData & 0x7F);  //<DATA7>
	m.append(buf);
	checksum ^= (slot124Message.cvData & 0x7F);
	m.append("00 00 ");
	snprintf(buf, 5, "%02X \n", checksum);  // newline char essential here, else DP will fail to process the message
	m.append(buf);
	queueMessage(m, slot124Message.client);
	nsWiThrottle::queueMessage(m, "DEBUG");

	/*final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>*/

	//cBAK 0 	RECEIVE E7 0E 7C 2B 00 00 00 05=trk 00=cvH 01=cvL     00=data7    00 00 45
	//comes up as 1, not zero

	//there's a bug in the DP CV screen.
	// [E7 0E 7C 2B 00 00 00 05 00 01 00 00 00 45]  Programming Response: Read Byte in Direct Mode on Service Track Was Successful: CV2 value 0 (0x00, 00000000b).
	//i.e. it read as zero, DP decoded it as zero and yet it writes a 1 to the CV window.  IF you run a compare operation, it correctly can verify zero.
	//the bug is inconsistent because sometimes it can correctly read zero values.


}






/// <summary>
/// handles async callbacks from DCCcore relating to Service Mode and POM. Does not support bit manipulation.
/// </summary>
/// <param name="ack">true if ACK seen, meaning data was read/written correctly</param>
/// <param name="cvReg">cv register</param>
/// <param name="cvVal">cv data</param>
void nsLOCONETprocessor::handlerLocoNet(bool ack, uint16_t cvReg, uint8_t cvVal) {
	char buf[12];
	if (ack) {
		snprintf(buf, 12, "cBAK %d\n ", cvVal);
	}
	else {
		snprintf(buf, 12, "cBAD %d\n ", cvVal);
	}
	
	nsWiThrottle::queueMessage(buf,"DEBUG");
	
	/*final response <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
	PSTAT is <7-4> reserved <3> user abort <2> failed read (no ACK) <1> no write ACK response<0> no loco
	PCMD was <7>=0 <6>wr/rd <5>byte/bit <4,3>ty1,2 <2>ops/sm <1,0> reserved
	*/
	std::string m;
	
	
	//we see PCMD = d43 =0x2b = 0010 1011. 
	//[EF 0E 7C 2B 00 00 00 00 00 00 03 7F 7F 4A]  Byte Read in Direct Mode on Service Track: CV1.

	if ((slot124Message.PCMD & 0b00100) == 0) {
		//Service Mode. For read and write we expect ACK.  Note that LocoNet also has error type PSTAT<0> meaning no loco detected
		//but I fail to see the benefit of implmenting this.  A loco with dirty wheels would give the same error as no loco at all.
		if ((slot124Message.PCMD & 0b1000000) == 0) {
			//SM read
			slot124Message.PSTAT = ack ? 0: 0b100;  //<2> no read ACK from decoder
			if (ack) slot124Message.cvData = cvVal;

		}
		else {
			//SM write
			slot124Message.PSTAT = ack ? 0:0b10;  //<1>=0 signifies write ACK seen from decoder

		}
		writeProgrammerTaskFinalReply();


	}
	else {
		//POM. If client expects ACK we need to confirm we saw it. TY=00 is no act TY=01 expect ACK
		//better to mask the 5 bits in and then do specific test
		//2025-12-29 whilst the DCC spec does allow for a current pulse ACK in response to POM, no decoder I own does this, and furthermore its far
		//more difficult to dectect a 64mA current pulse on the main where multiple other locos are running and causing current pulses due to poor contact.
		//S-9.3.2 table 2 there is a Railcom ACK code.  Not sure if my software picks this up.  Prob should run some POM commands and see if ACK appears in the railcom slot.

		nsWiThrottle::queueMessage("POMcb", "DEBUG");
		if ((slot124Message.PCMD & 0b1000000) == 0) {
			//POM reads, we expect to see ack=true
			slot124Message.PSTAT = ack ? 0 : 0b100;  //<2> no read ACK from decoder
			if (ack) slot124Message.cvData = cvVal;

		}
		else {
			//POM writes, we expect ack=true, if not then data is the actual ctrl value
			slot124Message.PSTAT = ack ? 0 : 0b10;  //<1> no write ACK from decoder



			//specifically you can send a busy message, if seen, rather than a failed write
			//the problem is, we have now overwritten the prior buffered cv+data we were trying to write... so we need to detect busy ahead of calling writeToPOM()

		}
		writeProgrammerTaskFinalReply();
		
		

		
	}

	


}


/// <summary>
/// Find system slot corresponding to locoNet slot
/// </summary>
/// <param name="locoNetSlot">loconet slot number</param>
/// <returns>pointer to system slot or nullptr if fail</returns>
void* nsLOCONETprocessor:: getSytemSlotPtr(uint8_t locoNetSlot) {
	//Note: have to declare as void* and later cast to (LOCO*) else compilation fails
	if ((locoNetSlot < 1) || (locoNetSlot > MAX_LOCO)) return nullptr;
	return &loco[locoNetSlot-1];
}



/*
std::string nsLOCONETprocessor::echoRequest(std::vector<std::uint8_t> tokens) {
	return "X";
}
*/

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

