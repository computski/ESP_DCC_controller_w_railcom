
//DCCcore.cpp 
// 
//2024-11-04 railcom cutout is sent after every DCC speed packet and after a POM packet (see DCCpacket.doCutout=true).  If a loco is writen to via POM, it can respond via Railcom
//2026-01-05 added support for LocoNet calls to this module

#include "Global.h"
#include "DCCcore.h"
#include "DCClayer1.h"
#include "DCCweb.h"
#include "WiThrottle.h"  //DEBUG for POM only


#include <LiquidCrystal_I2C.h>   //Github mlinares1998/NewLiquidCrystal
//https://github.com/mlinares1998/NewLiquidCrystal

/*LCD specific I2C hardware wireup, uses the LiquidCrystal_I2C library originally from
https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home   but now a dead link
allows you to wire-up the pins P0-P7 to match the EN,RW etc pins on the display
addr, en,rw,rs,d4,d5,d6,d7,backlight,backlightPolarity
BOOTUP_LCD creates an instance of the LiquidCrystal_I2C class and defines its parameters
its defined in Global.h hardware section
*/

//current and voltage monitoring device from arduino library
#include <Adafruit_INA219.h>


BOOTUP_LCD


#ifdef INA_ADDRESS
/*possible addresses are 0x40,0x41,0x44,0x45
	https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/library-reference
	*/
	Adafruit_INA219 ina219(INA219_ADDRESS);
#else
/*default is 0x40*/
Adafruit_INA219 ina219;
#endif



/*NOTES on the DCC protocol

A long address can be in the range 1 - 10023
A short address is 1-127
the decoder is set in cV29 to work either with a short or long value, these are mutually exclusive.

This system supports the full range of both long and short addresses.  For example if you wish to use 39 as a long address, you may
however general convention in the industry means that addresses of 1-127 are typically only used in short form

The accessory address space is another, separate address space.  This system supports the full address range 1-2047
and uses the common industry convention of addresses being sequentially 1 value apart, rather than the true DCC specification
of addresses having sub-elements 1 to 4 beneath them (which the DCC++ protocol implements).

*/

//2026-0-05 pointer to a callback function in the locoNetprocessor
//https://stackoverflow.com/questions/1789807/function-pointer-as-an-argument
static void (*LocoNetcallbackPtr)(bool,uint8_t); 



/*defaultController contains defaults defined in the CONTROLLER object (DCCcore.h) and is used to override eeprom settings
on new compilations where data structures have changed.  The system runs off the bootController object normally*/
CONTROLLER bootController;


/*These are defined as externs in the header, they are instanced here*/
POWER power;
KEYPAD keypad;
nsJogWheel::JOGWHEEL jogWheel;
TURNOUT turnout[MAX_TURNOUT];
LOCO loco[MAX_LOCO];
ACCESSORY accessory;
bool quarterSecFlag;

static SM m_sm;
static POM m_pom;

uint8_t m_generalTimer;
uint8_t m_tick;  //25 ticks in a quarter second
uint8_t m_eStopDebounce;  //holds debounce scan of local estop button

static DCCBUFFER m_LocoNetHoldingBuffer;  //2026-02-18


/*define LCD special chars map to 0x01 through 0x06*/
const uint8_t UP_ARROW[8] = {
0b00100,
0b01110,
0b10101,
0b00100,
0b00100,
0b00100,
0b00100,
0x00  //cursor row
};

const uint8_t DOWN_ARROW[8] = {
0b00100,
0b00100,
0b00100,
0b00100,
0b10101,
0b01110,
0b00100,
0x00  //cursor row
};

const uint8_t OPEN_DOT[8] = {
0x00,
0b01110,
0b10001,
0b10001,
0b10001,
0b01110,
0x00  //cursor row
};

const uint8_t CLOSE_DOT[8] = {
0x00,
0b01110,
0b11111,
0b11111,
0b11111,
0b01110,
0x00  //cursor row
};

const uint8_t SPEED_28[8] = {
0x00,
0x00,
0x00,
0b00100,
0b01110,
0b10001,
0x00  //cursor row
};

const uint8_t SPEED_126[8] = {
0b00100,
0b01110,
0b10001,
0b00100,
0b01110,
0b10001,
0x00  //cursor row
};

const uint8_t SHUNTER_UP_ARROW[8] = {
0b00100,
0b01110,
0b11111,
0b00100,
0b00100,
0b00100,
0b00100,
0x00  //cursor row
};

const uint8_t SHUTER_DOWN_ARROW[8] = {
0b00100,
0b00100,
0b00100,
0b00100,
0b11111,
0b01110,
0b00100,
0x00  //cursor row
};



//ESP onboard LED flashing states
enum ledSTATE
{
	L_NORMAL = 0b10000000,
	L_ESTOP = 0b11001100,
	L_SERVICE = 0b11110000,
	L_TRIP = 0b10101010
};

uint8_t m_stateLED = L_NORMAL;
uint8_t m_ledCount;




static uint8_t m_locoIndex = 0;
static uint8_t m_funcIndex = 0;

dccSTATE dccSE = DCC_LOCO;


/*turnout related*/
enum turnoutSTATE
{
	TURNOUT_DIGIT_1_WAIT,
	TURNOUT_DIGIT_1,
	TURNOUT_DIGIT_2_WAIT,
	TURNOUT_DIGIT_2,
	TURNOUT_TOGGLE
};


/*statics*/
turnoutSTATE m_turnoutSE = TURNOUT_DIGIT_1_WAIT;
uint8_t m_turnoutEnteredAddress;  //pending address


/*state engine for the local machine*/
enum machineSTATE
{
	M_BOOT,
	M_TURNOUT,
	M_SERVICE,
	M_POM,   //prog on main
	M_POWER,  //power and voltage settings
	M_ESTOP,
	M_TRIP,
	M_UNI_SET,  //uni throttle set loco address and speed steps
	M_UNI_RUN  //uni throttle runtime
};


/*statics*/
machineSTATE m_machineSE = M_BOOT;



#pragma region Test_and_debug
/*DEBUG ROUTINES*/
void debugTurnoutArray(void) {
	//debug dummp the turnout array
	for (int i = 0; i < MAX_TURNOUT; ++i) {
		Serial.println("");
		Serial.print(turnout[i].address);
		Serial.print(" ");
		Serial.print(turnout[i].history);
		Serial.print(" ");
		Serial.print(turnout[i].selected);
	}
}

void debugPacket(void) {
	Serial.println(F("dccPacket"));

	for (int i = 0; i < DCCpacket.packetLen; ++i) {
		Serial.println(DCCpacket.data[i], BIN);
	}
	Serial.println(DCCpacket.packetLen, DEC);
}


float getVolt() {
	//debug
	return ina219.getBusVoltage_V();
}
/*END DEBUG ROUTINES*/
# pragma endregion


/*UniThrottle is the local hardware interface throttle.  It can select/display a single loco at a time from the roster
The A key will enter/exit address set mode.  when in this mode,* will set short/long address, # will set 28/126 speed steps
B will toggle between locos in the roster (or abort address edit)
C will toggle shunter mode
D will toggle the headlight
digits 1-8 will set the functions
* and 0 represent speed up/down, or together invoke estop on that loco
# will toggle direction
Consists are not supported.
*/



struct UNITHROTTLE {
	LOCO* locPtr;
	uint8_t digitPos;
}unithrottle;

LOCO m_tempLoco;

//generates dcc packets and queues to DCClayer1
void dccPacketEngine(void) {
	/*transmit the loco buffer, if a loco address is zero, transmit idle instead
	 * note: there is a finite time window to calculate DCCpacket values before DCCclearToSend is set to false by the layer1
	 * buffer-copy mechanism.
	 *
	 * Note:  dccState can be changed by other functions outside of this DCCcore function, e.g. eStop or Accessory
	 * and this will prioritise their transmission
	 *
	 * A packet takes around 8mS to transmit.
	 * The routine waits for DCCpacket.clearToSend, and immediately clears this.  It loads up the DCCpacket buffer and
	 * then changes to the next state, however this next state cannot be executed until another CTS is seen.
	 *
	 *
	 *note need to add funcIndex and run this thro 0-11 for 4loco x 3 func groups
	 *int(funcIndex/3) to point to Loco[] and then mod 3 for the group itself
	 *
	 * 2023-08-30 maybe we should not wipe the cts flag on entry, but instead build the packet and test when cts flag is still there just at the point
	 * of writing it.  That said, the int-routine just copies whatever's in the buffer at the point it decides to read it. so if we keep that flag mostly clear
	 * then we should spot it going hi immediately after the buffer was read. so yes, keep code as is.
	*/


	if (!DCCpacket.clearToSend) return;

	DCCpacket.clearToSend = false;
	uint8_t i = 0;

	//2021-10-14 enable/disable of track power is now controlled from DCClayer1
	DCCpacket.trackPower = power.trackPower;
	//2023-09-27 default is to disable railcom, and only enable on a loco speed packet
	DCCpacket.doCutout = false;


	switch (dccSE) {

	case DCC_LOCO:
		power.serviceMode = false;
		/*skip any loco packets with address zero as this is a broadcast address*/
		if (loco[m_locoIndex].address == 0) {
			/*send idle for zero-address (empty) slots*/
			DCCpacket.data[0] = 0xFF;
			DCCpacket.data[1] = 0;
			DCCpacket.data[2] = 0xFF;
			DCCpacket.packetLen = 3;

		}
		else
		{
			/*Build a packet, first step is to calculate NMRA speedCode to send to line*/
			uint8_t speedCode = loco[m_locoIndex].speedStep;
			/*2019-10-11 speedStep is the UI displayed value e.g. 0-28 or 0-128, active braking will halve this value*/
			if (loco[m_locoIndex].brake) { speedCode = speedCode / 2; }
			/*this does not impact the value displayed but does impact the value transmitted to line*/

			if (loco[m_locoIndex].use128) {
				/*calculate 128 step code. speed value 1 in the UI maps to 2 in NMRA code
				display code 126 represents max speed and is a NRMA code of 127*/
				if (speedCode > 0) { speedCode++; }
				speedCode &= 0b01111111;
			}
			else {
				/*calculate 28 step code. see S-9.2 para 60*/
				if (speedCode > 0) {
					speedCode += 3;
					/*move <0> to <5> then shift result >>1*/
					speedCode &= 0b00011111;
					speedCode |= (speedCode & 0x01) << 5;
					speedCode = speedCode >> 1;
				}
			}
			/*done, how we use this code depends on whether we use baseline or extended packets
			 *note that an address<127 with a 28 step speed is a baseline packet.  This code does
			 *not implement addresses<127 as long adddresses.
			 *Decoders can be set to respond to either short or long address, but never both.
			 */

			if (loco[m_locoIndex].useLongAddress) {
				/*long address format S9.2.1 para 60*/
				DCCpacket.data[0] = loco[m_locoIndex].address >> 8;
				DCCpacket.data[0] |= 0b11000000;
				DCCpacket.data[1] = loco[m_locoIndex].address & 0x00FF;
				i = 2;
			}
			else {
				DCCpacket.data[0] = (loco[m_locoIndex].address & 0x7F);
				i = 1;
			}

			if (loco[m_locoIndex].use128) {
				/*two speed-bytes*/
				DCCpacket.data[i] = 0b00111111;
				i++;
				/*mask in <7> which is direction*/
				if (loco[m_locoIndex].forward) { speedCode |= 0b10000000; }
				/*nudge code, will assert max speed in alternate directions until nudge=0*/
				if (loco[m_locoIndex].nudge > 0) {
					speedCode = 0x7F;
					if ((loco[m_locoIndex].nudge & 0x01) == 0x00) { speedCode ^= 0b10000000; }
					loco[m_locoIndex].nudge--;
				}
				/*special case for eStop*/
				if (loco[m_locoIndex].eStopTimer != 0) { speedCode = 0x01; }
				DCCpacket.data[i] = speedCode;
				i++;
			}
			else {
				/*write single speed byte in legacy mode 010=reverse speed 011=forward*/
				  /*mask in direction bit <5>*/
				if (loco[m_locoIndex].forward) { speedCode |= 0b00100000; }
				/*nudge code, will assert max speed in alternate directions until nudge=0*/
				if (loco[m_locoIndex].nudge > 0) {
					speedCode = 0x1F;
					if ((loco[m_locoIndex].nudge & 0x01) == 0x00) { speedCode ^= 0b00100000; }
					loco[m_locoIndex].nudge--;
				}
				/*special case for eStop, need to preserve direction*/
				if (loco[m_locoIndex].eStopTimer != 0) { speedCode &= 0b00100000; speedCode |= 0x01; }
				/*set <7-6> = 01*/
				DCCpacket.data[i] = speedCode | 0b01000000;
				i++;
			}
			/*calc checksum and packet length. i points to checksum byte*/
			DCCpacket.data[i] = 0;
			for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
				DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
			}
			DCCpacket.packetLen++;
			//will exit with DCCpacket.packetLen set at correct length of i+1

			DCCpacket.doCutout = true;

		} //end zero address test

		  //only advance locoIndex if we don't need to deal with nudge 
		if (loco[m_locoIndex].nudge == 0) { m_locoIndex++; }
		if (m_locoIndex >= MAX_LOCO) {
			m_locoIndex = 0;
		}
		//next up is function packet
		dccSE = DCC_FUNCTION;
		break;


		//2024-11-04 a wierd bug on function group1.  7003 is a long address but we are transmitting only the >>8 part of this
		//and withought |= 0b11000000
		//and then 3 is being treated as a long address.


	case DCC_FUNCTION:
		//function packets are controlled by funcIndex and transmit at 1/3rd rate of loco
	{//block start
		uint8_t func_Value;
		uint8_t func_LocoIndex = m_funcIndex / 3;

		if (loco[func_LocoIndex].address != 0) {
			/*2019-10-08 support long address*/
			if (loco[func_LocoIndex].useLongAddress) {
				/*long address format S9.2.1 para 60*/
				DCCpacket.data[0] = loco[func_LocoIndex].address >> 8;   //upper byte of address
				DCCpacket.data[0] |= 0b11000000;
				DCCpacket.data[1] = loco[func_LocoIndex].address & 0x00FF;   //lower byte of address
				i = 2;
			}
			else {
				DCCpacket.data[0] = (loco[func_LocoIndex].address & 0x7F);
				i = 1;
			}

			/*i will point to next data[] element we write to*/

			switch (m_funcIndex % 3) {
			case 1:
				/*send a function group 2 packet S-9.2.1 para 270 101SDDDD*/
				func_Value = (loco[func_LocoIndex].function >> 5) & 0b1111;
				func_Value |= 0b10110000;
				break;
			case 2:
				/*send a function group 3 packet S-9.2.1 para 270 101SDDDD*/
				func_Value = (loco[func_LocoIndex].function >> 9) & 0b1111;
				func_Value |= 0b10100000;
				break;
			default:
				/*send a function group 1 packet S-9.2.1 para 260 100DDDDD*/
				/*take 5 function bits and move bit 0 to bit 4*/
				func_Value = loco[func_LocoIndex].function & 0b11111;
				if (func_Value & 0x01) { func_Value = func_Value | 0b100000; }
				func_Value = func_Value >> 1;
				func_Value |= 0b10000000;
			}
			DCCpacket.data[i] = func_Value;
			i++;

			/*calc checksum and packet length. i points to checksum byte*/
			DCCpacket.data[i] = 0;
			for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
				DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
			}
			DCCpacket.packetLen++;
			/*will exit with DCCpacket.packetLen set at correct length of i+1*/

		}

		/*increment funcIndex and rollover at 3*max_loco. Next up is loco packet*/
		++m_funcIndex;
		if (m_funcIndex >= MAX_LOCO * 3) { m_funcIndex = 0; }
		dccSE = DCC_LOCO;
	}//block end
	break;

	case DCC_POM:
		m_pom.packetCount -= m_pom.packetCount > 0 ? 1 : 0;
		if (m_pom.packetCount != 0) { break; }
		/*S9.2.1 configuration variable access instruction, long form. para 375 aka POM
		addr [1 or 2] 1110CCVV 0 VVVVVVVV 0 DDDDDDDD [checksum] max 6 byte packet.  two packets must
		be received for the decoder to act. any broadcast or other decoder specific packet in between
		will cancel the write.  so in theory we write three identical packets and this should work.*/

		//2020-6-17 can also write to accessory 10AAAAAA 0 1AAACDDD address see S9.2.1 para 465

		switch (m_pom.state) {

		case POM_BYTE:
		case POM_BIT:
			/*whilst user is editing POM values, keep transmitting loco bytes*/
			dccSE = DCC_LOCO;
			break;


		case POM_BYTE_WRITE:
			//set to repeat packet transmit 4 times
			m_pom.packetCount = 4;
			//2020-06-17 add accessory support
			if (m_pom.useAccessoryAddr) {
				/*accessory address format S9.2.1 para 420  10AAAAAA 0 1AAACDDD, where CDDD=0000 is the only type
				supported.  first byte is 6 lsb, second holds 3 msb but inverted.
				Note that turnouts are grouped in 4 off a single base address.  i.e. addr 1-4 is a single controller
				addr 5-9 maps to a second.  so if we set CV 7 on  addr 1-4, since we send CDDD=0000 it applies to the
				whole controller.  whereas it is possible that each turnout has a CV7, and these can be individually addressed
				only its not done. the controller has one CV7 instance, and if you write 1-4 that single CV7 will be changed.*/
				DCCpacket.data[0] = m_pom.address & 0b00111111;
				DCCpacket.data[0] |= 0b10000000;
				DCCpacket.data[1] = ~(m_pom.address >> 1);
				DCCpacket.data[1] &= 0b01110000;
				DCCpacket.data[1] |= 0b10000000;

				DCCpacket.data[0] = (m_pom.address >> 3) & 0b00111111;
				DCCpacket.data[0] |= 0b10000000;
				DCCpacket.data[1] = ~(m_pom.address >> 4);
				DCCpacket.data[1] &= 0b01110000;
				DCCpacket.data[1] |= 0b10000000;

				i = 2;
			}
			else if (m_pom.useLongAddr) {
				/*long address format S9.2.1 para 60*/
				DCCpacket.data[0] = m_pom.address >> 8;
				DCCpacket.data[0] |= 0b11000000;
				DCCpacket.data[1] = m_pom.address & 0x00FF;
				i = 2;
			}
			else {
				DCCpacket.data[0] = (m_pom.address & 0x7F);
				i = 1;
			}

			/*CV#1 is transmitted as zero*/
			DCCpacket.data[i] = (m_pom.cvReg - 1) >> 8;
			DCCpacket.data[i] |= 0b11101100;  //byte write CC=11
			i++;
			DCCpacket.data[i] = (m_pom.cvReg - 1) & 0xFF;
			i++;
			DCCpacket.data[i] = m_pom.cvData;
			i++;
			/*calc checksum and packet length. i points to checksum byte*/
			DCCpacket.data[i] = 0;
			for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
				DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
			}
			DCCpacket.packetLen++;
			/*will exit with DCCpacket.packetLen set at correct length of i+1*/
			//m_pom.state = POM_BYTE;
			m_pom.state = POM_WAIT_CTRL;
			break;

		case POM_WAIT_CTRL:
			//intiate a railcom read for ctrl char, if required
			//railcomRead(true);
			m_pom.state = POM_BYTE;
			break;


		case POM_BIT_READ:   //2024-12-05 take same byte read action
		case POM_BYTE_READ:  //2024-05-04
			//2024-11-02 note, S9.2.1 para 375 only defines write bit/byte and verify bit/byte
			//to read a byte we need to refer to S9.3.2 para 5.1.1 but actually the read 1 byte command is actually the byte verify command with data=0

			//Byte read: addr [1 or 2]  111001VV 0 VVVVVVVV 0 00000000 [checksum] where V=cv
			//Bit read 


			//If railcom is enabled, the decoder can send "operations mode acknowledgment".  See para 495, which refers to S9.3.1(no longer exists) and S9.3.2.(railcom)
			// except "operations mode acknowledgment" does not appear in that doc.  But see p19 of that doc, POM section, it does define an ACK packet.

			/*findings. you must turn off auto discovery in cv28, as this randomly puts a series of 6 0x0F bytes in ch2.
			you must turn off broadcast on ch1, because this puts random non 4-8 bytes on the rc cutout
			you must turn on broadcast on ch2, and here you will see a 2 byte (12 bit) datagram which is the returned ID=0 and cv value
			and the orientation of the loco seems to affect the ch1 garbage, however the ch2 stuff still reads correclty in either orientation
			tests also prove that with all the other traffic for loco 6830, such as speed and func, it still responds to POM read correctly.
			*/

			//2025-01-15 BUG/LIMITATION. If the addressed loco is not in the roster, the read is less reliable presumably because once the POM command is sent
			//there are no subsequent packets addressed to that loco, so it can never respond.

			m_pom.packetCount = 10;  //increase from 4 to 10.  Each packet takes around 8 mS.  This did not help much.  Better if the loco is in the roster.
			

			//2026-01-03 should we send the POM command 10 times? surely the decoder is trying to respond to it


			if (false) {
				//placeholder for accessory read
			}
			else if (m_pom.useLongAddr) {
				/*long address format S9.2.1 para 60*/
				DCCpacket.data[0] = m_pom.address >> 8;
				DCCpacket.data[0] |= 0b11000000;
				DCCpacket.data[1] = m_pom.address & 0x00FF;
				i = 2;

			}
			else {
				DCCpacket.data[0] = (m_pom.address & 0x7F);
				i = 1;
			}
			/*CV#1 is transmitted as zero*/
			DCCpacket.data[i] = (m_pom.cvReg - 1) >> 8;
			DCCpacket.data[i++] |= 0b11100100;  //byte verify C=01
			DCCpacket.data[i++] = (m_pom.cvReg - 1) & 0xFF;
			DCCpacket.data[i++] = 0;  //send zero in place of a cv value.  This is per S9.3.2 though it contradicts S9.2.1 para 390
			/*calc checksum and packet length. i points to checksum byte*/
			DCCpacket.data[i] = 0;
			for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
				DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
			}
			DCCpacket.packetLen++;
			
			/*will exit with DCCpacket.packetLen set at correct length of i+1*/
			m_pom.state = m_pom.state== POM_BYTE_READ ? POM_BYTE : POM_BIT;
			//revert to transmitting packets, i.e. this packet gets sent
	
			//2025-02-17 bug fix; create the loco address in the roster if it does not exist.  This makes reading more reliable because more railcom cutouts 
			//addressed to that loco will be sent

			//2025-11-16 there is a bug created by this; we seem to end up with multiple loco S3 created in the free slots.  This causes erratic behaviour because they all
			//transmit their speed/dir/func to the track.. not fixed yet
			
			{//scope block start RAILCOM READ
				char buffer[10];
				char existing[10];
				sprintf(buffer, "S%d", m_pom.address);
				buffer[0] = m_pom.useLongAddr ? 'L' : 'S';   
				int8_t theSlot = findLoco(buffer, existing);
				//kick off the read process, which returns a result via DCCcore::railcomCallback()
				railcomRead(false);

				//pick up a zero slot or bump one. POM read is only reliable if the loco is in the roster
				if (theSlot == -1) break;
				if (m_pom.useAccessoryAddr) break;  //not supported at this time
				if ((strcmp(existing, buffer) == 0) && (loco[theSlot].address != 0)) {
					//loco exists in roster, no further action
					break;
				}
				else {
					//add loco to roster
					loco[theSlot].clear();
					loco[theSlot].address = m_pom.address;
					loco[theSlot].useLongAddress = m_pom.useLongAddr;
					//loco[theSlot].speed = 0;
					//loco[theSlot].speedStep = 0;
					//loco[theSlot].use128 = true;
					//memset(loco[theSlot].name, '\0', sizeof(loco[theSlot].name));
					//write back to eeprom
					bootController.isDirty = true;  //pending write
					bootController.flagLocoRoster = true;
					//2021-09-01 increment age
					incrLocoHistory(&loco[theSlot]);

				}

			}//scope block end
			break;

		case POM_BIT_WRITE:
			m_pom.packetCount = 4;
			if (m_pom.useLongAddr) {
				/*long address format S9.2.1 para 60
				14bit address, so shift 8 means 6msbit are used*/
				DCCpacket.data[0] = m_pom.address >> 8;
				DCCpacket.data[0] |= 0b11000000;
				DCCpacket.data[1] = m_pom.address & 0x00FF;
				i = 2;
			}
			else {
				DCCpacket.data[0] = (m_pom.address & 0x7F);
				i = 1;
			}
			/*CV#1 is transmitted as zero*/
			DCCpacket.data[i] = (m_pom.cvReg - 1) >> 8;
			DCCpacket.data[i] |= 0b11101000;  //bit manipulation CC=10   
			i++;
			DCCpacket.data[i] = (m_pom.cvReg - 1) & 0xFF;
			i++;
			/*111CDBBB  C=1 for write, D is the bit value, BBB bit pos para 405*/
			DCCpacket.data[i] = 0b11110000;  //would be 11100000 for bit verify
			DCCpacket.data[i] |= (m_pom.cvBit & 0b111);
			if (m_pom.cvBit >= 128) {
				DCCpacket.data[i] |= 0b1000;
			}
			i++;
			/*calc checksum and packet length. i points to checksum byte*/
			DCCpacket.data[i] = 0;
			for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
				DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
			}
			DCCpacket.packetLen++;
			/*will exit with DCCpacket.packetLen set at correct length of i+1*/
			m_pom.state = POM_BIT;
			break;
		}

		//2024-05-06 send a cutout at end of any/all POM packet
		DCCpacket.doCutout = true;

		break; //end POM case


	case DCC_SERVICE:
		/*service mode, the makes use of a state engine within the SM struct see S9.2.3*/
		power.serviceMode = true;
		m_sm.packetCount -= m_sm.packetCount > 0 ? 1 : 0;
		if (m_sm.packetCount != 0) { break; }

		switch (m_sm.state) {
		case PG_START:
			/*PAGED CV ADDRESSING para 205. 3 or more reset packets*/
			DCCpacket.longPreamble = true;
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			DCCpacket.packetLen = 3;
			m_sm.state = PG_PG_WRITE;
			break;

		case PG_PG_WRITE:
			m_sm.packetCount = 6;
			/* 5 or writes to page register
			 * long-preamble 0 0111CRRR 0 DDDDDDDD 0 EEEEEEEE 1
			 * C=1 for write RRR=101 for page*/
			DCCpacket.data[0] = 0b01111101;
			DCCpacket.data[1] = m_sm.pgPage;
			DCCpacket.data[2] = DCCpacket.data[0] ^ DCCpacket.data[1];
			m_sm.state = PG_RESET;
			break;

		case PG_RESET:
			/*6 or more reset packets*/
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			m_sm.state = PG_WRITE;
			break;

		case PG_WRITE:
			/*5 or more writes to data register 1-4*/
			m_sm.packetCount = 6;
			DCCpacket.data[0] = m_sm.pgReg | 0b01111000;
			DCCpacket.data[1] = m_sm.cvData;
			DCCpacket.data[2] = DCCpacket.data[0] ^ DCCpacket.data[1];
			m_sm.state = PG_RESET_2;
			break;

		case PG_RESET_2:
			/* 10 reset packets */
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			m_sm.state = CV_IDLE;
			break;

		case D_START:
			/*DIRECT WRITE para 100.  3 or more reset packets
			* Para 55 of s-9.2.3, for write, ACK should not occur before until data-storage is complete. Controller should scan
			* for ACK window starting at end of second service mode instruction packet and through the required number of instruction packets and through
			* the recovery time.
			*
			*2026-07-01 policy now, is to call ina219mode(false) prior to initiating D_START
			*/
			DCCpacket.longPreamble = true;
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			DCCpacket.packetLen = 3;
			m_sm.state = D_WRITE;
			break;

		case D_WRITE:
			/*5 or more writes
			* in theory we could start looking for ACK immediately after 2nd write.  Each packet is around 10mS long and 
			* one shot sample is about 60mS and we need to see ACK for about 30mS within that.   So its possible ACK could occur entirely within D_WRITE
			* or overlapping and into D_RESET.
			*/
			m_sm.packetCount = 6;
			/*Long-preamble 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
			 *CC=11 for byte write S9.2.3 para 110
			 *the CV target AAAAAAAAAA target is 0 for CV1, etc
			 */
			DCCpacket.data[0] = ((m_sm.cvReg - 1) >> 8) | 0b01111100;
			DCCpacket.data[1] = (m_sm.cvReg - 1) & 0xFF;
			DCCpacket.data[2] = m_sm.cvData;
			DCCpacket.data[3] = DCCpacket.data[0] ^ DCCpacket.data[1] ^ DCCpacket.data[2];
			DCCpacket.packetLen = 4;
			m_sm.state = D_RESET;
			ina219Mode(false);  //2026-01-07 new, kick off another ack read
			break;

		case D_RESET:
			/*6 or more write or reset packets.  This is the decoder recovery time*/
			//did we see ACK already during D_WRITE?
			power.ackFlag = ((ina219.getCurrent_mA() - power.ackBase_mA) > 20) ? true : false;

			ina219Mode(false);  // kick off another ack read
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			DCCpacket.packetLen = 3;
			m_sm.state = D_ACK;  //rollback
			//m_cv.state = CV_IDLE;
			break;

		case D_ACK:   //rolled back
			//2026-1-7 look for ACK, but OR this with prior result during D_RESET.  This fixed the issue of detecting ACK
#ifdef USE_ANALOG_MEASUREMENT
/*when detecting a 6mS current pulse using the AD converter, this is done outside of this routine
on a 1mS sample frequency. This routine won't be called again until the RD_RESET packet has been
sent out*/
#else
			//read the result of the one-time sample. was 128 samples, and we need one third  of these to be >55mA up-pulse
			power.ackFlag |= ((ina219.getCurrent_mA() - power.ackBase_mA) > 20) ? true : false;
#endif		

			//revert to average mode, and invoke a callback
			//2026-01-06 BUG. we see ACK after one write, but then any subsequent read fails unless you exit SM entirely and re-enter. why?
			
			ina219Mode(true); //restore averaging mode
			if (LocoNetcallbackPtr) LocoNetcallbackPtr(power.ackFlag,m_sm.cvData);
			m_sm.state = CV_IDLE;
			break;

		case CV_IDLE:
			DCCpacket.longPreamble = false;
			DCCpacket.data[0] = 0xFF;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0xFF;
			DCCpacket.packetLen = 3;
			break;

		case RD_START:
			/*for CV read, we will read 8 bits in turn, followed by a byte-read to confirm the
			inferred byte value is correct.   Means we repeat the verify bit sequence 8 times over
			with 1 following byte verify.
			/*DIRECT MODE para 100.  3 or more reset packets*/
			//2026-01-07 policy now is to call ina219mode(false) prior to setting RD_START

			DCCpacket.longPreamble = true;
			m_sm.packetCount = 10;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			DCCpacket.packetLen = 3;
			m_sm.state = RD_VERIFY;
			break;

		case RD_VERIFY:  //RD_command
			/*5 or more verifies*/
			m_sm.packetCount = 6;
			/*Long-preamble 0 011110AA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1
			Where BBB represents the bit position within the CV (000 being defined as bit 0)
			and D contains the value of the bit to be verified or written, K=1 write-bit
			K=0 is verify bit. see S9.2.3 para 130
			the CV target AAAAAAAAAA target is 0 for CV1, etc
			 */
			if (m_sm.bitCount >= 0) {
				DCCpacket.data[0] = ((m_sm.cvReg - 1) >> 8) | 0b01111000;
				DCCpacket.data[1] = (m_sm.cvReg - 1) & 0xFF;
				DCCpacket.data[2] = 0b11101000 | (m_sm.bitCount & 0b111);
				DCCpacket.data[3] = DCCpacket.data[0] ^ DCCpacket.data[1] ^ DCCpacket.data[2];
				DCCpacket.packetLen = 4;
				m_sm.state = RD_RESET;
			}
			else {
				/*byte verify the data we pulled bitwise earlier
				Long-preamble 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
				CC=01 Verify byte	*/
				DCCpacket.data[0] = ((m_sm.cvReg - 1) >> 8) | 0b01110100;
				DCCpacket.data[1] = (m_sm.cvReg - 1) & 0xFF;
				DCCpacket.data[2] = (m_sm.cvData & 0xFF);
				DCCpacket.data[3] = DCCpacket.data[0] ^ DCCpacket.data[1] ^ DCCpacket.data[2];
				DCCpacket.packetLen = 4;
				m_sm.state = RD_RESET;
			}
			//start looking for ACK pulse. Per the S-9.2.3 para 55, this can occur anytime from the second command packet
			//onward to end of recovery time.
			ina219Mode(false);  //kick off another one-time 69mS sample

			break;

		case RD_RESET:
			/*6 or more reset packets, during which we look for the ACK pulse.  This should occur
			within the 50mS it takes to issue reset packets. Might it happen sooner during RD_VERIFY?*/
			m_sm.packetCount = 8;
			DCCpacket.data[0] = 0x00;
			DCCpacket.data[1] = 0x00;
			DCCpacket.data[2] = 0x00;
			DCCpacket.packetLen = 3;
			m_sm.state = RD_FINAL;
			break;

		case RD_FINAL:
	#ifdef USE_ANALOG_MEASUREMENT
			/*when detecting a 6mS current pulse using the AD converter, this is done outside of this routine
			on a 1mS sample frequency. This routine won't be called again until the RD_RESET packet has been
			sent out*/
#else
			//read the result of the one-time sample. was 128 samples, and we need one third  of these to be >55mA up-pulse
			power.ackFlag = ((ina219.getCurrent_mA() - power.ackBase_mA) > 20) ? true : false;
#endif			
			//read the ack pulse as sampled, 80mS+ will have passed since RD_RESET entered
			if (m_sm.bitCount >= 0) {
				//bits 7 thro to 1
				m_sm.cvData = m_sm.cvData << 1;
				if (power.ackFlag) { m_sm.cvData++; }
				m_sm.bitCount--;
				m_sm.state = RD_START;
				//note, if we exit -1 then loop will do a byte verify text next
				break;
			}
			else {
				//arrive here with -1 and ackFlag true if we have correctly pulled the whole byte
				if (power.ackFlag) {
					m_sm.cvData &= 0xFF;
					trace(Serial.println(F("verify good"));)
				}
				else
				{//byte verify failed, indicate we have unknown value
					trace(Serial.printf("verify fail %d \n\r", m_sm.cvData);)
						m_sm.cvData = -1;
				}
				m_sm.state = CV_IDLE;
				//exit with _cv.bitCount still at -1
				//revert to averaging mode
				ina219Mode(true);

				//2019-12-02 not very elegant, but update display here else it won't happen
				//unless we code as a timeout elsewhere
				updateServiceModeDisplay();
				//2020-12-23 call DCCweb to send a read result over the websocket
				trace(Serial.printf("reg %d val %d\n\r", m_sm.cvReg, m_sm.cvData);)
				nsDCCweb::broadcastSMreadResult(m_sm.cvReg, m_sm.cvData);
								
				//invoke the callback function which was passed to actionPCMDfromLoconet()
				if (LocoNetcallbackPtr) LocoNetcallbackPtr(power.ackFlag,m_sm.cvData);
			}
			

		}

		break;


	case DCC_ESTOP:
		//broadcast an eStop packet, set all locos to zero speed. see S=9.2.1 para 50 and para 100
		DCCpacket.longPreamble = false;
		DCCpacket.data[0] = 0x00;
		DCCpacket.data[1] = 0b01000001;
		DCCpacket.data[2] = 0b01000001;
		DCCpacket.packetLen = 3;

		//2021-9-1 bug fix
		//my TCS decoder does not respond to broadcast eStop. my Zen and NCE decoders do
		//so, send just one eStop broadcast packet and then set all loco slots to eStop 
		//as belt-and-braces
		for (auto& loc : loco) {
			loc.speed = 0;
			loc.speedStep = 0;
			//note a non-zero eStopTimer lets the dcc packet engine know to transmit an estop message
			loc.eStopTimer = LOCO_ESTOP_TIMEOUT;
			//flag a change so this gets broadcast over all channels
			loc.changeFlag = true;
		}

		/*restore any power trip condition*/
		/*2020-03-29 restore power if it was turned off remotely*/

		if (power.trip || !power.trackPower) {
			power.bus_mA = 50;  //emulate 50mA and 5v to avoid retriggering a trip from held value
			power.bus_volts = 5;
			power.trip = false;
			power.trackPower = true;
			ina219Mode(true);  //use averaging mode (trip may have been during SM read)
		}

		dccSE = DCC_LOCO;
		break;

	case DCC_IDLE:
		DCCpacket.longPreamble = false;
		DCCpacket.data[0] = 0xFF;
		DCCpacket.data[1] = 0x00;
		DCCpacket.data[2] = 0xFF;
		DCCpacket.packetLen = 3;
		break;


	case DCC_IMMEDIATE:
		//we have to hold the loconet data in its own buffer as the loconet call might occur during a period when
		//CTS is low and DCCpacket is not allowed to change.  We copy over this buffered data when CTS is high.
		//m_LocoNetHoldingBuffer.railcomPacketCount is repurposed as a generic repeat counter.  0 represents a single
		//transimission i.e. zero repeats.

		if (m_LocoNetHoldingBuffer.railcomPacketCount == 0) dccSE = DCC_LOCO;

		m_LocoNetHoldingBuffer.railcomPacketCount -= m_LocoNetHoldingBuffer.railcomPacketCount == 0 ? 0 : 1;
		DCCpacket.longPreamble = false;
		DCCpacket.data[0] = m_LocoNetHoldingBuffer.data[0];
		DCCpacket.data[1] = m_LocoNetHoldingBuffer.data[1];
		DCCpacket.data[2] = m_LocoNetHoldingBuffer.data[2];
		DCCpacket.data[3] = m_LocoNetHoldingBuffer.data[3];
		DCCpacket.data[4] = m_LocoNetHoldingBuffer.data[4];
		DCCpacket.data[5] = m_LocoNetHoldingBuffer.data[5];
		DCCpacket.packetLen = m_LocoNetHoldingBuffer.packetLen;
		trace(debugPacket();)  //DEBUG
		break;
	

	case DCC_ACCESSORY:
		/* basic packet is  {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1, see S-9.2.1 para 420
		 * For turnouts its effectively an 11 bit address 10AAAAAA 1AAACAAT
		 * C is 1 for power on, 0 off. Used for signal aspects.  Turnouts generally act on the 1 instruction.
		 * D in <0> (or T in my notation) indicates which half of the pair is activated.
		 * The most significant bits of the  address are bits <6-4> of the second data byte. By convention these bits in the
		 * second data byte are in ones complement.  2018-10-02 re-written to fix bugs
		 * valid UI addresses are 1 through 4096, but 1 effectively maps to 4dec in the address space
		 * i.e. no one talks about turnouts 1-511 (9bit) with each addressing output 1-4, instead everyone refers
		 * to a continous range of 1 to 2047 (11bit) in the UI, where the first turnout controller responds to 1-4, the next 5-9 etc
		 * but you need to be mindful that the controllers 'base address' is in the range 1-511 when you set it up
		 * i.e. turnout 1 is 00000000100 and turnout 4 is 00000000111 this is not made clear in the specification
		 */
	{//scope block
		uint16_t a = accessory.address + 3;

		//2021-01-04 address space is 9 bits followed by 2 bits of device pair id
		//data[0] is therefore the lsb from a>>2
		DCCpacket.data[0] = (a >> 2) & 0b111111; //shift and mask in 6 bits
		DCCpacket.data[0] |= 0b10000000;  //set <7>

		//device identifier is <2-1> of data[1]
		DCCpacket.data[1] = (a << 1) & 0b00000110;
		//data[1] <6-4> holds the address msb, i.e. a<10-8> become data[1]<6-4>
		DCCpacket.data[1] |= (a >> 4) & 0b01110000;
		//complement bits <6-4>
		DCCpacket.data[1] ^= 0b01110000;
		//DCCpacket.data[1] |= 0b10001000; //set <7> marker and <3> activate bit.  2026-02-05 <3> is set below
		DCCpacket.data[1] |= 0b10000000; //set <7> marker

		//if thrown, set <0>
		if (accessory.thrown) DCCpacket.data[1]++;

		//2026-02-05 if powerOn then set C bit
		DCCpacket.data[1] |= accessory.powerOn ? 0b1000 : 0;

		DCCpacket.data[2] = DCCpacket.data[0] ^ DCCpacket.data[1];
		DCCpacket.packetLen = 3;

		//2021-1-4 debug dump the packets
		trace(Serial.printf("ACC packets %d %d\r\n", DCCpacket.data[0], DCCpacket.data[1]);)

			dccSE = DCC_LOCO;
		break;
	}//scope block

	}//end switch


}  //end of function



#pragma region local_hardware_interface


/*select and toggle a turnout.  Only the first 8 slots are available on the hardware UI.
Also only addresses 1 through 99 can be operated in the UI.
 returns -2 if no change. If turnout was toggled, returns turnout slot modified*/
int8_t setTurnoutFromKey(KEYPAD& k) {
	uint8_t i;
	if (k.key == 27) { m_turnoutSE = TURNOUT_DIGIT_1; return -2; }//estop
	if (k.key == 28) { m_turnoutSE = TURNOUT_DIGIT_1; return -2; }//mode

	/*Process valid keys, clearing keyFlag and ignore key repeats*/
	/*keyUp events are also processed*/

	trace(Serial.println(F("setTurnoutFromKey"));)
		trace(Serial.println(k.key);)

		/*handle digit 0-9, and also accept k.key==0*/
		if ((k.keyASCII >= '0' && k.keyASCII <= '9') || k.key == 0) {

			switch (m_turnoutSE) {

			case TURNOUT_DIGIT_1_WAIT:
			case TURNOUT_DIGIT_1:
				if (k.key == 0) {
					m_turnoutSE = TURNOUT_DIGIT_2_WAIT;
					break;
				}
				trace(Serial.println(F("digit 1"));)
					m_turnoutEnteredAddress = k.keyASCII - 48;
				k.requestKeyUp = true;
				trace(Serial.println(F("requestKeyUp"));)
					m_turnoutSE = TURNOUT_DIGIT_1;
				break;


			case TURNOUT_DIGIT_2_WAIT:
			case TURNOUT_DIGIT_2:
				if (k.key != 0) {
					/*if no key up, just present the digit, do not process it until user lifts their finger*/
					trace(Serial.println(F("digit 2"));)
						m_turnoutEnteredAddress *= 10;
					m_turnoutEnteredAddress += k.keyASCII - 48;
					//find selected address, or assign one
					trace(Serial.println(F("m_turnoutEnteredAddress "));)
						trace(Serial.print(m_turnoutEnteredAddress, DEC);)
						k.requestKeyUp = true;
					m_turnoutSE = TURNOUT_DIGIT_2;

					if (m_turnoutEnteredAddress == 0) {
						m_turnoutSE = TURNOUT_DIGIT_1; // 00 is not a valid address
						return -2;
					}

					/*we are done, next step is update UI*/
					break;
				}


				/*saw key-up event. So process the entered address
				_turnoutEnteredAddress is static and will still hold the address as based on last digit pressed*/

				trace(Serial.println(F("check history"));)

					//2021-02-04 refactored. the returned slot i value is not used
					i = findTurnout(m_turnoutEnteredAddress);

				/*note no need to assign to zero slots as these will be picked up from the history*/
				m_turnoutSE = TURNOUT_TOGGLE;
				break;

			case TURNOUT_TOGGLE:
				/* if a numeric is pressed, go back into input mode, but we also need to clear any selected flag */
				m_turnoutEnteredAddress = k.keyASCII - 48;
				m_turnoutSE = TURNOUT_DIGIT_1;  //have received a key
				k.requestKeyUp = true;

				for (i = 0; i < 8; i++) { turnout[i].selected = false; }
				break;

			}//end switch

			return -2;
		}//end 0-9 or key==0 test

	uint8_t r;
	if (k.keyASCII == '#' && m_turnoutSE == TURNOUT_TOGGLE)
	{
		/*find selected turnout, toggle it*/
		for (i = 0; i < MAX_TURNOUT; i++) {
			if (turnout[i].selected) {
				turnout[i].thrown = !turnout[i].thrown;

				/*2020-05-18 modified.  Set a flag, updateLocalDisplay will queue the transmission*/
				turnout[i].changeFlag = true;
				r = i;
			}
		}
		return r;
	}

	//2021-01-10 ABCD buttons now perform a direct toggle on slots 0-3
	if (k.keyASCII >= 'A' && k.keyASCII <= 'D' && m_turnoutSE == TURNOUT_TOGGLE) {
		for (i = 0; i < MAX_TURNOUT; ++i) {
			turnout[i].selected = false;
		}

		i = k.keyASCII - 'A';
		turnout[i].selected = true;
		turnout[i].thrown = !turnout[i].thrown;
		turnout[i].changeFlag = true;
		r = i;
	}
	return r;


}//end func



/// <summary>
/// Find index slot of a given turnout address, or assign a new one
/// </summary>
/// <param name="turnoutAddress">DCC address of turnout</param>
/// <returns>integer slot that exists, or the newly assigned one</returns>
int8_t findTurnout(uint16_t turnoutAddress) {
	/*find the age of oldest history, and capture the oldest slot.  clear any selected flag*/
	/*slots with a zero address, i.e. blank, should be assigned first*/
	uint8_t age = 0;
	uint8_t oldestSlot = 0;
	uint8_t i;
	uint8_t r;
	
	/*clear all selected flags, find oldest slot*/
	for (i = 0; i < MAX_TURNOUT; i++) {
		turnout[i].selected = false;
		if (turnout[i].history > age) { 
			age = turnout[i].history; 
			oldestSlot = i; }
	}

	//scan addresses, is there a match to an existing slot?
	for (i = 0; i < MAX_TURNOUT; i++) {
		if (turnout[i].address == turnoutAddress)
		{
			turnout[i].selected = true;
			break;
		}
	}

	//if we fail to find an existing slot, assign first available zero slot
	//2025-10-27 bug fix, .selected needed to be set
	if (i == MAX_TURNOUT) {
		for (i = 0; i < MAX_TURNOUT; i++) {
			if (turnout[i].address == 0) {
				turnout[i].address = turnoutAddress;
				turnout[i].selected = true;   
				//default name is the numeric address
				snprintf(turnout[i].name, 8, "%d", turnout[i].address);
				bootController.flagTurnoutRoster = true;
				break;
			}
		}
	}

	//else assign one based on history, effectively we bump an old slot
	if (i == MAX_TURNOUT) {
		turnout[oldestSlot].address = turnoutAddress;
		turnout[oldestSlot].selected = true;
		turnout[oldestSlot].history = 0;
		bootController.flagTurnoutRoster = true;
		//overwrite the name with the numeric address
		snprintf(turnout[oldestSlot].name, 8, "%d", turnout[i].address);
	}

	//at this point we always have a slot selected, increment history of all other items
	for (i = 0; i < MAX_TURNOUT; i++) {
		if (!turnout[i].selected)
			{turnout[i].history++; }
		else
			{r = i;}
	}
	return r;
}



/*2019-11-25 re-write to include bit mainpulation and cleaner value edits
returns true if triggering a display update
2020-12-21 it only supports byte write*/
bool setServiceModefromKey(void) {
	LocoNetcallbackPtr = nullptr;
	uint8_t i;
	if (keypad.keyHeld) { return false; }
	if (keypad.keyASCII >= '0' && keypad.keyASCII <= '9') {
		//write digit pos and advance
		uint16_t d = m_sm.cvReg;
		if (m_sm.digit < 4) {
			changeDigit(keypad.keyASCII, 3 - m_sm.digit, &d);
			m_sm.cvReg = d;
			if (m_sm.cvReg > 1024) { m_sm.cvReg = 1024; }
			if (m_sm.cvReg == 0) { m_sm.cvReg = 1; }
		}
		else {
			/*2019-12-01 cvData==-1 if not valid from a verify*/
			if (m_sm.cvData == -1) {
				d = 0;
			}
			else {
				d = m_sm.cvData;
			}
			changeDigit(keypad.keyASCII, 6 - m_sm.digit, &d);
			m_sm.cvData = d;
			if (m_sm.cvData > 255) { m_sm.cvData = 255; }
		}


		/*increment digit position but do not wrap*/
		m_sm.digit += m_sm.digit < 6 ? 1 : 0;
		/*update page and reg*/
		int v = m_sm.cvReg - 1;
		m_sm.pgPage = v / 4 + 1;
		m_sm.pgReg = v % 4;
		return true;


	}
	if (keypad.keyASCII == '*') {
		m_sm.digit -= m_sm.digit > 0 ? 1 : 0;
	}
	if (keypad.keyASCII == '#') {
		m_sm.digit += m_sm.digit < 6 ? 1 : 0;
	}


	/*Direct write byte*/
	if (keypad.keyASCII == 'D') {
		if (m_sm.cvData < 0) { return true; }
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		m_sm.state = D_START;
	}

	/*Page write byte*/
	if (keypad.keyASCII == 'C') {
		if (m_sm.cvData < 0) { return true; }
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		m_sm.state = PG_START;
	}

	/*Read byte using direct mode*/
	if (keypad.keyASCII == 'A') {
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		//change to trigger mode and take one sample to initialise the ina
		ina219Mode(false);  
		m_sm.state = RD_START;
		m_sm.bitCount = 7;
		m_sm.cvData = 0;
		return false;
	}

	if (keypad.keyASCII == 'B') {
		return true;
	}
	/*default return is to request a screen update*/
	return true;
}

uint8_t m_powerDigit;
void setPowerFromKey(void) {
	/*A,B toggles cursor to mA or volts, numeric values entered and wrap
	mode will exit. changes are written to eeprom*/
	switch (keypad.keyASCII) {
	case 'A':
		m_powerDigit = 0;
		return;
	case 'B':
		m_powerDigit = 4;
		return;
	}
	if (keypad.keyASCII < '0' || keypad.keyASCII>'9') { return; }
	/*accept 0-9 only, set the active digit*/
	/*powerDigit points to current digit posn*/
	if (m_powerDigit < 4) {
		/*current limit; 4 digits, zero digit indx 3*/
		changeDigit(keypad.keyASCII, 3 - m_powerDigit, &bootController.currentLimit);
		/*move or wrap*/
		m_powerDigit += m_powerDigit == 3 ? -3 : 1;
		if (bootController.currentLimit > 4000) { bootController.currentLimit = 4000; }
		if (bootController.currentLimit < 250) { bootController.currentLimit = 250; }
	}
	else {
		/*voltage limit; 2 digits, zero digit indx 5*/
		changeDigit(keypad.keyASCII, 5 - m_powerDigit, &bootController.voltageLimit);
		/*move or wrap*/
		m_powerDigit += m_powerDigit == 5 ? -1 : 1;
		if (bootController.voltageLimit > 22) { bootController.voltageLimit = 22; }
		if (bootController.voltageLimit < 10) { bootController.voltageLimit = 10; }
	}
	/*flag pending write to eeprom*/
	bootController.isDirty = true;
}


/// <summary>
/// Execute SM or POM activity from a loconet command
/// </summary>
/// <param name="PCMD"></param>
/// <param name="addr"></param>
/// <param name="cv"></param>
/// <param name="data"></param>
/// <param name="callback"></param>
/// <returns>false if operation unsupported</returns>
bool actionPCMDfromLoconet(uint8_t PCMD, uint16_t addr, uint16_t cv, uint8_t data, void (*callback)(bool, uint8_t)) {
	//2026-01-01 if we have been given a callback address, then ACK is expected.  S-9.3.2 states ACK|NACK|BUSY is only seen on write operations
	//copy the callback function pointer to a static var for use later in the DCCpacketEngine
	
	//LocoNet specification fails to cover how bit operation on POM are acheived.  Potentially its encoded in the data value, e.g. 
	LocoNetcallbackPtr = callback;
	if ((PCMD & 0b100) != 0) {
		//POM block

		m_pom.useAccessoryAddr = false;
		m_pom.address = addr;
		m_pom.useLongAddr = addr > 127 ? true : false;
		m_pom.cvReg = cv + 1;  //m_pom is one-based
		m_pom.cvData = data;
		m_pom.readSuccess = false;

		if (m_pom.address > 10239) { m_pom.address = 10239; }
		if (m_pom.cvReg > 1024) { m_pom.address = 1024; }

		//what kind of POM operation do we need?
		//<6>=write/read <5>=byte/bit <3>=TY1 = feedback/no feedback
		m_pom.expectACK = (PCMD & 0b1000) == 0 ? 0 : 1;

		//[EF 0E 7C 2F 00 01 54 00 00 00 00 7F 7F 18]  Byte Read on Main Track (Ops Mode): Decoder address 212: CV1.
		// [EF 0E 7C 67 00 01 54 00 00 01 03 7F 7F 52] Byte Write(No feedback) on Main Track : Decoder address 212 : CV2 value 3 (0x03, 00000011b).
		//2f=00101111
		//67=01100111
		//yeah but despite DP stating no feedback, it still expects feedback as in a full E7 response rathr than blind no E7 


		if ((PCMD & 0b1000000) == 0) {
			//read
			if ((PCMD & 0b100000) != 0) {
				//byte
				m_pom.state = POM_BYTE_READ;
				dccSE = DCC_POM;  //initiate sequence
				m_pom.timeout = 8; //2 sec and then display will update
				updatePOMdisplay();
				return true;
			}
			else {
				//bit. not supported
				return false;
			}

		}
		else {
			//write
			if ((PCMD & 0b100000) != 0) {
				//byte
				m_pom.state = POM_BYTE_WRITE;
				dccSE = DCC_POM;  //initiate write sequence
				m_pom.timeout = 8;
				updatePOMdisplay();
				return true;
			}
			//bit, not supported.
			return false;

		}
		return false;

	}//end POM
	
	//SERVICE MODE routines.  Note: DCCpacketEngine has to execute callbacks to loconet::asyncLocoNet()
	//enter service mode
	writeServiceCommand(0, 0, false, true, false); 
		
	//what kind of SM operation do we need?
	//<6>=write/read <5>=byte/bit <4,3>=TY1,0 = page mode ,direct, register.  Address is not relevant in SM
	m_sm.cvData = data;
	m_sm.cvReg = cv+1;  //cv values from loconet are base zero.
	
	//bit operations are not supported, nor are page mode
	if ((PCMD & 0b100000) == 0) return false;
	
	//deal with page or direct modes only
	switch (PCMD & 0b1011000) {
	case 0b1000000:  //page mode write
		return false;
	case 0x00: //page mode read
		return false;
	case 0b1001000:  //direct mode write
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		m_sm.state = D_START;
		return true;

	case 0b0001000:  //direct mode read
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		//change to trigger mode and take one sample to initialise the INA
		ina219Mode(false);
		m_sm.state = RD_START;
		m_sm.bitCount = 7;
		m_sm.cvData = 0;
		return true;

	default:
		return false;
		
	}

	return false;
}



void setPOMfromKey(void) {
	//default action is to re-enable numeric/bit display
	//it is only set to ??? when a read is intiated
	LocoNetcallbackPtr = nullptr;
	m_pom.readSuccess = true;

	switch (keypad.keyASCII) {
	case 'A':
		//2024-12-02 initiate a POM read, either bit or byte
		m_pom.readSuccess = false;
		if (m_pom.state == POM_BIT) { m_pom.state = POM_BIT_READ; }
		if (m_pom.state == POM_BYTE) { m_pom.state = POM_BYTE_READ; }
		dccSE = DCC_POM;  //initiate sequence
		m_pom.timeout = 8; //2 sec and then display will update
		return;

		//note, this initiates a POM read which has a 0.5 sec timeout and it either returns with readSuccess true/false
		//based on what came back from the railcom reader.  if valid, the m_pom.cvData and cvBit are updated by the read routine
	

	case 'B':
		//toggle binary/byte
		switch (m_pom.state) {
		case POM_BYTE:
			m_pom.state = POM_BIT;
			break;
		case POM_BIT:
			m_pom.state = POM_BYTE;
		}
		m_pom.digitPos = 0;
		return;
	case 'D':
		//write
		if (m_pom.state == POM_BIT) { m_pom.state = POM_BIT_WRITE; }
		if (m_pom.state == POM_BYTE) { m_pom.state = POM_BYTE_WRITE; }
		dccSE = DCC_POM;  //initiate write sequence
		m_pom.timeout = 8;
		return;
	}

	/*generic left navigation*/
	if (keypad.keyASCII == '*') {
		m_pom.digitPos -= m_pom.digitPos > 0 ? 1 : 0;
		return;
	}

	//debug
	trace(Serial.printf("setPOMkey %d\n", m_pom.state);)




	if (m_pom.state == POM_BYTE) {
		/*bytewise*/

		if (keypad.keyASCII == '#') {
			/*1024-234  L12345 there are 12 digit posns, zero indexed*/
			m_pom.digitPos += m_pom.digitPos < 13 ? 1 : 0;
			return;
		}

		if (keypad.keyASCII < '0' || keypad.keyASCII>'9') { return; }
		if (m_pom.digitPos < 4) {
			changeDigit(keypad.keyASCII, 3 - m_pom.digitPos, &m_pom.cvReg);
		}
		else if (m_pom.digitPos < 7) {
			uint16_t d = m_pom.cvData;
			/*need 16 working register, else entering 999 goes haywire*/
			changeDigit(keypad.keyASCII, 6 - m_pom.digitPos, &d);
			if (d > 255) { d = 255; }
			m_pom.cvData = d & 0xFF;
		}
		else if (m_pom.digitPos == 7)
			/*2020-6-17 address type, toggle through S,L,A based on any numeric key input*/
		{
			if (m_pom.useAccessoryAddr) {
				//toggle back to S
				m_pom.useAccessoryAddr = false;
				m_pom.useLongAddr = false;
			}
			else {
				//neat way to toggle through S,L and into A 
				m_pom.useAccessoryAddr = m_pom.useLongAddr;
				m_pom.useLongAddr = !m_pom.useLongAddr;
			}
		}

		else
		{
			changeDigit(keypad.keyASCII, 12 - m_pom.digitPos, &m_pom.address);
		}
		m_pom.digitPos += m_pom.digitPos < 12 ? 1 : 0;

	}

	else if (m_pom.state == POM_BIT) {/*bitwise 1023-b7-1 L10239*/
		if (keypad.keyASCII == '#') {
			/*1024-b1-1 L10239 there are 11 digit posns, zero indexed*/
			m_pom.digitPos += m_pom.digitPos < 11 ? 1 : 0;
			return;
		}
		if (keypad.keyASCII < '0' || keypad.keyASCII>'9') { return; }
		if (m_pom.digitPos < 4) {
			changeDigit(char(keypad.keyASCII), 3 - m_pom.digitPos, &m_pom.cvReg);
		}
		else if (m_pom.digitPos == 4) {
			/*manipulate bit index*/
			m_pom.cvBit &= 0b10000000;
			m_pom.cvBit += (keypad.keyASCII - 48) & 0b111;
		}
		else if (m_pom.digitPos == 5) {
			/*set/reset bit*/
			m_pom.cvBit &= 0b111;
			m_pom.cvBit += keypad.keyASCII == 48 ? 0 : 0b10000000;
		}
		else if (m_pom.digitPos == 6)
		{
			m_pom.useLongAddr = !m_pom.useLongAddr;
		}
		else
		{
			changeDigit(keypad.keyASCII, 11 - m_pom.digitPos, &m_pom.address);
		}

		m_pom.digitPos += m_pom.digitPos < 12 ? 1 : 0;
	}

	/*final limits checks on reg and address values*/
	/*long address valid to 10239, short to 127. 0 is not valid*/
	if (m_pom.useLongAddr) {
		if (m_pom.address > 10239) { m_pom.address = 10239; }
	}
	else
	{
		if (m_pom.address > 127) { m_pom.address = 127; }
	}
	if (m_pom.address == 0) { m_pom.address = 1; }
	/*registers are displayed as +1 compared to the actual value.  e.g. reg 23 is location 22 in memory*/
	if (m_pom.cvReg > 1024) { m_pom.cvReg = 1024; }
	if (m_pom.cvReg == 0) { m_pom.cvReg = 1; }
	/*cvData limits were set in the routine above*/

}




#pragma  endregion


/// <summary>
/// abstract loco speed and direction as a function.
/// updates history value. using uint16 even with one change per second, it will be 18+ hours before
/// this counter wraps
/// </summary>
/// <param name="loc">loco object</param>
/// <param name="speed">+1 or -1 or 0 for no change</param>
/// <param name="dir">true will cause direction to toggle if loco is stationary</param>
/// <returns>returns the loco[] array index if successful or -1 otherwise.
/// Also -1 if we are operating with a temp LOCO object.
/// </returns>
int8_t setLoco(LOCO* loc, int8_t speed, bool dir) {
	if (loc == nullptr) return -1;
	trace(Serial.println(F("\nsetLoco\n"));)

		//estop condition is blocking, meaning no changes to dir or speed
		if (loc->eStopTimer == 0) {
			//find max of history


			if (speed > 0) {
				//incr speed
				if (loc->use128) {
					loc->speedStep += (loc->speedStep < 126) ? 1 : 0;
				}
				else {
					loc->speedStep += (loc->speedStep < 28) ? 1 : 0;
				}
			}
			else if (speed < -1) {
				//estop is -2
				loc->speed = 0;
				loc->speedStep = 0;
				//note a non-zero eStopTimer lets the dcc packet engine know to transmit an estop message
				loc->eStopTimer = LOCO_ESTOP_TIMEOUT;
			}
			else if (speed == -1) {
				//decr speed
				loc->speedStep -= (loc->speedStep > 0) ? 1 : 0;
			}

			/*recalc the float speed value. C++ will give an int result for an integer divisor, hence need to express divisor
			 *as a double to force floating point math*/
			if (loc->use128) {
				loc->speed = loc->speedStep / 126.0;
			}
			else {
				loc->speed = loc->speedStep / 28.0;
			}

			loc->changeFlag = speed != 0 ? true : false;

			//direction
			if (dir) {
				if (loc->speed == 0) {
					/*if loco is stationary, reverse direction*/
					loc->forward = !loc->forward;
					loc->directionFlag = true;
				}
				else {
					/*else give it a nudge*/
					loc->nudge = 1;
				}
				loc->changeFlag = true;
			}

			//end of no-estop non-blocking code section
		}
	//return the slot modified, -1 if its not in the loco array
	//if eStop was blocking changes, we still need to return the slot
	for (int i = 0; i < MAX_LOCO; ++i) {
		if (&loco[i] == loc) {
			//increment history provided this was not an estop event
			if (speed >= -1) {
				incrLocoHistory(loc);
			}
			return i;
		}
	}
	return -1;

}



/*call usinng &target, will put given digitASCII at the digPos position
where 0 is least significant digit.  can handle 5 digit values
cannot handle -ve values
Two overloads, one changes 8 bit ints, the other 16*/
void changeDigit(char digitASCII, uint8_t digPos, uint8_t* target) {
	char buffer[11];
	/*sprintf ref http://www.cplusplus.com/reference/cstdio/sprintf/
	returns length of string excluding null terminatior*/
	uint8_t i = sprintf(buffer, "00000%d", *target);
	/*find substring target digit, zero is least sig digit*/
	i = i - 1 - digPos;
	/*change this digit and return new integer value*/
	buffer[i] = digitASCII;
	*target = atoi(buffer);
}

void changeDigit(char digitASCII, uint8_t digPos, uint16_t* target) {
	char buffer[11];
	/*sprintf ref http://www.cplusplus.com/reference/cstdio/sprintf/
	returns length of string excluding null terminatior*/
	uint8_t i = sprintf(buffer, "00000%d", *target);
	//Serial.println("changeDigit");
	//Serial.println(*target, DEC);
	/*find substring target digit, zero is least sig digit*/
	i = i - 1 - digPos;
	/*change this digit and return new integer value*/
	buffer[i] = digitASCII;
	*target = atoi(buffer);
}



int8_t setFunctionFromKey(KEYPAD& k) {
	/*loco[].function holds 8 bit function values
	 *key 1-4 relates to loco[0] with msb being the 4, then key5-8 relates to loco[1]
	 *returns with the index of loco[] which was changed, or -2 if no change
	 *for consistency we also set functionFlag*/
	uint8_t i = keypad.key;
	if (i > 16) { return -2; }
	if (k.keyHeld) { return -2; }
	/*F1-4 shown as bit 1234 msb is rightmost. F0 is controlled from the loco display*/
	i--;
	loco[(i / 4)].function ^= (0b10 << (i % 4));
	loco[(i / 4)].functionFlag = true;
	return (i / 4);
}



# pragma region LCD_display_routines


void updatePOMdisplay() {
	lcd.noBlink();
	lcd.home();
	//Cv   Val  POM Ad/
	//1024-123  L12345
	//1024-b0-1 S12345
	//1024-b0-1 A12345


	//2024-12-04 rewrite to deal with extra read states
	//write first line of display
	switch (m_pom.state) 
	{
	case POM_BYTE_WRITE:
	case POM_BIT_WRITE:
		lcd.print("POM written     ");
		return;
	case POM_BYTE_READ:
	case POM_BIT_READ:
		lcd.print("POM read        ");
		break;
	case POM_BYTE:
		//because read immediately decays to byte/bit, don't update if this func is called from the railcom postback, but do update this line if
		//called from the timeout
		if (m_pom.timeout > 0)break;
		lcd.print("Cv   Val  POM Ad");
		break;
	default:
		if (m_pom.timeout > 0)break;
		lcd.print("Cv   Bit  POM Ad");
	}
	lcd.setCursor(0, 1);

	//deal with second display line
	char buffer[20];
	switch (m_pom.state)
	{
	case POM_BYTE:
	case POM_BYTE_READ:
		//2024-12-02 added m_pom.readSuccess. This is true for a good read or if user is updating write value
		if (m_pom.useAccessoryAddr) {
			//2020-06-17 added accessory POM support
			if (m_pom.readSuccess) {
				sprintf(buffer, "%04d-%03d  A%05d", m_pom.cvReg, m_pom.cvData, m_pom.address);
			}else{
				sprintf(buffer, "%04d-???  A%05d", m_pom.cvReg, m_pom.address);
			}
		}
		else if (m_pom.useLongAddr) {
			//0000-000  L00000
			if (m_pom.readSuccess){
				sprintf(buffer, "%04d-%03d  L%05d", m_pom.cvReg, m_pom.cvData, m_pom.address);
			}else{
			sprintf(buffer, "%04d-???  L%05d", m_pom.cvReg, m_pom.address);
			}
		}
		else {
			if (m_pom.readSuccess) {
				sprintf(buffer, "%04d-%03d  S%05d", m_pom.cvReg, m_pom.cvData, m_pom.address);
			}else{
				sprintf(buffer, "%04d-???  S%05d", m_pom.cvReg, m_pom.address);
			}

		}
		lcd.print(buffer);
		/*set cursor*/
		if (m_pom.digitPos < 4) {
			lcd.setCursor(m_pom.digitPos, 1);
		}
		else if (m_pom.digitPos < 7) {
			lcd.setCursor(m_pom.digitPos + 1, 1);
		}
		else {
			lcd.setCursor(m_pom.digitPos + 3, 1);
		}
		lcd.blink();

		break;
	case POM_BIT:
	case POM_BIT_READ:
	/*display binary 1234-b1-0 S12345*/
		if (m_pom.useLongAddr) {
			if (m_pom.readSuccess) {
				sprintf(buffer, "%04d-b%d-%d L%05d", m_pom.cvReg, (m_pom.cvBit & 0b111), (m_pom.cvBit >> 7), m_pom.address);
			}
			else {
				sprintf(buffer, "%04d-b%d-? L%05d", m_pom.cvReg, (m_pom.cvBit & 0b111),  m_pom.address);
			}
			
		}
		else {
			if (m_pom.readSuccess) {
				sprintf(buffer, "%04d-b%d-%d S%05d", m_pom.cvReg, (m_pom.cvBit & 0b111), (m_pom.cvBit >> 7), m_pom.address);
			}
			else {
				sprintf(buffer, "%04d-b%d-? S%05d", m_pom.cvReg, (m_pom.cvBit & 0b111), m_pom.address);
			}
			
		}
		lcd.print(buffer);
		/*set cursor 1024-b2-1 L10239*/
		if (m_pom.digitPos < 4) {
			lcd.setCursor(m_pom.digitPos, 1);
		}
		else if (m_pom.digitPos == 4) {
			lcd.setCursor(m_pom.digitPos + 2, 1);
		}
		else if (m_pom.digitPos == 5) {
			lcd.setCursor(m_pom.digitPos + 3, 1);
		}
		else
		{
			/*address*/
			lcd.setCursor(m_pom.digitPos + 4, 1);
		}
		lcd.blink();
		break;
	}

}


void updatePowerDisplay() {
	lcd.noBlink();
	lcd.home();
	/*mA   limit  1234
	 *volt limit    14*/
	char buffer[17];
	sprintf(buffer, "mA   limit  %04d", bootController.currentLimit);
	lcd.print(buffer);
	lcd.setCursor(0, 1);  //col-row both zero indexed
	sprintf(buffer, "volt limit    %02d", bootController.voltageLimit);
	lcd.print(buffer);
	/*set cursor position per UI*/
	if (m_powerDigit < 4) {
		lcd.setCursor(m_powerDigit + 12, 0);
	}
	else
	{
		lcd.setCursor(m_powerDigit + 10, 1);
	}
	lcd.blink();
}


/*Update unithrottle Loco display
LOC:10023 1234mA    TRIP      1234mA
R:^128 012345678    R:^128 012345678

short locos are denoted by i.e. only 3 chars and more spaces
LOC:023   1234mA
*/
void updateUNIdisplay() {
	lcd.noBlink();
	lcd.home();
	char buffer[20];

	if (unithrottle.locPtr == nullptr) {
		lcd.write("nullptr");
		trace(Serial.println(F("UNI nullptr"));)
			return;
	}

	trace(Serial.printf("updateUNIdisplay %d\n", unithrottle.locPtr->address);)

		//one sec toggle alt display
		if (power.trip) {
			//2021-10-22 overcurrent or overvolt trip?
			if (power.bus_mA >= bootController.currentLimit) {
				sprintf(buffer, "TRIP      %04dmA", (int)power.bus_mA);
			}
			else {
				sprintf(buffer, "TRIP         %02dV", (int)power.bus_volts);
			}
		}
		else if (power.trackPower == false) {
			strcpy(buffer, "TRACK POWER OFF ");
		}
	//add brake and service here
		else if (unithrottle.locPtr->brake) {
			sprintf(buffer, "BRAKE     %04dmA", (int)power.bus_mA);
		}


		else {
			//display address and current level
			if (unithrottle.locPtr->useLongAddress) {
				sprintf(buffer, "LOC:%05d %04dmA", unithrottle.locPtr->address, (int)power.bus_mA);
			}
			else {
				sprintf(buffer, "LOC:%03d   %04dmA", unithrottle.locPtr->address, (int)power.bus_mA);
			}


		}
	lcd.print(buffer);
	memset(buffer, '\0', sizeof(buffer));

	//speed and functions
	lcd.setCursor(0, 1);  //col-row both zero indexed
	if (unithrottle.locPtr->use128) {
		sprintf(buffer, "F:%03d  012345678", unithrottle.locPtr->speedStep);
	}
	else {
		sprintf(buffer, "F:%02d   012345678", unithrottle.locPtr->speedStep);
	}

	if (unithrottle.locPtr->shunterMode == 0) {
		buffer[0] = unithrottle.locPtr->forward ? 0x01 : 0x02;
	}
	else {
		//use solid direction arrows in shunter mode
		buffer[0] = unithrottle.locPtr->forward ? 0x05 : 0x06;
	}

	//now assert function states.  012345678 if active, a digit will be replaced with a dot
	uint16_t f = unithrottle.locPtr->function;
	for (uint8_t i = 0; i < 9; ++i) {
		if ((f & (1 << i)) != 0) {
			buffer[i + 7] = 0x04;  //solid dot
		}
	}
	lcd.print(buffer);


	//When editing, mark current address digit with a blink
	if (m_machineSE == M_UNI_SET) {
		//digit pos0 is least sig digit
		if (unithrottle.locPtr->useLongAddress) {
			//LOC:00005 
			lcd.setCursor(8 - unithrottle.digitPos, 0);
		}
		else
		{
			//LOC:003
			lcd.setCursor(6 - unithrottle.digitPos, 0);
		}
		lcd.blink();
	}


}


/*turnout display will show _2 for first digit and 2_ for second
only supports first 8 turnouts on screen, irrespecive of MAX_TURNOUT*/
void updateTurnoutDisplay(void) {
	uint8_t y = 2;
	uint8_t i;
	char temp[20];
	lcd.home();
	
	for (i = 0; i < 8; i++) {
		//any address over 100 will be shown as hex, any address over 255 will be shown as XX
		if (turnout[i].address > 0xFF) {
			strcpy(temp, "XX| ");
		}
		else if (turnout[i].address > 100) {
			sprintf(temp, "%02x| ", turnout[i].address);
		}
		else {
			sprintf(temp, "%02d| ", turnout[i].address);
		}
		if (turnout[i].thrown) temp[2] = '/';
		lcd.print(temp);
		//advance to next row
		if (i == 3)  lcd.setCursor(0, 1);   //col,row
	}

	memset(temp, '\0', sizeof(temp));
	//digit entry and blinking
	switch (m_turnoutSE) {

	case TURNOUT_DIGIT_1_WAIT:  //Bx
		lcd.setCursor(0, 0);
		break;

	case TURNOUT_DIGIT_1: //B_
		lcd.setCursor(0, 0);
		sprintf(temp, "%d_", m_turnoutEnteredAddress);
		lcd.print(temp);
		lcd.setCursor(0, 0);
		break;

	case TURNOUT_DIGIT_2_WAIT: //xB
		lcd.setCursor(0, 0);
		sprintf(temp, "%d_", m_turnoutEnteredAddress);
		lcd.print(temp);
		lcd.setCursor(1, 0);
		break;

	case TURNOUT_DIGIT_2:
		lcd.setCursor(0, 0);
		sprintf(temp, "%02d", m_turnoutEnteredAddress);
		lcd.print(temp);
		lcd.setCursor(1, 0);
		break;

	case TURNOUT_TOGGLE:
		/*find selected item and set cursor posn*/

		for (i = 0; i < 8; i++) {
			if (turnout[i].selected) {
				lcd.setCursor(y, (i / 4));
				break;
			}
			y += 4;
			if (y > 14) { y = 2; }
		}
		break;
	}//end switch

	lcd.blink();

}//end

//update local hardware display when in Service Mode
void updateServiceModeDisplay(void) {
	lcd.home();
	char buffer[20];
	switch (m_sm.state) {
	case CV_IDLE:
		lcd.print("Cv   Val  Pg Rg ");
		break;
	case D_START:
		lcd.print("Direct write    ");
		break;
	case PG_START:
		lcd.print("Page-reg write  ");
		break;
	}

	lcd.setCursor(0, 1);
	/*2019-12-01 if data==-1 then display ??? as its not valid*/
	if (m_sm.cvData >= 0) {
		sprintf(buffer, "%04d-%03d  %03d/%02d", m_sm.cvReg, m_sm.cvData, m_sm.pgPage, m_sm.pgReg);
		//0000-000  00/00
	}
	else {
		//failed read of cv values
		sprintf(buffer, "%04d-???  failed", m_sm.cvReg);
	}
	lcd.print(buffer);

	/*set active digit*/
	if (m_sm.digit < 4) {
		lcd.setCursor(m_sm.digit, 1);
	}//col,row
	else { lcd.setCursor(m_sm.digit + 1, 1); }
	lcd.blink();
}

#pragma endregion


/// <summary>
/// restores settings from EEPROM. If the software version has changed, we overwrite the eeprom with defaults.
/// we also need to clear certain values on boot.max EEPROM we can use is 4096
/// </summary>
void dccGetSettings() {
	CONTROLLER defaultController;  //grab defaults as per DCCcore.h
	EEPROM.begin(1024);  //allows for 8 locos and 8 turnouts (uses 532)
	int eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	if (defaultController.softwareVersion != bootController.softwareVersion) {
		/*need to re-initiatise eeprom with factory defaults*/
		EEPROM.put(0, defaultController);
		eeAddr += sizeof(bootController);
		//2021-10-07 when doing a factory reset, only load loco 3
		for (int i = 0; i < MAX_LOCO; ++i) {
			//loco[i].address = (i <MAX_LOCO )? i+3  : 0;
			loco[i].address = (i == 0) ? i + 3 : 0;
			loco[i].useLongAddress = false;
		}
		//other settings such as defaults for 28 steps and longAddr are defined in the struct itself
		EEPROM.put(eeAddr, loco);
		//2020-05-03 also store turnouts
		eeAddr += sizeof(loco);
		EEPROM.put(eeAddr, turnout);
		EEPROM.commit();
	}
	//Now populate our structs with EEprom values
	eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.get(eeAddr, loco);
	//2020-05-03 also turnouts
	eeAddr += sizeof(loco);
	EEPROM.get(eeAddr, turnout);
	eeAddr += sizeof(turnout);
	//Reset certain parameters on every boot
	for (auto& loc : loco) {
		loc.speed = 0;
		loc.speedStep = 0;
		loc.consistID = 0;
		loc.forward = true;
		loc.jog = false;
		loc.history = 0;
		loc.LocoNetSTATUS1 = 0b11;
	}

	for (auto& t : turnout) {
		t.thrown = false;
	}


	/*initiailise UNIthrottle*/
	loco[0].jog = true;
	unithrottle.locPtr = &loco[0];
	m_tempLoco.address = 0;
	m_tempLoco.use128 = true;
	m_tempLoco.useLongAddress = true;
	m_tempLoco.function = 0;
	unithrottle.digitPos = 0;

	//trace dump the eeprom size used
	trace(Serial.printf("GETsettings loco %d, turnout %d, bytes %d\r\n ", sizeof(loco), sizeof(turnout), eeAddr);)

}

/*Call dccPutSettings if user changes a loco addr, short/long or step-size, or at system level they change current trip*/
void dccPutSettings() {
	if (bootController.isDirty == false) { return; }
	int eeAddr = 0;
	EEPROM.put(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.put(eeAddr, loco);
	/*2020-05-03 also store turnouts*/
	eeAddr += sizeof(loco);
	EEPROM.put(eeAddr, turnout);
	eeAddr += sizeof(turnout);

	EEPROM.commit();
	bootController.isDirty = false;
	trace(Serial.printf("EEPROM commit, bytes %d\r\n", eeAddr);)
}


void DCCcoreBoot() {

	//2022-01-08 pinMode for PIN_HEARTBEAT, and PIN_JOG_xx now handled in Jogwheel cpp


#ifdef PIN_ESTOP
	pinMode(PIN_ESTOP, INPUT_PULLUP); //pull low to signal Emergency Stop
#endif	


	power.trackPower = false;

	/*LCD initialise block*/
	Wire.begin(PIN_SDA, PIN_SCL);  //D1 sda and D2 scl

	/*I2C device scan*/
	byte error, address;
	int nDevices;


	Serial.println(F("I2C scanning..."));
	nDevices = 0;
	for (address = 1; address < 127; ++address)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println(" !");

			++nDevices;
		}
		else if (error == 4)
		{
			Serial.print(F("Unknown error at address 0x"));
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println(F("No I2C devices found\n"));
	else
		Serial.println(F("I2C scan done\n"));
	
	//2025-08-15 if Device count is >1 then we have a DSKY present
	if (nDevices > 1) {
		bootController.hasDSKY = true;
	}
	else {
		bootController.hasDSKY = false;
		//also put all locos into shunter mode
		for (auto& loc : loco) {
			loc.shunterMode = true;
		}
	}

	

	/*end I2C scan */


	lcd.begin(16, 2); // initialize the lcd
	lcd.setBacklight(255);
	/*load custom chars, after this command we must return device to DRAM mode. 0 does not seem to work*/
	//2026-01-08 the custom chars are now defined as const uint8_t to save ram, hence need to cast to a pointer here
	lcd.createChar(1, (uint8_t*)UP_ARROW);
	lcd.createChar(2, (uint8_t*)DOWN_ARROW);
	lcd.createChar(3, (uint8_t*)OPEN_DOT);
	lcd.createChar(4, (uint8_t*)CLOSE_DOT);
	lcd.createChar(5, (uint8_t*)SHUNTER_UP_ARROW);
	lcd.createChar(6, (uint8_t*)SHUTER_DOWN_ARROW);


	/*lcd.home() will reset to DRAM mode #SPLASH*/
	lcd.home(); lcd.clear();
	lcd.print("ESP8266 DCC\x04\x03");
	lcd.setCursor(0, 1);
	/*pull the software version from eeprom*/
	/*_bootController.softwareVersion is a long representing the date, e.g. 20210503, convert to string and split*/
	char tmp[17];
	itoa(bootController.softwareVersion, tmp, 10);
	char out[17];
	memset(out, '\0', sizeof(out));
	strncpy(out, tmp, 4);
	strcat(out, "-");
	strncat(out, tmp + 4, 2);
	strcat(out, "-");
	strncat(out, tmp + 6, 2);
	/*prints the software version date*/
	lcd.print(out);


	/*would be useful to show IP address*/
	m_generalTimer = 16;


	/*INA219 current sensor*/
	ina219.begin();  // Initialize first board (default address 0x40)
	ina219Mode(true); //modify ina config to do 8.5mS averaged samples


	
	

}


/// <summary>
/// core machine processing, called regularly from INO program loop
/// </summary>
/// <returns>-2 if no change made to any loco or turnout
/// 127 if eStop was pressed
/// 0-8 for the loco slot affected
/// turnout slots indicated by 31-39 (offset 31) </returns>
int8_t DCCcore(void) {

	/*every 10mS as flagged from DCClayer1 as DCCpacket.msTickFlag, run keyscans and processing*/
	int8_t r = -2;  //default return value
	if (DCCpacket.msTickFlag) {
		DCCpacket.msTickFlag = false;


		//scan jogwheel
		nsJogWheel::jogWheelScan();

		//scan I2C keypad
		keyScan();

		//scan local estop button if present. we shift in a zero, then read the estop pin
		m_eStopDebounce = m_eStopDebounce << 1;
#ifdef PIN_ESTOP
		if (digitalRead(PIN_ESTOP) == HIGH) { m_eStopDebounce++; }
#else
		//Estop button not implemented. Emulate a high reading
		m_eStopDebounce++;
#endif

		//if local Emergency Stop key pressed, emulate an I2C keypad estop
		if ((m_eStopDebounce & 0b111) == 0x00) {
			keypad.key = KEY_ESTOP;
			keypad.keyFlag = true;
			keypad.keyHeld = false;
		}

		//process I2C keypad key value if we have one
		if (keypad.keyFlag) {
			keypad.keyFlag = false;

			//estop always performs an eStop, and this is how you exit M_TRIP
			//note that local estop key also performs same function 
			//2020-6-14 except when in boot mode.  in which case eStop will set max current at 250mA
			//test first to prevent re-entrant calling of this block
			if (keypad.key == KEY_ESTOP) {
				if (m_machineSE == M_BOOT && bootController.currentLimit != 250) {
					bootController.currentLimit = 250;
					lcd.setCursor(0, 1);  //col-row both zero indexed
					lcd.print("250mA mode set  ");

					//there is no protected mode as such, its just that we have override the user current limit
					//and set 250mA  which is the service mode current limit.  Even if user trips this and resets
					//250mA will still be set.
					//Bit of a bodge because there is a service mode flag on the power object, however
					//this gets cleared if you exit service mode which is unsafe and additionally that service mode
					//needs to be triggered after the unit has booted and allowed a say 3amp trip to be established.
					//any inproperly wired decoder would be fried by then.
					//also proper service mode sends idle not all loco slot data
				}
				else {
					//regular eStop processing
					lcd.home();
					lcd.print(F("!EMERGENCY STOP!"));
					Serial.println(F("!EMERGENCY STOP!"));
					m_machineSE = M_ESTOP;
					m_stateLED = L_ESTOP;
					//call out to the e stop routine, will broadcast and estop signal and zero all individual locos
					m_generalTimer = 16;
					dccSE = DCC_ESTOP;
					r = 127;
				}
			}


			//key-press related changes to local machine state engine
			switch (m_machineSE) {

			case M_UNI_SET:
				//assigns a new loco or updates an existing one
			{//scope block M_UNI_SET. to avoid crosses-initialization-error in the compiler

				bool checkAddress = false;

				if (keypad.key == KEY_MODE) {
					m_machineSE = M_TURNOUT;
					updateTurnoutDisplay();
					break;
				}
				//process keys, no repeats
				if (keypad.keyHeld) break;

				switch (keypad.keyASCII) {
				case 'A':  //exit to UNI_RUN from the loco editor
					if (m_tempLoco.address == 0) {
						//2021-1-9 entering zero as an address will delete the loco
						//locPtr  on entry will be pointing to m_tempLoco
						m_generalTimer = 12;

						//find the active loco in-use on the display
						LOCO* active = nullptr;
						for (auto& loc : loco) {
							if (loc.jog) { active = &loc; break; }
						}

						if (active == nullptr) { lcd.print("error 7"); break; }
						trace(Serial.printf("active %d", active->address);)
							//can we delete existing loco?
							if ((active->consistID != 0) || (active->speed != 0)) {
								lcd.clear();
								lcd.print("Loco in use     ");
								lcd.setCursor(0, 1);
								lcd.print("cannot delete   ");

								//abandon delete, return to active loco
								unithrottle.locPtr = active;
								m_machineSE = M_UNI_RUN;
								break;
							}


						//proceed with delete
						active->address = 0;
						memset(active->name, '\0', sizeof(active->name));
						active->jog = false;
						lcd.setCursor(0, 1);
						lcd.print("Loco deleted    ");
						//allow message to persist (on timeout of this timer it will update to show another loco)
						m_generalTimer = 12;

						bootController.isDirty = true;
						dccPutSettings();
						trace(Serial.printf("addr is clear %d", active->address);)

							//find next loco and point at it
							unithrottle.locPtr = getNextLoco(unithrottle.locPtr);
						m_machineSE = M_UNI_RUN;
						bootController.flagLocoRoster = true;
						//updateUNIdisplay();  //don't update now
						break;
						//new code

					}

					m_generalTimer = 12;

					{//scope block 2, proceed with writing a new address
						char buffer[10];
						char existing[10];
						sprintf(buffer, "S%d", m_tempLoco.address);
						buffer[0] = m_tempLoco.useLongAddress ? 'L' : 'S';
						//2021-01-08 at this juncture we overwrite existing slot if appropriate or pick up a zero slot
						int8_t theSlot = findLoco(buffer, existing);
						if (theSlot == -1) {
							//cannot find existing loco nor able to create a slot for one
							lcd.clear();
							lcd.print("No slots free");
							//we need to abandon edit mode and return to active loco
							for (auto& loc : loco) {
								if (loc.jog) {
									unithrottle.locPtr = &loc;
									break;
								}
							}

							m_machineSE = M_UNI_RUN;
							break;
						}
						else
						{
							//write back to the appropriate loco slot as user may have changed speed steps
							//in the editor
							for (auto& loc : loco) {
								loc.jog = false;
							}


							//are we changing the slot address?
							//2021-7-8 special case check theSlot is not a blank slot
							if ((strcmp(existing, buffer) == 0) && (loco[theSlot].address != 0)) {
								//selecting existing loco. preserve address, function and consistID
								lcd.clear();
								lcd.print("Loco updated");

								//we picked up the loco details when we entered address.
								//user might have changed the speed steps though, so recalculate.
								loco[theSlot].use128 = m_tempLoco.use128;
								if (loco[theSlot].use128) {
									loco[theSlot].speedStep = 126 * loco[theSlot].speed;
								}
								else {
									loco[theSlot].speedStep = 28 * loco[theSlot].speed;
								}
								//write back to eeprom
								bootController.isDirty = true;  //pending write
								bootController.flagLocoRoster = true;
								//2021-09-01 increment age
								incrLocoHistory(&loco[theSlot]);
							}
							else {
								//overwrite slot details, we are either writing to a blank slot or overwriting one
								//nominated as underutilised
								lcd.clear();
								lcd.print("Loco created");
								loco[theSlot] = m_tempLoco;
								loco[theSlot].speed = 0;
								loco[theSlot].speedStep = 0;
								loco[theSlot].function = 0;
								//2021-09-01 increment age
								incrLocoHistory(&loco[theSlot]);

								trace(Serial.printf("overwrt %d\n", loco[theSlot].address);)
									//write to eeprom
									bootController.isDirty = true;  //pending write
								bootController.flagLocoRoster = true;
							}
							//execute eeprom write
							dccPutSettings();
							//assign the slot to the unithrottle
							unithrottle.locPtr = &loco[theSlot];
							unithrottle.locPtr->jog = true;

							m_machineSE = M_UNI_RUN;


						}
						trace(Serial.printf("UNI write to slot %d", theSlot);)


					}//end of scope block 2

					break;

				case 'B':
					//abandon edit, point to loco[0];
					unithrottle.locPtr = &loco[0];
					m_machineSE = M_UNI_RUN;
					break;

				case 'C':
					//2020-10-10 toggle shunter mode
					unithrottle.locPtr->shunterMode == 0 ? 1 : 0;
					unithrottle.locPtr->speedStep = 0;
					unithrottle.locPtr->speed = 0;
					break;

				case '*':
					//toggle speed steps
					unithrottle.locPtr->use128 = !unithrottle.locPtr->use128;
					unithrottle.locPtr->speedStep = 0;
					unithrottle.locPtr->speed = 0;
					updateUNIdisplay();
					break;

				case '#':
					//toggle long/short  LOC:12345 LOC:123
					unithrottle.locPtr->useLongAddress = !unithrottle.locPtr->useLongAddress;
					if (unithrottle.locPtr->useLongAddress) {
						unithrottle.digitPos = 4;
					}
					else {
						//if switching to short address, cap this at 127
						unithrottle.digitPos = 2;
						if (unithrottle.locPtr->address > 127) unithrottle.locPtr->address = 127;
					}
					updateUNIdisplay();
					checkAddress = true;
					break;
				}//end of ABCD*# select

				//direct digit entry for address editing
				if ((keypad.keyASCII >= '0') && (keypad.keyASCII <= '9')) {
					changeDigit(keypad.keyASCII, unithrottle.digitPos, &unithrottle.locPtr->address);
					//digitPos will wrap around
					if (unithrottle.digitPos == 0) {
						unithrottle.digitPos = unithrottle.locPtr->useLongAddress ? 4 : 2;
					}
					else
					{
						unithrottle.digitPos--;
					}

					//limit the address value
					if (unithrottle.locPtr->useLongAddress) {
						if (unithrottle.locPtr->address > 10023) unithrottle.locPtr->address = 10023;

					}
					else
					{
						if (unithrottle.locPtr->address > 127) unithrottle.locPtr->address = 127;
					}
					//zero indicates no-selection
					checkAddress = true;
					updateUNIdisplay();
				}

				//If address or addr length changed, lookup existing slot & pull a copy of the matching loco
				//if a match exists
				if (checkAddress) {
					char buffer[10];
					char existing[10];
					sprintf(buffer, "S%d", m_tempLoco.address);
					buffer[0] = m_tempLoco.useLongAddress ? 'L' : 'S';
					//2021-07-8 ignore empty slots. If we don't find an exact match we continue building
					//the address in m_tempLoco
					int8_t theSlot = findLoco(buffer, existing, true);
					trace(Serial.printf("checkAddress %d", theSlot);)
						if (theSlot != -1) {
							//are we selecting an existing slot or overwriting one?
							if (strcmp(buffer, existing) == 0) {
								m_tempLoco = loco[theSlot];
							}
							//else do nothing, we stick with m_tempLoco as is
							updateUNIdisplay();
						}
						else {
							//this is the case for a blank slot as -1 is the return value
							//reset certain params
							trace(Serial.println(F("checkAddress reset"));)
							
								//2025-02-19 use a function in the struct
							m_tempLoco.clear();
							/*
							m_tempLoco.consistID = 0;
							m_tempLoco.function = 0;
							m_tempLoco.speed = 0;
							m_tempLoco.speedStep = 0;
							m_tempLoco.jog = false;
							*/
							memset(m_tempLoco.name, '\0', sizeof(m_tempLoco.name));
						}
				}

			}//end of scope block M_UNI_SET
			break;


			case M_UNI_RUN:
				//2020-10-07 long-mode-press takes us to M_POM, short to M_TURNOUT

				if (keypad.key == 0) {
					//had requested keyup, here it is, this was a short-mode-press, go to turnout
					m_machineSE = M_TURNOUT;
					updateTurnoutDisplay();
					break;
				}

				if (keypad.key == KEY_MODE) {
					//need to wait and see key-held, which will take us to POM
					keypad.requestKeyUp = true;
					if (keypad.keyHeld) {
						m_machineSE = M_POM;
						m_pom.state = POM_BYTE;
						updatePOMdisplay();
					}
				}

				//2020-10-07 otherwise process the key

				//reset general timer for a current update
				m_generalTimer = 12;

				if (unithrottle.locPtr == &m_tempLoco) {
					m_machineSE = M_UNI_SET;
					unithrottle.digitPos = unithrottle.locPtr->useLongAddress ? 4 : 2;
					trace(Serial.println(F("invoke editor"));)
						updateUNIdisplay;
					break;
				}
				if (unithrottle.locPtr == nullptr) unithrottle.locPtr = getNextLoco(unithrottle.locPtr);


				//* inc speed, 0 dec, # rev, D lamp
				//1-8 are direct function keys
				switch (keypad.keyASCII) {
				case 'A':
				{//scope block. Enter address-set mode /  edit current loco mode
					if (keypad.keyHeld) break;
					//enter M_UNI_SET, default to long address and 128 speed steps
					//2020-07-6 if we have an existing valid loco then don't reset address to zero.
					//user may wish to change the speed steps
					int e = setLoco(unithrottle.locPtr, 0, false);  //find current loco slot index
					if ((e < 0) || (e > MAX_LOCO)) {
						//no an active loco slot, entering a new loco from scratch
						m_tempLoco.address = 0;
						m_tempLoco.useLongAddress = true;
						m_tempLoco.speed = 0;
						m_tempLoco.speedStep = 0;
						m_tempLoco.use128 = true;
						m_tempLoco.function = 0;
						trace(Serial.println(F("oranges #2"));)
					}
					else
					{
						//edit a COPY of the currently selected loco
						m_tempLoco = loco[e];
					}

					unithrottle.locPtr = &m_tempLoco;
					unithrottle.digitPos = unithrottle.locPtr->useLongAddress ? 4 : 2;

					m_machineSE = M_UNI_SET;
					updateUNIdisplay();
					//exit with unithrottle.locPtr pointing at an editor object

				}//end scope block
				break;

				case 'B':
				{//scope block ADVANCE
					if (keypad.keyHeld) break;
					//step through all available locos
					int8_t thisLoco = setLoco(unithrottle.locPtr, 0, false);
					//check existing loco is a valid slot
					if ((thisLoco == -1) || (thisLoco > MAX_LOCO)) {
						//no, so clear locPtr and let UNI_SET entry point deal with it
						unithrottle.locPtr = nullptr;
					}
					else
					{
						//advance to next non-zero slot
						//2021-01-08 bug here when all slots are zero we get stuck in an infinite loop
						//and have a WDT timeout. To protect against this, we will fail out of the loop
						//after a couple of cycles, pointing at an empty slot
						//the UNI_RUN code should then force the UI into editor mode

						for (int fail = 0; fail < MAX_LOCO + 1; ++fail) {
							thisLoco++;
							if (thisLoco == MAX_LOCO) thisLoco = 0;
							if (loco[thisLoco].address != 0) break;
						}

						//exits with next non-zero address selected, or a zero address if all slots are zero
						for (auto& loc : loco) {
							loc.jog = false;
						}
						loco[thisLoco].jog = true;
						unithrottle.locPtr = &loco[thisLoco];
						//if address is zero, invoke editor
						if (unithrottle.locPtr->address == 0) {
							m_machineSE = M_UNI_SET;
							updateUNIdisplay();
						}
					}
					//we either successfully advance or exit this block with locPtr=nullPtr which
					//is deal with by the M_UNI_RUN entry point


				}//end scope block ADVANCE
				break;

				case 'C':
					//2020-10-07 toggle shunter mode.  "off" is 0, 1 or -1 is the direction override
					unithrottle.locPtr->shunterMode = unithrottle.locPtr->shunterMode == 0 ? 1 : 0;
					break;

				case '*':
					if (unithrottle.locPtr->shunterMode == 0) {
						//normally a speed increase
						setLoco(unithrottle.locPtr, 1, false);
						break;
					}
					else {
						//in shunter mode, we might need to decr instead
						setLoco(unithrottle.locPtr, unithrottle.locPtr->shunterMode, false);
						//if we just hit zero (means we were decrementing) then flip the sense to incr
						if (unithrottle.locPtr->speed == 0) {
							unithrottle.locPtr->shunterMode = 1;
							setLoco(unithrottle.locPtr, 0, true);  //flip direction
						}
					}
					break;
				case '0':
					if (unithrottle.locPtr->shunterMode == 0) {
						//normally a speed decrease
						setLoco(unithrottle.locPtr, -1, false);
						break;
					}
					else {
						//in shunter mode, we might need to incr instead
						setLoco(unithrottle.locPtr, -unithrottle.locPtr->shunterMode, false);
						//if we just hit zero (means we were decrementing) then flip the sense to incr
						if (unithrottle.locPtr->speed == 0) {
							unithrottle.locPtr->shunterMode = -1;
							setLoco(unithrottle.locPtr, 0, true);  //flip direction
						}
					}
					break;
				case'#':
					if (keypad.keyHeld) break;
					setLoco(unithrottle.locPtr, 0, true);
					break;
				case 'D':
					if (keypad.keyHeld) break;
					unithrottle.locPtr->function ^= 1;
					unithrottle.locPtr->functionFlag = true;
					break;
				}

				//no repeat key for the next section
				if (keypad.keyHeld) break;

				//eStop key is code 18
				if (keypad.key == 18) {
					setLoco(unithrottle.locPtr, -2, false);
				}

				//direct function control
				if ((keypad.keyASCII >= '1') && (keypad.keyASCII <= '8')) {
					uint8_t k = keypad.keyASCII - '1';
					unithrottle.locPtr->function ^= (0b10 << k);
					unithrottle.locPtr->functionFlag = true;
				}

				updateUNIdisplay();  //need this if we  are using temp LOCO as that is not going to trigger the update


				//if we made a speed/dir change, check jog is set
				if ((unithrottle.locPtr->changeFlag) && (!unithrottle.locPtr->jog)) {
					for (auto& loc : loco) {
						loc.jog = false;
					}
					unithrottle.locPtr->jog = true;
				}

				break;

			case M_TURNOUT:
				if (keypad.keyHeld) { break; }
				if (keypad.key == KEY_MODE) {
					m_machineSE = M_UNI_RUN;
					updateUNIdisplay();
				}  //mode
				else {
					/*pass keypad by pointer as the function will modify it
					 *the turnout index returned needs to have an offset of 31 added*/
					r = setTurnoutFromKey(keypad);
					/*only add the index if return value is +ve*/
					if (r >= 0) { r += 31; }
					updateTurnoutDisplay();
				}
				break;

			case M_SERVICE:
				if (keypad.keyHeld) { break; }
				if (keypad.key == KEY_MODE) {
					writeServiceCommand(0, 0, true, false, true);
					updateUNIdisplay();
				}
				else {
					/*2019-12-02 fix to problem with CV-verify. we kick off ACK monitor but then burn up
					cycles with update cvDisplay.  so fix is have setCVfromKey return bool and only update if true*/
					if (setServiceModefromKey()) updateServiceModeDisplay();
				}
				break;

			case M_ESTOP:
				//2019-11-18 if mode-short then enter service mode, if mode-long then enter Power settings
				//key-release (i.e. less than Mode-hold-period) will take us to Service Mode
				if (keypad.key == 0) {
					//cv control is entered whilst eStop active
					writeServiceCommand(0, 0, true, true, false); 
					updateServiceModeDisplay();
					break;
				}

				//if mode key, request a key-up event, trigger POWER if its held		   				 
				if (keypad.key == KEY_MODE) {
					//need to wait and see key-held, which will take us to Power settings, otherwise go to Service Mode
					keypad.requestKeyUp = true;
					if (keypad.keyHeld) {
						m_machineSE = M_POWER;
						updatePowerDisplay();
					}
				}
				//any other key is ignored, instead we rely on eStop timeout to revert us to ops mode
				break;

			case M_POWER:
				//exit with mode button, we arrived here because of mode-held so ignore this
				if (keypad.keyHeld) { break; }
				if (keypad.key == KEY_MODE) {

					m_machineSE = M_UNI_RUN;
					updateUNIdisplay();

					//write pending eeprom changes
					dccPutSettings();
					dccSE = DCC_LOCO;  //return to loco packet transmission
				}  //mode
				else {
					setPowerFromKey();
					updatePowerDisplay();
				}
				break;

			case M_POM:
				//exit with mode button, we arrived here because of mode-held so ignore this
				if (keypad.keyHeld) { break; }
				if (keypad.key == KEY_MODE) {
					dccSE = DCC_LOCO;  //return to loco packet transmission
					m_machineSE = M_UNI_RUN;
					updateUNIdisplay();
				}
				else {
					setPOMfromKey();
					updatePOMdisplay();
				}
				break;

			}//end machine SE

		}//end keyflag


		//2019-10-10 check jogwheel activity
		if (jogWheel.jogEvent || jogWheel.jogButtonEvent) {
			//jogwheel related changes to local machine state engine

				switch (m_machineSE) {
				case M_UNI_RUN:
					m_generalTimer = 10;
					//2020-07-05 for unithrottle we need to trigger a display refresh

				//2020-06-14 allow jog to control loco during turnout mode and function mode
				case M_TURNOUT:


					r = setLocoFromJog(jogWheel);

					//2020-05-03 flag a change for broadcasting, UI update to follow
					if (r >= 0) {
						loco[r].changeFlag = true;
						//modifying a single loco that is in a WiThrottle consist is handled here
						replicateAcrossConsist(r);
					}
				}
			
		}

		//2019-11-15 deal with jog button pushes. These apply the brake/not if pushed/not
		//potentially a long-push could reverse direction or apply a nudge
		switch (m_machineSE) {
			///only act if we are in loco operate mode, don't need to consider the button event status

		case M_TURNOUT:  //added 2020-10-07
		case M_UNI_RUN:
			for (auto& loc : loco) {
				if (loc.jog) loc.brake = jogWheel.jogButton;
			}
		}


		//deal with all 250mS event counters here
		++m_tick;
		if (m_tick >= 25) {
			//count down any open eStop timers and the generalTimer
			m_tick = 0;
			quarterSecFlag = true;
			for (auto& loc : loco) {
				loc.eStopTimer -= loc.eStopTimer == 0 ? 0 : 1;
			}

			//countdown cv timeout. repaint display as we hit zero
			if (m_sm.timeout > 0) {
				m_sm.timeout--;
				if (m_sm.timeout == 0) { updateServiceModeDisplay(); }
			}
			//countdown POM timeout. repaint display as we hit zero
			if (m_pom.timeout > 0) {
				m_pom.timeout--;
				if (m_pom.timeout == 0) {
					trace(Serial.println(F("POM timeout"));)
					updatePOMdisplay();
				}
			}

			//handle LED display state. We don't write directly to the PIN_HEARTBEAT pin, instead it is handled through
			//the jogWheel routines
			++m_ledCount;
			if (m_ledCount >= 8) { m_ledCount = 0; }
			if ((m_stateLED & (1 << m_ledCount)) == 0) {
				jogWheel.pinState = HIGH;
			}
			else {
				jogWheel.pinState = LOW;
			}


			//timer event driven machine state
			if (m_generalTimer > 0) {
				m_generalTimer--;

				if (m_generalTimer == 0)
				{//hit zero
					switch (m_machineSE) {
					case M_BOOT:
						//enable track power, then measure quiescent current
						trace(Serial.println(F("enable power"));)
							power.trackPower = true;
						//assume quiescent power is say 250mA,this gets adjusted downward as the unit sees
						//real readings come in
						power.quiescent_mA = 250;
						//2021-10-22 clear bus_volts
						power.bus_volts = 0;

						///2020-10-07 we boot in M_UNI_RUN
						m_machineSE = M_UNI_RUN;
						m_stateLED = L_NORMAL;
						updateUNIdisplay();

						//2020-06-14 if user pressed mode during boot or estop, then boot in limited current mode.
						//this is handled above in the key routines
						break;

					case M_ESTOP:
						m_machineSE = M_UNI_RUN;
						//2020-01-26 return to processing loco packets
						dccSE = DCC_LOCO;

						//no break, run on into block below
					case M_UNI_RUN:
					case M_UNI_SET:
						updateUNIdisplay();
						//force display to refresh with current reading every 1.5 second
						m_generalTimer = 6;
						m_stateLED = L_NORMAL;
						break;

					}
				}
			}//end general timer

		}//end 250mS tick event

		/*INA219 current monitoring code block, samples at 10mS intervals.  The device is set to average
		16 samples over 8.5mS which will avoid spikes causing false trips
		do not update the reading if we have a trip condition, wish to preserve the peak current on the display
		if the INA is not present, bus volts will read as 32v or higher*/

		/*NOTE: to reset a power trip, press ESTOP, it will clear the trip on exit
		   alternatively cycle power through JSON WiThrottle*/

		   //still within the 10mS tick calls
		if (power.trip == false) {
			/*implement a rolling average weighing of 0.2, this gives 90% of final value in 10 samples
			i.e. a 100mS response to overloads*/

#ifdef USE_ANALOG_MEASUREMENT
			//Analoge measurement exponentially smooths readings into the bus_mA value.
			power.ADresult = int(power.ADresult * 0.8 + 0.2 * analogRead(A0));
			power.bus_mA = ANALOG_SCALING * power.ADresult;

#else
			//INA based measurement
			power.bus_volts = ina219.getBusVoltage_V();
			//2021-02-18 use an exponential average for current monitoring.  The L298 and IBT2 do cause a trip without 
			//exponential smoothing just on a INA average reading, however the LMD18200 does not cause a trip.
			//Its short-protection feature seems to act very quickly and limit the current to about 1.5A which sustains
			//the short and won't trigger a 2A trip.
			//If we apply exponential smoothing, the 2A overload is correctly captured, which is counter intuitive.
			
			//2026-01-07 apply averaging if required (turned off when looking for ACK)
			if (power.averaging) power.bus_mA = (ina219.getCurrent_mA() * 0.2) + 0.8 * power.bus_mA;
#endif

			if (power.bus_mA < power.quiescent_mA) { power.quiescent_mA = power.bus_mA; }

			//Power trip condition present?
			if (power.serviceMode) {
				if (power.bus_mA - power.quiescent_mA > 250) {
					Serial.print(F("Service Mode power trip "));
					power.trip = true;
				}
			}
			else if (power.bus_mA > bootController.currentLimit) {
				Serial.print(F("Current trip "));
				Serial.println(power.bus_mA, DEC);
				power.trip = true;
			}

			//look for over-voltage condition
			if (power.bus_volts > bootController.voltageLimit) {
				Serial.print(F("Voltage trip "));
				power.trip = true;
			}


			//2021-10-12 did we just trip the power?  If so signal that trackPower is now off 
			//and broadcast ths to the WiThrottles

			if (power.trip) {
				power.trackPower = false;
#ifdef _WITHROTTLE_h
				nsWiThrottle::broadcastPower();
				trace(Serial.println("TRIP bcast");)
#endif
			}
		}   //end of power trip monitoring


	}//end msTickFlag, 10mS


	//Code below is run on every call to DCCcore.
	//call the dcc packet engine
	dccPacketEngine();

	//If using the ESP AD converter, sample AD every 1mS when in DCC Service Mode
#ifdef USE_ANALOG_MEASUREMENT
	switch (m_cv.state)
	{
	case RD_START:
		//get ready for ACK, capture the base current
		//ACK might occur during the RD_RESET packets or RD_FINAL packets that follow RD_START
		power.ackBase_mA = power.bus_mA;
		power.ackFlag = false;
		break;
	case RD_FINAL:
		//sample at 1mS, we don't look for a sustained 60mA for 6mS pulse, the first high sample will do
		if (!DCCpacket.fastTickFlag) break;
		DCCpacket.fastTickFlag = false;
		if (power.ackFlag) break;
		if ((analogRead(A0) * ANALOG_SCALING) > (power.ackBase_mA + 60)) { power.ackFlag = true; }
	}
	//m.cv reverts to CV_IDLE when done
#endif

	return r;

}//end function



/// <summary>
/// sets INA current monitoring in average mode or trigger mode (one time 68mS sample for ACK pulse in service mode)
/// first entry to trigger mode will populate baseline current value and suspend averaging calculation
/// </summary>
/// <param name="AverageMode">false will search for ACK pulse in Service Mode</param>
void ina219Mode(boolean AverageMode) {
	uint16_t config;
	if (AverageMode) {
		power.averaging = true;

		//change ina config to support 32V 3.2A and 8ms averaging over 16 samples
		config = INA219_CONFIG_BVOLTAGERANGE_32V |
			INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
			INA219_CONFIG_SADCRES_12BIT_16S_8510US |
			INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

		//INA219_CONFIG_SADCRES_12BIT_8S_4260US
	}
	else
	{	//For Service Mode ACK detection. Use one time trigger mode, setting 68mS with 128 samples, shunt only.
		//assuming this is triggered at about the time we expect to see ACK, then if half the samples see 55mA+
		//then the measured avg current should be >27mA.
		//The moment we write this mode is the moment it is triggered, even if we re-write the same mode

		config = INA219_CONFIG_BVOLTAGERANGE_32V |
			INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
			INA219_CONFIG_SADCRES_12BIT_128S_69MS |
			INA219_CONFIG_MODE_SVOLT_TRIGGERED;

		if (power.averaging) {
			//first entry, take a baseline sample.
			power.ackBase_mA = power.bus_mA;
			power.bus_peak_mA = power.ackBase_mA;
			power.averaging = false;
		}



		//Only sample the shunt voltage, because sampling both this and the bus will half the effective rate on both.

		//INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
		//INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED
		//INA219_CONFIG_SADCRES_12BIT_16S_8510US
		//INA219_CONFIG_SADCRES_12BIT_8S_4260US
		//INA219_CONFIG_SADCRES_12BIT_1S_532US
		//INA219_CONFIG_SADCRES_12BIT_2S_1060US
		//INA219_CONFIG_SADCRES_12BIT_128S_69MS
		//INA219_CONFIG_SADCRES_12BIT_64S_34MS
	}

	Wire.beginTransmission(INA219_ADDRESS);
	Wire.write(INA219_REG_CONFIG);
	Wire.write((config >> 8) & 0xFF); // Upper 8-bits
	Wire.write(config & 0xFF);        // Lower 8-bits
	Wire.endTransmission();

}



/// <summary>
/// update local machine display in response to JRMI instructions over JSON or WiThrottle
/// or from the local hardware interface
/// will first transmit turnout - commands to line
/// will also clear the change flags and will clear broadcast roster flags
/// </summary>
/// 2026-02-05 possible bug;  we should not allow entry into DCC_ACCESSORY if ddcSE is DCC_SERVICE | DCC_POM because that could 
/// cause unpredicatable operation.
void updateLocalMachine(void) {

	bool doUpdate = false;
	for (auto& loc : loco) {
		if (loc.changeFlag) {
			doUpdate = true;
			loc.changeFlag = false;
			loc.directionFlag = false;
		}
		if (loc.functionFlag) {
			doUpdate = true;
			loc.functionFlag = false;
		}
	}
	for (auto& turn : turnout) {
		if (turn.changeFlag) {
			doUpdate = true;
			//2020-05-18 queue transmission to line, and only handle one change at a time
			accessory.address = turn.address;
			accessory.thrown = turn.thrown;
			//2026-02-05 for the turnout roster, we only support powerOn.  powerOff is relevant for signal aspects.
			//turnouts, whether solenoid or servo have decoders that control pulses or positions so powerOff is not relevant.
			accessory.powerOn = true;
			dccSE = DCC_ACCESSORY;
			/*transmit one at a time*/
			turn.changeFlag = false;
			break;
		}
	}

	//2026-02-05 insert something here to set accessory from LocoNet or do we just directly write to accessory and go into dccSE = DCC_ACCESSORY in a separate sub?



	//2021-1-27 clear roster flags, these should have been acted on by WiThrottle and DCCweb
	bootController.flagLocoRoster = false;
	bootController.flagTurnoutRoster = false;

	/*only update local display if a change is required, else we will attempt to overwrite the LCD too frequently*/
	if (!doUpdate)return;

	switch (m_machineSE) {


	case M_TURNOUT:
		updateTurnoutDisplay();
		break;

	case M_UNI_RUN:
		updateUNIdisplay();
		break;

	}

}



/// <summary>
/// modify speed of loco using the jogwheel, apply brake if pushbutton pressed
/// reverse direction on stationary if jog button held
/// </summary>
/// <param name="j">jogwheel struct</param>
/// <returns>index of loco the jogwheel is currently assigned to, -2 if Estop is blocking</returns>
int8_t setLocoFromJog(nsJogWheel::JOGWHEEL& j) {
	uint8_t i;
	for (i = 0; i < MAX_LOCO; i++) {
		if (loco[i].jog) { break; }
	}

	/*range test i. Estop condition is blocking, cannot increment or decrement speed until it is cleared*/
	if (i == MAX_LOCO || loco[i].eStopTimer != 0) {
		j.jogButtonEvent = false;
		j.jogEvent = false;
		return -2;
	}

	if (j.jogEvent) {
		//2021-01-26 implement jogLoSpeed. If user is slowly rotating the knob they have precision control and it's
		//appropriate to move through the direction change and increment speed.  if they were rotating more quickly
		//we want to pause on zero so they don't overshoot.

		if ((loco[i].shunterMode != 0) && (loco[i].speedStep == 0) && (!j.jogLoSpeed)) {
			//user has to slowly rotate through zero.  Don't allow high-speed shoot-through
			j.jogEvent = false;
		}
	}


	if (j.jogEvent) {
		//2020-10-10 follow jogCW unless shunter is -1 and in which case invert
		if (loco[i].shunterMode == -1) { j.jogCW = !j.jogCW; }

		if (j.jogCW) {
			/*increment speed for clockwise rotation*/
			if (loco[i].use128) {
				if (j.jogHiSpeed) {
					loco[i].speedStep += 5;
					if (loco[i].speedStep > 126) loco[i].speedStep = 126;
				}
				else {
					loco[i].speedStep += (loco[i].speedStep < 126) ? 1 : 0;
				}
			}
			else {
				loco[i].speedStep += (loco[i].speedStep < 28) ? 1 : 0;
			}
		}
		else {
			/*decrement speed on counter clockwise*/
			loco[i].speedStep -= (loco[i].speedStep > 0) ? 1 : 0;
			if (loco[i].use128 && j.jogHiSpeed) {
				loco[i].speedStep -= (loco[i].speedStep > 5) ? 5 : 0;
			}

		}

		//2020-10-10 if have hit zero, and we are in shunter mode, flip the direction and
		//the shuntermode flag

		if ((loco[i].shunterMode != 0) && (loco[i].speedStep == 0)) {
			loco[i].forward = !loco[i].forward;
			loco[i].shunterMode = loco[i].shunterMode == 1 ? -1 : 1;
		}

		/*recalc the float speed value. C++ will give an int result for an integer divisor, hence need to express divisor
		 as a double to force floating point math*/
		if (loco[i].use128) {
			loco[i].speed = loco[i].speedStep / 126.0;
		}
		else {
			loco[i].speed = loco[i].speedStep / 28.0;
		}

		j.jogEvent = false;
	}

	//2020-06-15 reverse direction if button held and we are at rest
	if (j.jogButtonEvent) {
		if (j.jogHeld && (loco[i].speed == 0)) {
			loco[i].forward = !loco[i].forward;
			loco[i].directionFlag = true;
		}
		j.jogButtonEvent = false;
	}

	return i;
}//end function



/// <summary>
/// If a change is made to a loco, replicate this to other locos in the same consist.
/// Consists can be created in WiThrottle, not in the local hardware interface
/// </summary>
/// <param name="slot">index of target loco slot</param>
void replicateAcrossConsist(int8_t slot) {
	if (slot<0 || slot > MAX_LOCO) return;
	if (loco[slot].consistID == 0) return;

	for (int i = 0; i < MAX_LOCO; ++i) {
		if (i == slot) continue;
		if (loco[i].consistID != loco[slot].consistID) continue;
		/*replicate*/
		if (loco[slot].changeFlag) {
			/*replicate speed*/
	//DEBUG
			trace(Serial.printf("consist %d replicate %d to %d", loco[i].consistID, slot, i);)
			loco[i].speed = loco[slot].speed;
			loco[i].changeFlag = true;
			//calculate the speed-step value from the percentile value
			if (loco[i].use128) {
				loco[i].speedStep = int(0.05 + (loco[i].speed * 126));
			}
			else {
				loco[i].speedStep = int(0.05 + (loco[i].speed * 28));
			}

			/*direction is more complex, if it has indeed changed, then we need to toggle all other locos*/
			if (loco[slot].directionFlag) {
				/*toggle, do not set absolute*/
				loco[i].forward = !loco[i].forward;
			}
		}
		if (loco[slot].functionFlag) {
			loco[i].function = loco[slot].function;
			loco[i].functionFlag = true;
		}

	}

}



/// <summary>
/// Initiates a POM command to the track, supports read and write
/// </summary>
/// <param name="addr">first char A accessory, L long, S short. Followed by the address numeric</param>
/// <param name="cv">CV register to write to</param>
/// <param name="val">first char B write byte, R read byte, S set bit, C clear bit. Followed by numeric</param>
///  <returns>false if command malformed, true if initiated</returns>
bool writePOMcommand(const char* address, uint16_t cv, const char* val) {
	
	if (address == nullptr) return false;
	if (val == nullptr) return false;
		
	m_pom.useLongAddr = address[0] == 'L' ? true : false;
	m_pom.address = atoi(address + 1);
	if (m_pom.address == 0) return false;


	if (cv == 0 || cv > 1024) return false;
	//POM processing code remaps 1 to 0 on the line.
	m_pom.cvReg = cv;

	switch (val[0]) {
	case 'B':
		m_pom.cvData = atoi(val + 1);
		m_pom.state = POM_BYTE_WRITE;
		break;

	case 'R':  //2024-05-04
		m_pom.cvData = atoi(val + 1);
		m_pom.state = POM_BYTE_READ;
		break;

	case 'S':
	case 'C':
		//bit write. cvBit <2-0> represent the bit posn, <7> represents set or clear
		//the val string will be b21 where b<7-0><1|0>
		m_pom.cvBit = (val[1] - '0') & 0b111;
		m_pom.cvBit += val[0] == 'S' ? 0b10000000 : 0;
		m_pom.state = POM_BIT_WRITE;
		break;

	default:
		return false;
	}
	//initiate write sequence
	dccSE = DCC_POM;
	//we do not set m_pom.timeout as we don't want to update local display for this remote operation
	return true;

}


//rollback.

bool writeServiceCommand(uint16_t cvReg, uint8_t cvVal, bool verify, bool enterSM, bool exitSM) {
	//2020-12-27 routine is initiate-only.  i.e. initiate a write, initiate a read.  in the case of read
	//there is a call back to nsDCCweb

	//note that the DCC spec does not expect to run a loco in service mode, all it supports is setting
	//and reading of CVs.  If you can read a CV then reasonably you can assume the loco is wired 
	//correctly and will operate on the Main.
	//https://github.com/esp8266/Arduino/issues/4689

	if (enterSM) {
		if (power.serviceMode) return true;
		//process an eStop on all locos
		for (auto& loc : loco) {
			//setLoco(&loc, -1, true);

		}
		//this will take around 100mS to execute, we immediately go into service mode which will result in 
		//idle being sent to line and a 250mA current limit which we might trip if several locos were running at
		//the point we switched to SM
		dccSE = DCC_SERVICE;
		m_sm.state = CV_IDLE;
		power.serviceMode = true;

		//for the local hardware UI to at least display the mode
		m_machineSE = M_SERVICE;
		m_stateLED = L_SERVICE;
		return true;
	}
	if (exitSM) {
		if (!power.serviceMode) return true;
		m_stateLED = L_NORMAL;
		m_machineSE = M_UNI_RUN;
		power.serviceMode = false;
		dccSE = DCC_LOCO;
		return true;
	}



	if (!verify) {
		//initiate Direct write

		if (cvReg > 1024)return false;
		//for service mode, cv is display location, i.e. 1 to 1024
		m_sm.cvReg = cvReg;
		/*update page and reg*/
		int v = m_sm.cvReg - 1;
		m_sm.pgPage = v / 4 + 1;
		m_sm.pgReg = v % 4;
		m_sm.cvData = cvVal;
		if (m_sm.cvData < 0) { return false; }
		dccSE = DCC_SERVICE;
		m_sm.timeout = 8;  //2 sec
		m_sm.state = D_START;
		ina219Mode(false); //2026-01-07 added
		return true;
		//after processing the machine reverts to m_cv.state=CV_IDLE


	}
	else {
		//read a byte. this is done asynchronously and takes up to 1 second
		//the cv lifecycle mechanisms will generate a return call to nsDCCweb when read is complete
		//after processing the machine reverts to m_cv.state=CV_IDLE

		//free to read?
		if (m_sm.state != CV_IDLE) return false;
		//initate a read
		m_sm.timeout = 8;  //2 sec
		//change to trigger mode and take one sample to initialise the ina
		ina219Mode(false);
		m_sm.state = RD_START;
		m_sm.bitCount = 7;
		m_sm.cvReg = cvReg;
		m_sm.cvData = 0;
		return true;
	};
	return false;
}




/// <summary>
/// check if Service Mode is busy with an operation. Does not check if we are actually in SM.
/// </summary>
/// <returns>true if busy</returns>
bool ServiceModeBusy(void) {
	if (m_sm.state != CV_IDLE) return true;
	return false;
}



/// <summary>
/// search for an existing address in the loco[] array slots.
/// If searchOnly, the routine is passive and will return the matching slot or -1.
/// If not searchOnly, the routine will take allocate to a blank slot if existing address not found
/// and if slots are all full, it will bump the oldest non-consist slot.
/// This function DOES NOT MODIFY data in the loco[] array.
/// </summary>
/// <param name="address">string such as L123 or S3 holding the loco address we seek</param>
/// <param name="existingAddress">returns address held in bumped slot, else a copy of address. Pass NULL if you do not want a return value</param>
/// <param name="searchOnly">will not attempt to allocate or bump a slot</param>
/// <returns>index of slot matching address or slot just allocated/bumped. returns -1 if operation failed.
/// slotAddress will equal address if address exists in a slot or we just allocated that address to a slot. It will return the bumped address if we bumped an old slot.
/// Test loco[returned index].address for zero to determine if we just allocated a blank slot.</returns>
int8_t findLoco(char* address, char* existingAddress, bool searchOnly) {
	bool useLong = (address[0] == 'L') ? true : false;
	if (existingAddress != NULL) { memset(existingAddress, '\0', sizeof(existingAddress)); }
	if (address == nullptr) return -1;

	
	int8_t i;
	char buf[8];
	trace(Serial.printf("findLoc addr=%s S/L=%d\n", address, useLong);)
		strcpy(buf, address + 1);  //ignore leading S/L on the address

	//do not match a zero address
	//Refactor: WiT did not check for zero address, but it never expected to receive one
	if (atoi(buf) == 0) return -1;


	//match exactly on address and short/long
	for (i = 0; i < MAX_LOCO; i++) {
		if ((loco[i].address == atoi(buf)) && (loco[i].useLongAddress == useLong)) {
			if (existingAddress != NULL) {
				sprintf(existingAddress, "S%d", loco[i].address);
				if (loco[i].useLongAddress) existingAddress[0] = 'L';
				trace(Serial.println("fL1");)
			}
			return i;
		}
	}

	//2021-01-08 if looking for a match only, then exit now, ignoring zero slots
	if (searchOnly) return -1;
	trace(Serial.println("fL2");)

		//or take an empty slot
		for (i = 0; i < MAX_LOCO; i++) {
			if (loco[i].address == 0) {
				//2021-10-02 zero-slot bug fix. Don't attempt to write to slotAddress if it is null
				//otherwise, copy address to slotAddress
				trace(Serial.printf("fL2a %d\n",i);)
					if (existingAddress != NULL) strcpy(existingAddress, address);
				//2025-02-19 why don't we return S0 to denote a blank slot is to be allocated?
				return i;


				//2025-11-05 now i am confused.  why don't we actually write the address to this slot?

			}
		}

	trace(Serial.println("fL3");)
		//refactor: bump logic carried from WiT
		//2020-11-25 new slot bump logic, look for oldest stationary loco that has no consistID
		int8_t bump = -1;
	uint16_t age = 0xFFFF;

	for (i = 0; i < MAX_LOCO; i++) {
		if ((loco[i].speed == 0) && (loco[i].consistID == 0)) {
			//possible bump candidate, but check its the oldest, i.e. lowest history value
			if (loco[i].history < age) {
				bump = i;
				age = loco[i].history;
			}
		}
	}

	/*return bump slot, or -1 if none available*/
	if (bump >= 0) {
		//pull existing slot address value
		if (existingAddress != NULL) {
			sprintf(existingAddress, "S%d", loco[bump].address);
			if (loco[bump].useLongAddress) existingAddress[0] = 'L';
		}
	}

	return bump;
}


/// <summary>
/// Find the next loco in the roster given a pointer to the current one
/// </summary>
/// <param name="loc">pointer to current loco</param>
/// <returns>next loco in the roster, or if there are no valid locos it will generate a default loco at address 3 and point to this</returns>
LOCO* getNextLoco(LOCO* loc) {
	//Note, *loc is a copy of the pointer, you cannot modify the pointer value (i.e. point at a new location)
	//so rather than the complex **loc pointer to a pointer syntax, its easier to make it  return value
	loc->jog = false;

	//find slot position of nominated loco
	int8_t thisLoco = setLoco(unithrottle.locPtr, 0, false);
	if ((thisLoco == -1) || (thisLoco >= MAX_LOCO)) {
		//cannot find a match, so point at last loco slot so that next will return first
		thisLoco = MAX_LOCO - 1;
	}

	//advance to next non-zero slot. To avoid a stuck-loop and WDT timeout fail after MAX_LOCO attempts
	for (int fail = 0; fail < MAX_LOCO; ++fail) {
		thisLoco++;
		if (thisLoco == MAX_LOCO) thisLoco = 0;
		if (loco[thisLoco].address != 0) break;
	}
	loc = &loco[thisLoco];

	//if the slot address is zero, means all slots must be zero so create loco S3
	if (loc->address == 0) {
		loc->address = 3;
		loc->use128 = false;
		loc->useLongAddress = false;
		loc->forward = true;
		loc->history = 0;
		memset(loc->name, '\0', sizeof(loc->name));
		bootController.isDirty = true;
		dccPutSettings();
	}
	loc->jog = true;

	return loc;
}


/// <summary>
/// Increase the history of target loco in the loco[] array
/// </summary>
/// <param name="loc">target loco</param>
void incrLocoHistory(LOCO* loc) {
	uint16_t age = 0;
	for (auto h : loco) {
		if (h.history > age) { age = h.history; }
	}
	age++;
	loc->history = age;
}

/// <summary>
/// Handles event raised from layer1 railcom routine if data is read or there is a timeout
/// Also handles ACK|NACK|BUSY if required
/// </summary>
/// <param name="result">railcom value</param>
/// <param name="ctrl">railcom ACK|NACK|BUSY</param>
/// <param name="success">true if good data|ctrl, false if a timeout</param>
void railcomCallback(uint8_t result, uint8_t ctrl, bool success) {
	//if the HUI is actively showing the POM screen, then repaint this
	
		//send a websocket transmission
		//we do this via a routine in DCCweb because all websockets are handled there
		//we need to populate addr, cvReg and payload (i.e. the cv return value)
		
		char addrType = 'S';
		addrType = m_pom.useAccessoryAddr ? 'A' : addrType;
		addrType = m_pom.useLongAddr ? 'L' : addrType;

		//2026-01-02 if we were looking for a ctrl char this will appear in the result
		//PROBLEM: result is 8 bit so ctrl is indistinguishable from data.
		//will fix with ctrl value, more work to be done


		if (success){
			nsDCCweb::broadcastPOMreadResult(m_pom.cvReg,result,addrType,m_pom.address);
		}
		else {
			nsDCCweb::broadcastPOMreadResult(m_pom.cvReg, -1, addrType, m_pom.address);
		}

		//2026-02-02 deal with callback to LocoNetprocessor (bool ack, u8 data)
		if (LocoNetcallbackPtr) {
			//for writes, we expect to see ctrl=ACK, we cannot differentiate between BUSY|NACK
			if (ctrl == 0) {
				//was a data read
				LocoNetcallbackPtr(success, result);
			}
			else {
				//write, expect ACK, return the ctrl value seen.  ctrl=NACK for timeouts, and success==false
				//success==true if we see any valid control chr
				LocoNetcallbackPtr(ctrl == 0xF0?true:false,  ctrl);
			}
			LocoNetcallbackPtr = nullptr;
		}

	if (m_machineSE != M_POM) return;

	//POM_BYTE_READ and POM_BIT_READ immediately decay to BYTE and BIT 

	if ((m_pom.state == POM_BYTE) || (m_pom.state == POM_BIT)) {
		//can safely repaint the UI
		m_pom.readSuccess = success;

		if (success) {
			m_pom.cvData = result;
			//<7> is the bit value, <0-2> the bit pos
			m_pom.cvBit &= 0b111;  //clear <7>
			if ((result & (1 << m_pom.cvBit)) != 0) m_pom.cvBit |= 0x80;
		}
		updatePOMdisplay();
	}
}

/// <summary>
/// sets DCCpacket to an extended loco speed packet which is used for railcom polling. This avoids need to have the target loco in
/// the slot roster.  If loco exists in the roster, then the slot speed/dir will be used, else it will default to zero speed.  Works
/// off values held in m_pom
/// </summary>

void createPOMpollingPacket(void) {
	uint8_t i;
	if (m_pom.useLongAddr) {
		/*long address format S9.2.1 para 60*/
		DCCpacket.data[0] = m_pom.address >> 8;
		DCCpacket.data[0] |= 0b11000000;
		DCCpacket.data[1] = m_pom.address & 0x00FF;
		i = 2;
	}
	else {
		DCCpacket.data[0] = (m_pom.address & 0x7F);
		i = 1;
	}

	//does this loco exist in the roster?
	char buf[8];
	int s;
	snprintf(buf, 8, "L%d", m_pom.address);
	buf[0] = m_pom.useLongAddr ? 'L' : 'S';
	s = findLoco(buf, nullptr, true);
	if (-1 == s) {
	//not in the roster, we need to set zero speed and forward dir
	/*two speed-bytes*/
		DCCpacket.data[i] = 0b00111111;
		i++;
		DCCpacket.data[i] = 0b10000000;
		i++;
	
	}
	else
	{//in the roster, transmit its speed
		uint8_t speedCode = loco[s].speedStep;  //UI display speed 0-28 or 0-128
		//assume we always use 128 steps
		if (speedCode > 0) { speedCode++; }
		speedCode &= 0x7F;

	 /*two speed-bytes*/
		DCCpacket.data[i] = 0b00111111;
		i++;
		/*mask in <7> which is direction*/
		if (loco[s].forward) { speedCode |= 0b10000000; }
		DCCpacket.data[i] = speedCode;
		i++;



	}

	/*calc checksum and packet length. i points to checksum byte*/
	DCCpacket.data[i] = 0;
	for (DCCpacket.packetLen = 0; DCCpacket.packetLen < i; DCCpacket.packetLen++) {
		DCCpacket.data[i] ^= DCCpacket.data[DCCpacket.packetLen];
	}
	DCCpacket.packetLen++;
	//will exit with DCCpacket.packetLen set at correct length of i+1

	DCCpacket.doCutout = true;

	
	
	

}

/// <summary>
/// Issue a dcc accessory command, independent of the local turnout roster
/// </summary>
/// <param name="addr">zero-based accessory address</param>
/// <param name="thrown">thrown position</param>
/// <param name="powerOn">set as true for turnouts, or on/off for signal aspects</param>
void actionAccessoryFromLocoNet(uint16_t addr, bool thrown, bool powerOn) {
	/*2026-02-05 DCCcore::updatelocalmachine is the only place where turnout[] data is copied across to the temporary accessory object which is then used by
	the dccPacket engine to send a command to line.  It is also the only place we set dccSE = DCC_ACCESSORY which will trigger the dccPacketEngine.
	updatelocalmachine is called on the INO loop, so it will pick up on a turnout.changeFlag and then trigger dccSE.  But this is async and possibilbly could cause
	confusion if the dccSE was in a POM mode rather than just a generic packet cycle.

	Therefore, for turnouts via loconet, we don't have to interact with the ESP turnout roster, we can just initiate commands independently.  Means the turnout roster is
	not synced to loconet activity, but I don't think that's a bad thing.
	*/


	switch (dccSE) {
	case DCC_POM:
	case DCC_SERVICE:
	case DCC_ACCESSORY:
		return;
	}

	//proceeed to set up an accessory packet.  Note that turnout "1" in the local turnout roster maps to addres 0b100
	//whereas Loconet is a zero based index, so turnout "1" is referred to as address 0, hence the need to add 1.
	accessory.address = addr+1;
	accessory.thrown = thrown;
	accessory.powerOn = powerOn;
	dccSE = DCC_ACCESSORY;
	trace(Serial.printf("actionAccessory %d\n", addr + 1);)

	//2026-02-16 if this turnout address happens to be the ESP roster, update that also
	//.changeFlag will ensure the local hardware display updates.
	//This code block does not attempt to add the address to the turnout roster, which is why we don't use findTurnout()
	for (auto& t : turnout) {
		if (t.address != addr + 1) continue;
		t.selected = true;
		t.thrown = thrown;
		t.changeFlag = true;
		return;
	}
}

/// <summary>
/// Injects a DCC packet as received in the LocoNet message stream. Builds a valid dcc packet with CRC and 
/// queues for transmission
/// </summary>
/// <param name="payload">array holding the payload</param>
/// <param name="payloadLength">bytes to transmit, max 5</param>
/// <param name="repeat">repeat multiple times in succession 0 is one transmission</param>
void actionDCCpacketFromLocoNet(uint8_t* payload, uint8_t payloadLength, uint8_t repeat) {
	
	switch (dccSE) {
	case DCC_POM:
	case DCC_SERVICE:
	case DCC_ACCESSORY:
		return;
	}
	
	m_LocoNetHoldingBuffer.railcomPacketCount = repeat & 0b111;

	uint8_t crc = 0;
	uint8_t i;
	if (payloadLength > 5) return;

	//payload is expected to be 5 bytes max
	for (i = 0;i < payloadLength;i++) {
		m_LocoNetHoldingBuffer.data[i] = payload[i];
		crc ^= payload[i];
	}
	//exit pointing at the crc byte
	m_LocoNetHoldingBuffer.data[i] = crc;
	m_LocoNetHoldingBuffer.packetLen = i + 1;
	m_LocoNetHoldingBuffer.longPreamble = false;
	m_LocoNetHoldingBuffer.doCutout = false;
	m_LocoNetHoldingBuffer.packetLen = payloadLength+1;

	//we have to buffer the LocoNet payload like this because DCCpacketEngine has to remain synchronous with
	//the CTS flag, and only change the DCCpacket buffer during this period.  Clearly the call to this function
	//will be asynchronous and might occur when CTS=false.  DCCpacket is not allowed to change during this time
	//and the pending loconet data would be lost.
	trace(Serial.println("actionDCCpacketFromLocoNet");)
	dccSE = DCC_IMMEDIATE;

}