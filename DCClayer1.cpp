// 
// 
// 
#include "Global.h"
#include "DCClayer1.h"
#include "DCCcore.h"  //need for railcom


/*DCClayer1 puts a DCC signal on the track.  It will continuously write the DCCbuffer to the track
The routine also sets a msTickFlag every 10mS which the main loop can use for general timing such as
keyboard scans.  It also generates a Railcom Cutout and looks for inbound Railcom data.

Note: using non PWM compat mode, the timebase is 200nS.

CAUTION: you cannot write consecutively to w1ts or w1tc, as the second write seems to null the first.
Instead you should OR the bits together e.g. gpio->w1ts = this | that
//https://arduino.stackexchange.com/questions/44531/arduino-esp8266-direct-fast-control-of-the-digital-pins

rtc_reg_write decprecated warning...TIMER_REG_WRITE
https://github.com/esp8266/Arduino/blob/master/tools/sdk/include/eagle_soc.h

2024-12-07 added railcom decode functionality

2025-02-05 PIN_RAILCOM_SYNC_TOTEM defines a pin which supports the railcom sync signal as a totem pole output
PIN_RAILCOM_SYNC_TOTEM defines a pin which supports railcom sync active low, but otherwise acts as an WPU input and detects a switch as active low

The hardware (blue PCB) needs a mod using a 2k2 resistor to pull the D8 pin (PIN_RAILCOM_SYNC_TOTEM) to ground.  This is because the ESP module 12k pulldown resistor is
insufficient to pull down against the WPU in the 6N137 Opto, and this causes boot to fail as the pin must be low at boot.  2k2 directly to ground fixes this, but it does require
an active totem drive to overcome this 2k2.

If we want to use the pin as PIN_RAILCOM_SYNC_INPUT then we need to have D8 drive a 2k2 base resistor into a NPN which will pull the Opto enable pin to ground.  We then drive
D8 with inverse logic, i.e. it is high during the railcom blanking period, and low during the railcom cutout.  It become a WPD (12k) input at DCC_CUTOUT_END when it is read and
DCCpacket.pinSyncInputTriggered is set if we see active hi, and must be cleared in some other processing routine.
The pushbutton must pull D8 to 3v3 via a 680R resistor.  This is because D8 is active low during the railcom cutout.

railcomLoop() must be called every program loop. Due to the bloated arduino stack, we cannot guarantee that the serial data captured by the hardware based serial decoder will be
available at the end of the railcom cutout period, and in any event, a POM read command in DCC may require a varying number of railcom cutouts following the specific loco addressed, 
before the decoder emits data.  In principle we can look for a railcom ACK message, post POM-write, but the calls from DCCcore do not support this at present.  Additionally, it is not
possible to indentify whether the data appeared in Channel 1 or Channel 2.  This is because it is not possible to monitor hardware-serial events in real time.  
Channel 1 is always a 12 bit datagram, and is a broadcast channel.  We don't use it.  Channel 2 will only contain data related to the decoder whose address was sent in the DCC packet
preceeding that railcom cutout.   DCClayer1 will call railcomCallback() in DCCcore after a POM read
Note: as 12 bit datagram will contain a single 8 bit response value as well as an ID0 marker.

BUG: POM read sometimes reads zero. this is not a timeout. it means the decoded serial buffer contained two bytes forming a valid 12 bit datagram.

*/




#include <c_types.h>
#include <pwm.h>
#include <eagle_soc.h>
#include <ets_sys.h>
//now adding gpio.h for function GPIO_PIN_ADDR
#include <gpio.h>

#ifndef SDK_PWM_PERIOD_COMPAT_MODE
#define SDK_PWM_PERIOD_COMPAT_MODE 0
#endif
#define PWM_USE_NMI 0


/*notes.
 * compat mode 0.  works with nmI = 0 or 1
 * nmi=1  does not work with webserver present
 * nmi=0  does work with webserver present
 * compat mode 1.  does not work with mni=1
*/

#if SDK_PWM_PERIOD_COMPAT_MODE
/*ticks are 1uS. This may not work because minimum reload value needs to be >100*/
#define ticksZERO 116  //116uS half cycles for DCC zero
#define ticksONE  58  //58uS half cycles for DCC one
#define ticksMS  172  //10mS interval

#else
/*ticks are 200nS*/
#define ticksZERO 580  //116uS half cycles for DCC zero
#define ticksONE  281  //58uS half cycles for DCC one, was 290 tweaked to 281
#define ticksMS  172  //10mS interval.  will need adjusting from 172
#define ticksMSfast  17 //1mS interval.
#define ticksCutoutStart  130//26uS 
#define ticksCutout 2250  //450uS
#define ticksCutoutEnd 60 //11uS
#endif

//note dccCount increments for every 1-bit half period and is doubled for every 0-bit half period
//when we hit a cutout, we need to add 7 to it



// from SDK hw_timer.c
#define TIMER1_DIVIDE_BY_16             0x0004
#define TIMER1_DIVIDE_BY_256            0x0008
#define TIMER1_ENABLE_TIMER             0x0080

//https://esp8266.ru/esp8266-gpio-register/
struct gpio_regs {
	uint32_t out;         /* 0x60000300 entire output reg*/
	uint32_t out_w1ts;    /* 0x60000304 selective outputs hi*/
	uint32_t out_w1tc;    /* 0x60000308 selective outputs low*/
	uint32_t enable;      /* 0x6000030C enable outputs (hi) or inputs (low)*/
	uint32_t enable_w1ts; /* 0x60000310 seletive IO as output*/
	uint32_t enable_w1tc; /* 0x60000314 selective IO as input*/
	uint32_t in;          /* 0x60000318 Input level when IO is an input*/
	uint32_t status;      /* 0x6000031C interrupt status*/
	uint32_t status_w1ts; /* 0x60000320 */
	uint32_t status_w1tc; /* 0x60000324 */
};
static struct gpio_regs* gpio = (struct gpio_regs*)(0x60000300);

struct timer_regs {
	uint32_t frc1_load;   /* 0x60000600 */
	uint32_t frc1_count;  /* 0x60000604 */
	uint32_t frc1_ctrl;   /* 0x60000608 */
	uint32_t frc1_int;    /* 0x6000060C */
	uint8_t  pad[16];
	uint32_t frc2_load;   /* 0x60000620 */
	uint32_t frc2_count;  /* 0x60000624 */
	uint32_t frc2_ctrl;   /* 0x60000628 */
	uint32_t frc2_int;    /* 0x6000062C */
	uint32_t frc2_alarm;  /* 0x60000630 */
};
static struct timer_regs* timer = (struct timer_regs*)(0x60000600);

volatile DCCBUFFER DCCpacket;  //externally visible
volatile DCCBUFFER _TXbuffer;   //internal to this module

static uint16_t dcc_mask = 0;
static uint16_t dcc_maskInverse = 0;   //if this remains zero, we are using an LMD device
static uint16_t enable_mask = 0;
static uint16_t sync_mask = 0;  //always defined but if it remains zero, railcom is not enabled.

static uint8_t  dccCount;

enum DCCbit { DCC_ONE_H, DCC_ONE_L, DCC_ZERO_H, DCC_ZERO_L, TEST_H, TEST_L, DCC_CUTOUT_START, DCC_CUTOUT, DCC_CUTOUT_END };
static enum DCCbit DCCperiod = DCC_ONE_H;


//DCC layer 1 
volatile uint8_t  TXbyteCount;
volatile uint8_t  TXbitCount = 32;


//S-9.3.2 table 2: Railcom 4/8 decode table. The last 3 entries are NACK, ACK and BUSY
//BUSY potentially might be encountered on reads before the read value appears
//BUSY, ACK, NACK will appear in response to a POM write.  ACK means the command was received, not that the data was actually written.
const uint8_t railcomTable[] = {
0b10101100,0b10101010,0b10101001,0b10100101,0b10100011,0b10100110,0b10011100,0b10011010,0b10011001,0b10010101,0b10010011,0b10010110,0b10001110,0b10001101,0b10001011,0b10110001,
0b10110010,0b10110100,0b10111000,0b01110100,0b01110010,0b01101100,0b01101010,0b01101001,0b01100101,0b01100011,0b01100110,0b01011100,0b01011010,0b01011001,0b01010101,0b01010011,
0b01010110,0b01001110,0b01001101,0b01001011,0b01000111,0b01110001,0b11101000,0b11100100,0b11100010,0b11010001,0b11001001,0b11000101,0b11011000,0b11010100,0b11010010,0b11001010,
0b11000110,0b11001100,0b01111000,0b00010111,0b00011011,0b00011101,0b00011110,0b00101110,0b00110110,0b00111010,0b00100111,0b00101011,0b00101101,0b00110101,0b00111001,0b00110011,
0b00001111,0b11110000,0b11100001};
#define RAILCOMTABLELENGTH	67

//#define RC_NACK 0x40
//#define RC_ACK 0x41
//#define RC_BUSY 0x42


uint8_t _rcstate;
uint8_t _payload;


enum RC_STATE
{
	RC_EXPECT_ID0,
	RC_EXPECT_BYTE,
	RC_EXPECT_CTRL,
	RC_SUCCESS,
	RC_TIMEOUT	
};

# define LMD18200T_DEVICE	0


/*Interrupt handler for DCC
  for a dcc_zero or dcc_one the reload periods are different.  We queue up the next-bit in the second half of the bit currently being transmitted
  Some jitter is inevitable with maskable Ints, but it does not cause any problems with decooding in the locos at present.
  The handler will work its way through the TXbuffer transmitting each byte.  when it reaches the end, it sets the byte pointer to zero
  and sets the bitcounter to 22 indicating a preamble.  at this point it clears the DCCclearToSend flag to lock the DCCpacket buffer from writes for the next
  12 preamble bits this is so that the main prog loop routines have long enough to finish writing to the DCCpacket  if they had just missed the flag clearing.
  Once preamble is transmitted, the handler will copy the DCCpacket to the transmit buffer and set the DCCclearToSend flag indicating it is able to accept
  a new packet.  If the DDCpacket is not modified by the main loop, this layer 1 handler will continuously transmit the same packet to line.  This is useful
  as it allows an idle to be continuously transmitted when we are in Service Mode for example.

2023-08-30 modified to add a railcom cutout at end of packet.  Set doCutout when passing the DCCpacket to the routine.
The NRMA spec calls for a cutout after every loco (and accessory) packet sent.  The exception is during service mode.
In theory a loco will recognise packets sent to it, and it will respond during the cutout.  This avoids all locos trying to assert on the same cutout.

In theory, a cutout always starts after the 2nd half of a bit period and this is always a logic 0.  There is a transition to 'high' for Tcs
after which the cutout asserts.  The cutout ends with a logic 0 which is a psudeo end to an imaginary bit.  In any event the next bit start will be a logic 1.

The cutout start bit is asymetric, i.e. it has no complementary low-start bit of equal length.

*/


static void IRAM_ATTR dcc_intr_handler(void) {
	/*set the period based on the bit-type we queued up in the last interrupt*/

	switch (DCCperiod) {
	case DCC_ZERO_H:
		WRITE_PERI_REG(&timer->frc1_load, ticksZERO);
		gpio->out_w1ts = dcc_mask;  //set bits to logic 1
		gpio->out_w1tc = dcc_maskInverse;  //set bits to logic 0
		dccCount++;
		break;
	case DCC_ZERO_L:
		WRITE_PERI_REG(&timer->frc1_load, ticksZERO);
		gpio->out_w1ts = dcc_maskInverse;  //set bits to logic 1
		gpio->out_w1tc = dcc_mask;  //set bits to logic 0
		dccCount++;
		break;
	case DCC_ONE_H:
		WRITE_PERI_REG(&timer->frc1_load, ticksONE);
		gpio->out_w1ts = dcc_mask;  //set bits to logic 1
		gpio->out_w1tc = dcc_maskInverse;  //set bits to logic 0
		break;
	case DCC_ONE_L:
		WRITE_PERI_REG(&timer->frc1_load, ticksONE);
		gpio->out_w1ts = dcc_maskInverse;  //set bits to logic 1
		gpio->out_w1tc = dcc_mask;  //set bits to logic 0
		break;
	case DCC_CUTOUT_START:
		WRITE_PERI_REG(&timer->frc1_load, ticksCutoutStart);
		//this is a pseudo start to a new 1 bit, we high akin to _H on the other bit types
		//Railcom sync is in blanking state at this point, and only changes at the start and end of the actual cutout
		
		if (dcc_maskInverse == LMD18200T_DEVICE) {	//zero indictes this is a LMD18200T device
			gpio->out_w1ts = dcc_mask;
		}
		else {//L298 or IBT device
			gpio->out_w1ts = dcc_mask;
			gpio->out_w1tc = dcc_maskInverse;
		}
		break;

	case DCC_CUTOUT:
		WRITE_PERI_REG(&timer->frc1_load, ticksCutout);
		//assert a railcom cutout by taking all H bridge outputs to logic low and asserting ground on both rails
		//the H bridge needs to remain enabled.  dcc_sync is asserted during the cutout period.

		if (dcc_maskInverse == LMD18200T_DEVICE) {
			//LMD device 2 pin mode, take its enable pin (PWM) low, this will drive both X+Y high
#ifdef PIN_RAILCOM_SYNC_INPUT
				gpio->out_w1tc = enable_mask | sync_mask;	
#else 
				gpio->out_w1tc = enable_mask;
				gpio->out_w1ts = sync_mask;
#endif

		}
		else
		{
			//L298+IBT devices, enable is controlled in the power trip block.  We need to assert both X+Y lines low
			//remember X is controlled by dcc_mask, Y by dcc_maskInverse;
				
#ifdef PIN_RAILCOM_SYNC_INPUT
					gpio->out_w1tc = dcc_mask | dcc_maskInverse | sync_mask;
				
#else 
					gpio->out_w1tc = dcc_mask | dcc_maskInverse;
					gpio->out_w1ts = sync_mask;
#endif


		}

		dccCount += 7;
		break;

	case DCC_CUTOUT_END:
#ifdef PIN_RAILCOM_SYNC_INPUT
		//pin_railcom as input. Do this now, because it takes a finite time (<1uS) for the input value to propage to gpio->in
		//you cannot set as input and read immediately
		gpio->enable_w1tc = sync_mask;   
#endif
		WRITE_PERI_REG(&timer->frc1_load, ticksCutoutEnd);
		//this is a pseudo end of a bit, assert a low-half of a bit
		//at the end of the cutout, we need to re-assert enable as BAU level (assuming power is on)
		//at the end of the cutout we re-establish railcom blanking 
		

		if (dcc_maskInverse == LMD18200T_DEVICE) {
			//LMD device 2 pin mode, take its enable pin (PWM) high, provided we are not in power cutout
			
#ifdef PIN_RAILCOM_SYNC_INPUT
			//read input, then switch pin_railcom to be an output and assert railcom blanking
			if ((READ_PERI_REG(&gpio->in) & sync_mask) != 0) { DCCpacket.pinSyncInputEvent = true; }
			gpio->enable_w1ts = sync_mask; //pin_railcom as output
			if (DCCpacket.trackPower) gpio->out_w1ts = enable_mask | sync_mask;	
#else
			if (DCCpacket.trackPower) gpio->out_w1ts = enable_mask;
			gpio->out_w1tc = sync_mask;
#endif
		}
		else
		{//L298+IBT devices, enable is controlled in the power trip block.
			
#ifdef PIN_RAILCOM_SYNC_INPUT
			//read input and revert pin_railcom to output
			if ((READ_PERI_REG(&gpio->in) & sync_mask) != 0) { DCCpacket.pinSyncInputEvent = true; }
			gpio->enable_w1ts = sync_mask;

			gpio->out_w1ts = dcc_mask | sync_mask;
			gpio->out_w1tc = dcc_maskInverse ;
#else
			gpio->out_w1ts = dcc_mask;
			gpio->out_w1tc = dcc_maskInverse | sync_mask;
#endif
		
//we exit DCC_CUTOUT_END with railcom blanking set, this does not change (PIN_RAILCOM_SYNC_) until start of next DCC_CUTOUT state
//railcom blanking = disable the 6n137 opto isolator output
		
		}


		break;
		/*the delay gets executed and the block below sets next-state and queues up next databit*/
	}


	timer->frc1_int &= ~FRC1_INT_CLR_MASK;
	//this memory barrier compiler instruction is left in from the code I leveraged
	asm volatile ("" : : : "memory");

	//POWER TRIP: do power assert here, before we change DCCperiod in the next block
	//i.e. at this point we just asserted the current DCCperiod state, the next block queues up the next-state
	if (++dccCount >= ticksMS) {
		dccCount = 0;
		DCCpacket.msTickFlag = true;

		//2024-12-12 we do not want to accidentally modify the enable pins that were set during the cutout period		

		if ((dcc_maskInverse == 0) && (DCCperiod != DCC_CUTOUT)) {
			//LMD in 2 pin mode.  DCC_CUTOUT is a special period as this requires PWM (enable) to stay low
			//so this is a do nothing block because PWM (enable) was already set in the bit state above. 
		}
		else
		{//L298 and IBT devices, control based on the enable pin in all DCC states
		//plus LMD outside of the DCC_CUTOUT state is also based on the enable pin
			if (DCCpacket.trackPower) { gpio->out_w1ts = enable_mask; }
			else { gpio->out_w1tc = enable_mask; }
		}
	}



	switch (DCCperiod) {
	case DCC_ZERO_H:
		DCCperiod = DCC_ZERO_L;   //queue up second part of the zero bit
		break;
	case DCC_ONE_H:
		DCCperiod = DCC_ONE_L;   //queue up second part of the one bit
		break;

	default:
		/*if executing the low part of a DCC zero or DCC one, then advance bit sequence and queue up next bit */
		//2023-09-25 changed the way this works.  Packets are transmitted, and then followed by optionally a railcom cutout and then a preamble (before the next packet)
		//we will set up an extended pre-amble, load the next packet and if this does not require an extended pre-amble we will shorten the current preamble to 14 bits
		//At the end of the current packet, we decide if we need a railcom cutout or not.  if we do, we process this before the preamble, if not, then we go straight into 
		//the preamble.   DCC cutouts have 3 pseudo bits, a setup of 23us, a cutout of 455 uS and a close of 10uS followed by pre-amble
		//TXbitcount 33,34,35 are for DCC cutout
		//TXbitcount of 32 is a long preamble, 23 is a short preamble


		//note this default section will execute for zero_l and one_l, plus all the DCC_cutouts
		//default period
		DCCperiod = DCC_ONE_H;  //default


		//bug, we get a slightly lengthened 1-hi period for bit 36, why?  should be 58, it is 85uS (27 too long).  a zero-half is 116
		//is there a processing delay?

		switch (TXbitCount) {
		case 36:
			//do nothing, this is a stop-bit, a 1 as a precursor to the DCC cutout, its a DCC_ONE_H
			break;
		case 35:
			DCCperiod = DCC_CUTOUT_START;
			break;
		case 34:
			DCCperiod = DCC_CUTOUT;
			break;

		case 33:
			DCCperiod = DCC_CUTOUT_END;
			break;

			//note, case 32-19 are preamble bits, which by befault are transmitted as 1's per DCC_ONE_H default above.

		case 18:
			//preamble is either 24bits or 14 bits long.  We signal .clearToSend=false at the end of the previous packet, but this possibly was in the middle of DCCcore to writing
			//to the buffer because this int may have occured then.  To allow the main routine to finish that write, we don't read the buffer until we have completed at least 14 bits of preamble
			//and now REALLY need the buffer contents.  14bits = 1624uS of preamble, i.e. a 1624uS blanking period to finish the write.

			/*copy DCCpacket to TXbuffer. memcpy would be slower than direct assignment
			immediately after data is copied, set DCCclearToSend which flags to DCCcore module that a new
			DCCpacket may be written
			2019-12-05 increased to 6 packet buffer with copy-over*/

			_TXbuffer.data[0] = DCCpacket.data[0];
			_TXbuffer.data[1] = DCCpacket.data[1];
			_TXbuffer.data[2] = DCCpacket.data[2];
			_TXbuffer.data[3] = DCCpacket.data[3];
			_TXbuffer.data[4] = DCCpacket.data[4];
			_TXbuffer.data[5] = DCCpacket.data[5];
			_TXbuffer.packetLen = DCCpacket.packetLen;
			_TXbuffer.longPreamble = DCCpacket.longPreamble;
			_TXbuffer.doCutout = DCCpacket.doCutout;

			TXbyteCount = 0;
			DCCpacket.clearToSend = true;

			if (!_TXbuffer.longPreamble) {
				//we have already covered the standard 14 bits preamble, so trigger the start bit 
				TXbitCount = 9;
			}
			break;

		case 8:
			if (TXbyteCount != _TXbuffer.packetLen) {
				DCCperiod = DCC_ZERO_H;  //queue up a zero separator
				break;
			}
			//end of packet
			DCCpacket.clearToSend = false;
			if (_TXbuffer.doCutout) {
				//BUG even if .doCutout=false, this block gets executed.  why?  maybe bool does not copy over?  Debugger says yes it is set, and then later it is cleared
				_TXbuffer.doCutout = false; //no retrigger
				TXbitCount = 37;  //will be 36 on exit which will run into the cutout
				//TXbitCount = 33;  //will be 32 on exit   //debug huh, why is the cutout active?
			}
			else
			{//reached end of packet, trigger a long preamble as the default
				TXbitCount = 33;  //will be 32 on exit
			}

			//2024-11-8 every complete packet transmission clocks down the railcomPacketCount
			DCCpacket.railcomPacketCount -= DCCpacket.railcomPacketCount > 0 ? 1 : 0;
			break;

		default:
			if (TXbitCount > 7) break;
			//must be 7-0, queue up databit
			if ((_TXbuffer.data[TXbyteCount] & (1 << TXbitCount)) == 0)
			{//queue a zero
				DCCperiod = DCC_ZERO_H; //queue up a zero
			}

			/*special case bit 0, assert bit but set bit count as 9 as it immediatley decrements to 8 on exit*/
			if (TXbitCount == 0) { TXbyteCount++; TXbitCount = 9; }
		}

		TXbitCount--;

	}//end DCC period switch



	//one millisecond fast tick flag
	DCCpacket.fastTickFlag = ((dccCount % ticksMSfast) == 0) ? true : false;


}



/// <summary>
/// Initialisation. call repeatedly to activate additional DCC outputs
/// </summary>
/// <param name="pin_dcc">GPIO pin to carry DCC signal</param>
/// <param name="pin_enable">GPIO pin to enable power. Active high.</param>
/// <param name="phase">phase of DCC signal</param>
void dcc_init(uint32_t pin_dcc, uint32_t pin_enable, bool phase)
{
	//load with an IDLE packet
	DCCpacket.data[0] = 0xFF;
	DCCpacket.data[1] = 0;
	DCCpacket.data[2] = 0xFF;
	DCCpacket.packetLen = 3;

	pinMode(pin_dcc, OUTPUT);
	pinMode(pin_enable, OUTPUT);


	if (phase) {
		dcc_mask |= (1 << pin_dcc);
	}
	else {
		dcc_maskInverse |= (1 << pin_dcc);
	}

	//set up enable pin(s)
	enable_mask |= (1 << pin_enable);






#if PWM_USE_NMI
	ETS_FRC_TIMER1_NMI_INTR_ATTACH(dcc_intr_handler);
#else
	ETS_FRC_TIMER1_INTR_ATTACH(dcc_intr_handler, NULL);

#endif

	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();
	TIMER_REG_WRITE(FRC1_LOAD_ADDRESS, 0);  //This starts timer.  +++++++++ RTC_REG_WRITE is deprecated ++++++
	timer->frc1_ctrl = TIMER1_DIVIDE_BY_16 | TIMER1_ENABLE_TIMER;
}





#pragma region RAILCOM

/// <summary>
/// Initialise UART for railcom, start listening for incoming data
/// Also configure the railcom sync pin either not at all, as totem pole or as a IO line
/// </summary>
void railcomInit() {

#ifdef PIN_RAILCOM_SYNC_INPUT
//Pin will drive as totem pole, but inverted as it controls an NPN
//ISR with briefly switch to INPUT at very end of RC cutout and read.
//it is first set as an output and driven high
	pinMode(PIN_RAILCOM_SYNC_INPUT, OUTPUT);
	digitalWrite(PIN_RAILCOM_SYNC_INPUT ,HIGH);
	sync_mask |= (1 << PIN_RAILCOM_SYNC_INPUT);
	Serial.println(F("\n\nrailcom sync input"));
#elif PIN_RAILCOM_SYNC_TOTEM
	//Railcom, make this sync pin a totem-pole output
	pinMode(PIN_RAILCOM_SYNC_TOTEM, OUTPUT);
	digitalWrite(PIN_RAILCOM_SYNC_TOTEM, HIGH);
	Serial.println(F("\n\nrailcom sync totem"));
	sync_mask |= (1 << PIN_RAILCOM_SYNC_TOTEM);
#else
	_rcstate = RC_EXPECT_ID0;
	//exit without reconfiguring serial
	//note that dcc_sync will remain as zero and thus have no effect on set/clear gpio bits
	return;
#endif // !PIN_RAILCOM_SYNC

	Serial.flush();
	railcomRead(false);
	
	//railcom uses 250kbaud, don't enable this baud rate if we are compiling for TRACE because trace needs the serial port
#ifndef TRACE
	Serial.end();
	Serial.begin(250000);
#endif // !TRACE

	_rcstate = RC_EXPECT_ID0;

}

/// <summary>
/// Call once per program loop, reads any railcom data that has accumulated in the hardware-serial buffer.
/// This process is asynchronous to the actual railcom cutout that initiated the railcom read, because
/// the arduino stack is too bloated to ensure a serial read output by the end of the railcom cutout.
/// </summary>
void railcomLoop(void) {
//2025-12-29 note: this code expects to find ID0 with 2msb of a 8 bit data result. if it sees ID0 it expects 1 more byte, even though 4 bytes total are possible with ID0.
//for 12 bit datagrams, the payload is IIIIbb bbbbbb, where IIII is the ID which is 0 to 15 (i.e. 4 bits).  see table 4 ID[3-0]D[7-6]+D[5-0] i.e. 4 bits ID with bits <7,6> of data
//followed by <5-0> data.  Its possible that a 3rd 6-bit datagram follows, being ACK, NACK or BUSY

//2025-12-29 I sometimes see spurious zero reads.  This is only possible if dual zero bytes are present in the serial buffer



#ifdef PIN_RAILCOM_SYNC_TOTEM
#elif PIN_RAILCOM_SYNC_INPUT
#else
	//railcom not supported
	return;
#endif

	uint8_t byteNew;
	uint8_t	byteCount = 0;

	while (Serial.available() > 0) {
		// read the incoming byte:
		byteNew = Serial.read();

		switch (_rcstate) {
		case RC_EXPECT_BYTE:
			if (decodeRailcom(&byteNew, true)) {
				_payload += (byteNew & 0b00111111);

				//incoming websocket or HUI button pushes were processed in DCCcore.  Once layer1 decodes a railcom message
				//it needs to call back to DCCcore to process the valid payload.
				railcomCallback(_payload, 0,true);
			
			}
			//stop looking for incoming messages
			_rcstate = RC_SUCCESS;
			break;

		case RC_SUCCESS:
		case RC_TIMEOUT:
			break;

		case RC_EXPECT_CTRL:
			//expect a control byte 
			if (decodeRailcom(&byteNew, false)) {
				//is this a control char?
				switch (byteNew) {
				case 0x0F:  //NACK
				case 0xF0:  //ACK
				case 0xF1:  //BUSY
					railcomCallback(0,byteNew, true);
					//not always success as it might be NACK or BUSY but its not a timeout
					_rcstate = RC_SUCCESS;
				}
				//data payloads are ignored
			}
			break;

		default:
			if (decodeRailcom(&byteNew, true)) {
				if ((byteNew & 0b00111100) == 0) {
					//found ID0
					//2025-12-29 what, how?  why AND with 0x3C?
					_payload = byteNew << 6;  //two lsb become two msb
					_rcstate = RC_EXPECT_BYTE;
					break;
				}
			}
			_rcstate = RC_EXPECT_ID0;
			break;

		}

		//saftey valve,  process max 20 bytes before giving control back to main loop
		if (++byteCount > 20) break;
	}


	//timed out?

	if (DCCpacket.railcomPacketCount == 0) {
		switch (_rcstate) {
		case RC_SUCCESS:
		case RC_TIMEOUT:
			break;

		default:
			//send this message ONCE, and send NACK for timeouts
			railcomCallback(0,0x0F, false);
			_rcstate = RC_TIMEOUT;
		}
	}


}



//think again. if I have m_pom.expectACK then when we truely read, we are waiting on data or a timeout.  if we write, then we write a dccpacket and then wait on
//railcom for ACK or timeout

/// <summary>
/// Reset railcom reader.  Does not support accessory addresses.
/// </summary>
/// <param name="lookForCTRL">look for ACK|NACK|BUSY</param>
void railcomRead(bool lookForCTRL) {
#ifdef DEBUG_RC
	return;
#endif 
	//S-9.3.2 section 3.1
	DCCpacket.railcomPacketCount = 80;
	_rcstate = RC_EXPECT_ID0;
	if (lookForCTRL) _rcstate = RC_EXPECT_CTRL;
}

/*
/// <summary>
/// decode inbound data against the 4/8 decode table.		WE DON'T NEED THIS OVERLOAD
/// </summary>
/// <param name="inByte">4/8 coded serial data inbound</param>
/// <param name="dataOut">the decoded byte</param>
/// <param name="ignoreControlChars">ignore ACK, NACK, BUSY and only return data values</param>
/// <returns></returns>
bool decodeRailcom(uint8_t inByte, uint8_t* dataOut, bool ignoreControlChars) {
	for (int i = 0; i <= RC_BUSY; i++) {
		if (inByte == railcomTable[i]) {
			//valid
			*dataOut = i;
			return true;
		}
		if (ignoreControlChars && (i >= 0x3F)) return false;
	}
	//didn't find a match
	*dataOut = 0;
	return false;
}
*/

/// <summary>
/// decode inbound data against the 4/8 decode table, overwriting inByte with its decoded value
/// </summary>
/// <param name="inByte">4/8 coded serial data inbound, overwritten with decoded value</param>
/// <param name="ignoreControlChars">ignore ACK, NACK, BUSY and only return data values</param>
/// <returns>true if decode successful</returns>
bool decodeRailcom(uint8_t* inByte, bool ignoreControlChars) {
	
	for (int i = 0; i < RAILCOMTABLELENGTH; i++) {
		if (*inByte == railcomTable[i]) {
			//valid, but is it a control char (the last 3 entries in the table)
			if (ignoreControlChars && (i >= 0x3F)) return false;
			*inByte = i;
			return true;
		}
		
	}
	//didn't find a match
	return false;
}



#pragma endregion