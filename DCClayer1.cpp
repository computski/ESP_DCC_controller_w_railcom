// 
// 
// 

#include "DCClayer1.h"

/*DCClayer1 puts a DCC signal on the track.  It will continuously write the DCCbuffer to the track
The routine also sets a msTickFlag every 10mS which the main loop can use for general timing such as
keyboard scans.

Note: using non PWM compat mode, the timebase is 200nS.
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

/* no user servicable parts beyond this point */

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


	struct gpio_regs {
		uint32_t out;         /* 0x60000300 */
		uint32_t out_w1ts;    /* 0x60000304 */
		uint32_t out_w1tc;    /* 0x60000308 */
		uint32_t enable;      /* 0x6000030C */
		uint32_t enable_w1ts; /* 0x60000310 */
		uint32_t enable_w1tc; /* 0x60000314 */
		uint32_t in;          /* 0x60000318 */
		uint32_t status;      /* 0x6000031C */
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
	static uint16_t dcc_maskInverse = 0;
	static uint16_t enable_mask = 0;
	static uint16_t enable_maskInverse = 0;

	static uint8_t  dccCount;

	enum DCCbit { DCC_ONE_H, DCC_ONE_L, DCC_ZERO_H, DCC_ZERO_L, TEST_H, TEST_L , DCC_CUTOUT_START, DCC_CUTOUT, DCC_CUTOUT_END};
	static enum DCCbit DCCperiod = DCC_ONE_H;

	
	//DCC layer 1 
	volatile uint8_t  TXbyteCount;
	volatile uint8_t  TXbitCount=32;



	/*Interrupt handler ffor dcc
	  for a dcc_zero or dcc_one the reload periods are different.  We queue up the next-bit in the second half of the bit currently being transmitted
	  Some jitter is inevitable with maskable Ints, but it does not cause any problems with decooding in the locos at present.
	  The handler will work its way through the TXbuffer transmitting each byte.  when it reaches the end, it sets the byte pointer to zero
	  and sets the bitcounter to 22 indicating a preamble.  at this point it clears the DCCclearToSend flag to lock the DCCpacket buffer from writes for the next
	  12 preamble bits this is so that the main prog loop routines have long enough to finish writing to the DCCpacket  if they had just missed the flag clearing.
	  Once preamble is transmitted, the handler will copy the DCCpacket to the transmit buffer and set the DCCclearToSend flag indicating it is able to accept
	  a new packet.  If the DDCpacket is not modified by the main loop, this layer 1 handler will continuously transmit the same packet to line.  This is useful
	  as it allows an idle to be continuously transmitted when we are in Service Mode for example.
	
	2023-08-30 modified to add a railcom cutout at end of packet.  it needs to know how to assert a short-circuit on the outputs and how to time this event
	The NRMA is not clear on when the cutout is asserted, it suggests at the end of every packet
	other specs I have seen state the cutout must be preceded by an idle packet, which implies only idle packets should have a cutout

	In theory, a cutout always starts after the 2nd half of a bit period and this is always a logic 0.  There is a transition to 'high' for Tcs
	after which the cutout asserts.  The cutout ends with a logic 0 which is a psudeo end to an imaginary bit.  In any event the next bit start will be a logic 1.


	*/


	//static void ICACHE_RAM_ATTR dcc_intr_handler(void) //deprecated
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
			//this is a pseudo start to a new bit
			gpio->out_w1ts = dcc_mask;  //set bits to logic 1
			gpio->out_w1tc = dcc_maskInverse;  //set bits to logic 0
			break;
		case DCC_CUTOUT:
			WRITE_PERI_REG(&timer->frc1_load, ticksCutout);
			//assert a railcom cutout.
			//IMPORTANT: on some systems, enable makes the power go open circuit, in which case the cutout needs to be facilitated through the dcc drive pins
			//rather than enable.  If we execute through enable, the dcc outputs remain as they were in the preceeding DCC_ONE_L (as an IDLE preceeds the cutout)
			gpio->out_w1ts = enable_maskInverse;
			gpio->out_w1tc = enable_mask; 
			dccCount += 7;
			break;
		case DCC_CUTOUT_END:
			WRITE_PERI_REG(&timer->frc1_load, ticksCutoutEnd);
			//this is a pseudo end of a bit. re-enable power but also assert a low
			//you cannot re-write to same gpio register quickly as first instruction is not acted on.  instead, OR the commands together
			//2023-09-29 only re-enable power if trip is not active
			if (!DCCpacket.trackPower) break;
			gpio->out_w1ts = enable_mask | dcc_maskInverse;
			gpio->out_w1tc = enable_maskInverse | dcc_mask;
			break;
			/*the delay gets executed and the block below sets next-state and queues up next databit*/
		}


		timer->frc1_int &= ~FRC1_INT_CLR_MASK;
		//this memory barrier compiler instruction is left in from the code I leveraged
		asm volatile ("" : : : "memory");

		//do power assert here, before we change DCCperiod in the next block
		if (++dccCount >= ticksMS) {
			dccCount = 0;
			DCCpacket.msTickFlag = true;
			if (DCCperiod != DCC_CUTOUT) {
				//only assert system power on/off outside of cutouts
				gpio->out_w1ts = DCCpacket.trackPower ? enable_mask : enable_maskInverse;
				gpio->out_w1tc = DCCpacket.trackPower ? enable_maskInverse : enable_mask;
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


	//Initialisation. call repeatedly to activate additional DCC outputs
	//void ICACHE_FLASH_ATTR dcc_init (uint32_t pin_pwm,uint32_t pin_enable,bool phase, bool invert)   //ICACHE_FLASH_ATTR is deprecated
	void IRAM_ATTR dcc_init(uint32_t pin_pwm, uint32_t pin_enable, bool phase, bool invert)
	{
		//load with an IDLE packet
		DCCpacket.data[0] = 0xFF;
		DCCpacket.data[1] = 0;
		DCCpacket.data[2] = 0xFF;
		DCCpacket.packetLen = 3;
		
		pinMode(pin_pwm, OUTPUT);
		pinMode(pin_enable, OUTPUT);

		if (phase) {
			dcc_mask |= (1 << pin_pwm);
		}
		else {
			dcc_maskInverse |= (1 << pin_pwm);
		}
		
		/*set up enable pin(s)*/
		if (invert) {
			enable_maskInverse |= (1 << pin_enable);
		}
		else {
			enable_mask |= (1 << pin_enable);
		}
		

#if PWM_USE_NMI
		ETS_FRC_TIMER1_NMI_INTR_ATTACH(dcc_intr_handler);
#else
		ETS_FRC_TIMER1_INTR_ATTACH(dcc_intr_handler, NULL);  

#endif

		TM1_EDGE_INT_ENABLE();
		ETS_FRC1_INTR_ENABLE();
		RTC_REG_WRITE(FRC1_LOAD_ADDRESS, 0);  //This starts timer.  +++++++++ RTC_REG_WRITE is deprecated ++++++
		timer->frc1_ctrl = TIMER1_DIVIDE_BY_16 | TIMER1_ENABLE_TIMER;
	}


//DC pwm routines
#define DC_COUNT_RELOAD  140U   //10mS counting complete duty periods
#define DUTY_PERIOD 357U  //14kHz with 200nS ticks
#define KICK_COUNT 10U
#define KICKS 3U
	static uint16_t pwm_mask = 0;
	static uint16_t pwm_maskInverse = 0;
	static uint16_t dir_mask = 0;
	static uint16_t dir_maskInverse = 0;
	static uint8_t  dcCount = 0;   //counts complete cycles, will trigger 10mS tick
	static uint16_t  lowDutyPeriod = DUTY_PERIOD;
	static uint16_t  hiDutyPeriod = 0;
	static uint8_t  hiLowState = 0;
	static int8_t	kickCount = KICK_COUNT;
	
	
	//interrupt handler for DC pwm mode
	//static void ICACHE_RAM_ATTR pwm_intr_handler(void) //deprecated
		static void IRAM_ATTR pwm_intr_handler(void) {
		if (DCCpacket.trackPower) {
			
			//have we finished high or low?  
			if ((hiLowState ==0) && (hiDutyPeriod>0)) {
				//just finished a low, so load with high
				//EXCEPT where hiDuty is zero in which case we just execute low again
					WRITE_PERI_REG(&timer->frc1_load, hiDutyPeriod);
					gpio->out_w1ts = pwm_mask;  //set bits to logic 1
					gpio->out_w1tc = pwm_maskInverse;  //set bits to logic 0
					hiLowState = 1;
			}
			else {
				//just finished a hi, or hiDutyPeriod==0 so load with lo
				WRITE_PERI_REG(&timer->frc1_load, lowDutyPeriod);
				gpio->out_w1ts = pwm_maskInverse;  //set bits to logic 0
				gpio->out_w1tc = pwm_mask;  //set bits to logic 1
				hiLowState = 0;
				//only increment dcCount on low duty periods
				dcCount++;
			}
		}
		else {
			//track power off, load another full period of low
			lowDutyPeriod = DUTY_PERIOD;
			hiDutyPeriod = 0;
			WRITE_PERI_REG(&timer->frc1_load, DUTY_PERIOD);
			gpio->out_w1ts = pwm_maskInverse;  //set bits to logic 0
			gpio->out_w1tc = pwm_mask;  //set bits to logic 1
			dcCount++;
			hiLowState = 0;
		}

		//this will only be triggered during a low period when we have more time for processing
		if (dcCount >= DC_COUNT_RELOAD) {
			DCCpacket.msTickFlag = true;
			dcCount = 0;
			
			//copy the last packet sent, don't care about the preamble
			_TXbuffer.data[0] = DCCpacket.data[0];
			_TXbuffer.data[1] = DCCpacket.data[1];
			_TXbuffer.data[2] = DCCpacket.data[2];
			//don't bother with data[3,4,5] as we will only respond to short addr 3, and 1 possibly 2 (extended)
			//speed packets
			_TXbuffer.packetLen = DCCpacket.packetLen;
			//signal we are ready for another packet
			DCCpacket.clearToSend = true;

			//inspect the packet. we only care about loco 3, speed and dir
			//S 9.2 para 40
	
			
			if (_TXbuffer.data[0] == 3) {
				//instruction is for loco 3, short addr 
				if ((_TXbuffer.data[1] >> 6) == 0b01) {
					//this is a basic speed/dir command 01DCSSSS
					//C is the lsb of the speed code SSSS
					//per S 9.2 para 50
			//http://cpp.sh/9kmu6
					
					if ((_TXbuffer.data[1] & 0b1111) <= 1) {
						//0 or 1 indicate a stop condition C=don't care
						lowDutyPeriod = DUTY_PERIOD;
						hiDutyPeriod = 0;
					}
					else {
						//calculate the speed, essentially we have a 5 bit resolution.
						uint8_t j = (_TXbuffer.data[1] & 0b1111)<<1;
						j += (_TXbuffer.data[1] & 0b10000)>>4;  //add lsb 'C' bit
						//step 1 is integer 4
						j-=3;  //rebase at one
						//value will be between 1 and 28
						//saftey feature, cap j at 28
						j = j > 28 ? 28:j;
						

						//rebase logic.  basically no motor will move on less than 50% duty, so we should start 
						//at that point and the control is really exerted over 50-100% duty

						hiDutyPeriod = DUTY_PERIOD / 2;
						for (j = j;j > 0;j--) {
							hiDutyPeriod += DUTY_PERIOD / 56;
						}
						lowDutyPeriod = DUTY_PERIOD - hiDutyPeriod;
						//speed done.  speed 1 means hiDutyPeriod is slightly over 50% duty

					}
					

					//check direction 01DCSSSS
					if (_TXbuffer.data[1] & (1<<5)) {
						//forward
						gpio->out_w1ts = dir_mask;
						gpio->out_w1tc = dir_maskInverse;  
					}
					else {
						//reverse
						gpio->out_w1ts = dir_maskInverse;  
						gpio->out_w1tc = dir_mask;
					}

				}
				else if (_TXbuffer.data[1] == 0b111111) {
					//126 speed step instr follows
					//check direction
					if (_TXbuffer.data[2] & (1<<7)) {
						//forward
						gpio->out_w1ts = dir_mask;
						gpio->out_w1tc = dir_maskInverse;
					}
					else {
						//reverse
						gpio->out_w1ts = dir_maskInverse;  
						gpio->out_w1tc = dir_mask;  
					}
					//128 speed steps not implemented
					/*
					if (_TXbuffer.data[2] & 0b1111111 <=1) {
						//0 or 1 indicate a stop condition
						lowDutyPeriod = DUTY_PERIOD;
					}
					else {
						//calculate the speed as 7 bits rebased to 1
						uint8_t j = _TXbuffer.data[2] & 0b1111111;
						j--;  //rebase at one
							//value will be between 1 and 126
						hiDutyPeriod = 0;
						for (j = j;j > 0;j--) {
							hiDutyPeriod += DUTY_PERIOD / 126;
						}
						//DEBUG hiDuty must be 13 minimum
						if (hiDutyPeriod < 13) { hiDutyPeriod = 13; }

						lowDutyPeriod = DUTY_PERIOD - hiDutyPeriod;
						//speed done.  speed 1 means hiDutyPeriod is 3, so hopefully thats not too short
					}
					*/

				}
				

				//end packet inspection, all other packet types and all other locos are ignored
			}
			
			//kick logic. every KICK_COUNT x 10mS, put out KICKS x 10mS burst of 60% duty cycle
			if ((--kickCount <= 0) && (hiDutyPeriod>0)) {
				if (hiDutyPeriod < DUTY_PERIOD / 16) {
					hiDutyPeriod = DUTY_PERIOD / 16;
					lowDutyPeriod = DUTY_PERIOD - hiDutyPeriod;
					if (kickCount <= KICKS) kickCount = KICK_COUNT;
				}
			}
			
		}

	}



	//call with a pwm pin and direction pin
	void ICACHE_FLASH_ATTR dc_init(uint32_t pin_pwm, uint32_t pin_dir, bool phase, bool invert){
		//load with an IDLE packet
		DCCpacket.data[0] = 0xFF;
		DCCpacket.data[1] = 0;
		DCCpacket.data[2] = 0xFF;
		DCCpacket.packetLen = 3;

		pinMode(pin_pwm, OUTPUT);
		pinMode(pin_dir, OUTPUT);
		
		if (phase) {
			pwm_mask |= (1 << pin_pwm);
		}
		else {
			pwm_maskInverse |= (1 << pin_pwm);
		}

		if (invert) {
			dir_maskInverse |= (1 << pin_dir);
		}
		else {
			dir_mask |= (1 << pin_dir);
	}
		

#if PWM_USE_NMI
		ETS_FRC_TIMER1_NMI_INTR_ATTACH(pwm_intr_handler);
#else
		ETS_FRC_TIMER1_INTR_ATTACH(pwm_intr_handler, NULL);
#endif

		TM1_EDGE_INT_ENABLE();
		ETS_FRC1_INTR_ENABLE();
		RTC_REG_WRITE(FRC1_LOAD_ADDRESS, 0);  //This starts timer  ++++ RTC_REG_WRITE is deprecated ++++
		timer->frc1_ctrl = TIMER1_DIVIDE_BY_16 | TIMER1_ENABLE_TIMER;
}

	

