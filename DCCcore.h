

// DCCcore.h

#ifndef _DCCCORE_h
#define _DCCCORE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif




#include "Keypad.h"
#include "JogWheel.h"
#include <EEPROM.h>


/*version control and capture of some system defaults for new compilations*/
struct CONTROLLER
{
	long	softwareVersion = 20250812;  //yyyymmdd captured as an integer
	uint16_t	currentLimit = 1000;
	uint8_t	voltageLimit = 15;
	char SSID[21] = "DCC_ESP";
	char pwd[21] = "";
	char IP[17] = "192.168.6.1\0";   //note the actual setting requires comma separators
	char STA_SSID[21] = "YOUR_SSID";  //SSID when running as a station on an external WiFi network  YOUR_SSID
	char STA_pwd[21] = "";			//pwd for station
	uint16_t wsPort = 12080;        //websocket port
	uint16_t tcpPort = 12090;       //tcp port
	bool isDirty = false;  //will be true if EEPROM needs to be written
	bool flagLocoRoster;
	bool flagTurnoutRoster;
	bool bootAsAP = false;
};

//note for testing
//home page is http://192.168.6.1/index.htm
//websocket tests on ws://192.168.6.1:12080



/*current monitoring is performed as part of the power object
trip and trackpower are active flags, changing them will cause the unit to
disconnect power for example*/
struct POWER
{
	bool  trip;
	uint16_t current;
	bool  serviceMode;
	bool  trackPower;
	bool  report = true;  //set false for debug
	/*these are internal working registers*/
	int ADresult;   //native units
	/*these are results, populated either from INA or AD system*/
	float quiescent_mA = 0;
	float bus_volts = 0;
	float bus_mA = 0;
	/*these relate to acknowledgement pulses in service mode*/
	float ackBase_mA;  //set this to bus_mA before starting
	float bus_peak_mA = 0;
	bool ackFlag;  //used by Service Mode for acknowledgement
};


struct LOCO
{
	uint16_t    address = 0;
	char		name[9];
	float       speed = 0;  //a percentile value
	bool        forward = true;
	uint16_t    function = 0;
	uint8_t     speedStep; //percentile converted to the actual speed step instruction
	uint8_t     eStopTimer = 0;
	bool        use128 = false;  //use 128 speed steps
	bool        useLongAddress = false;
	int8_t      shunterMode = 0;   //2020-10-09 0=disabled 1,-1 are enabled and give direction
	uint8_t     nudge = 0;  //not a flag as such.  set a value to send n packets at full power
	bool        functionFlag = false;  //true if function value(s) change
	bool        debug;
	bool		changeFlag;
	bool		brake = false;  //apply brake (i.e. transmit half speedStep)
	bool		jog = false;// is the jogWheel currently controlling this loco?
	bool		directionFlag;  //indicates direction changed
	uint8_t		consistID;
	uint16_t	history;
	//clear a slot ready for use
	void clear(void) {
		speed = 0;
		speedStep = 0;
		consistID = 0;
		forward = true;
		jog = false;
		history = 0;
		use128 = true;
		function = 0;
		shunterMode = 0;
		memset(name, '\0', sizeof(name));
	};
};

struct TURNOUT
{
	uint16_t    address = 0;
	bool        thrown = false;
	uint8_t     history;
	bool        selected;   //is currently selected in the GUI
	char        name[9];
	bool		changeFlag;
};

/*states for Program on Main*/
enum POMstate {
	POM_BYTE,
	POM_BYTE_WRITE,
	POM_BIT,
	POM_BIT_WRITE,
	POM_BYTE_READ,
	POM_BIT_READ
};

struct POM {
	uint16_t address = 3;
	bool    useLongAddr;
	bool	useAccessoryAddr;  //added 2020-06-17
	bool	readSuccess =true;	//added 2024-12-02
	uint16_t cvReg = 29;
	uint8_t  cvData = 128;  
	uint8_t	 cvBit = 0; //<7> is the bit value, <0-2> the bit pos
	uint8_t digitPos;
	uint8_t	state = POM_BYTE;
	uint8_t timeout = 0;
	uint8_t packetCount;
};



/*states related to SERVICE MODE*/
enum cvSTATE
{
	CV_IDLE,
	PG_START,
	PG_PG_WRITE,
	PG_RESET,
	PG_WRITE,
	PG_RESET_2,
	D_START,
	D_WRITE,
	D_RESET,
	RD_START,
	RD_PREAMBLE,
	RD_VERIFY,
	RD_RESET,
	RD_FINAL
};

struct CV {
	int8_t digit;
	bool read;
	bool write;
	int cvReg = 1;
	int cvData;
	int8_t timeout;
	//bool ackFlag;
	int pgPage;
	int8_t pgReg;
	cvSTATE state = CV_IDLE;
	int8_t packetCount;
	int8_t bitCount;
};
/*cvReg and cvData need to be 16 bit ints due to temporary values they hold from key entry*/


/*Accessories are handled as 11-bit addresses with a boolean for thrown/through status
 *this is consumed in the packet engine */
struct ACCESSORY
{
	uint16_t    address = 0;
	bool        thrown;
};

/*dcc state engine*/
enum dccSTATE
{
	DCC_LOCO,
	DCC_FUNCTION,
	DCC_ACCESSORY,
	DCC_ESTOP,
	DCC_SERVICE,
	DCC_POM,
	DCC_IDLE
};





extern TURNOUT turnout[MAX_TURNOUT];
extern LOCO loco[MAX_LOCO];
extern POWER power;
extern CONTROLLER bootController;
extern ACCESSORY accessory;
extern dccSTATE dccSE;

extern bool  quarterSecFlag; //public flag for use in main loop

/*list of functions */

//core processing
void DCCcoreBoot();
int8_t DCCcore(void);
void dccPacketEngine(void);
void updateLocalMachine(void);
void dccGetSettings();
void replicateAcrossConsist(int8_t slot);
void dccPutSettings();
bool writePOMcommand(const char* address, uint16_t cv, const char* val);
bool writeServiceCommand(uint16_t cvReg, uint8_t cvVal, bool verify, bool enterSM, bool exitSM);
float getVolt();  //debug
void railcomCallback(uint8_t result, bool success);

//debug
void debugTurnoutArray(void);




/*static functions*/
//keypad related
static int8_t setTurnoutFromKey(KEYPAD& k);
static int8_t setFunctionFromKey(KEYPAD& k);
static void changeDigit(char digitASCII, uint8_t digPos, uint8_t* target);
static void changeDigit(char digitASCII, uint8_t digPos, uint16_t* target);
static bool setServiceModefromKey(void);

//display related

static void updateTurnoutDisplay(void);
static void updateServiceModeDisplay(void);
static void updateUNIdisplay();

//jogwheel related

static int8_t setLocoFromJog(nsJogWheel::JOGWHEEL& j);

//system or hardware

static void ina219Mode(boolean AverageMode);
static int8_t setLoco(LOCO* loc, int8_t speed, bool dir);
static LOCO* getNextLoco(LOCO* loc);
static void incrLocoHistory(LOCO* loc);

int8_t findLoco(char* address, char* slotAddress, bool searchOnly = false);
int8_t findTurnout(uint16_t turnoutAddress);



/*end list of functions */


#endif  //end guard
