
#include "OturmaOdasi.h"

#include <eEEPROM.h>
#include <NecIR.h>
#include <ir_cmds_44key_rgb_strip_controller.h>

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
// (warning: only initialized variables can be placed into program memory area)
#ifdef PROGMEM
  #ifdef OSX
	#ifndef PROGMEN
//	#define PROGMEM
	#endif
  #else
	#undef PROGMEM
	#define PROGMEM __attribute__((section(".progmem.data")))
#endif
#endif

#ifdef OSX
#include <math.h>
#endif

///////////////////////////////////////////////////////////////////////////////////
// definitions for compiled in features
#ifdef OSX
	// #define WITH_TIME_LAPSE
	//#define WITH_TIMEWARP
#endif
//#define WITH_HEARTBEAT // adds 90 bytes
//#define WITH_PING
//#define WITH_STARTTIMES
#define WITH_DUMPING // adds   1k code
#define WITH_HELP
#define WITH_INDICATE_STARTUP // adds 100 bytes code
#define WITH_SAVE_UPTIME
#define WITH_SERIAL
//#define WITH_STATS // adds 4k
#define WITH_SHOW_EVENTS
#define WITH_STRAND_UPDATES
#define WITH_IR
///////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////// <prototypes>
// Arduino environment does not require these, however compiler on the Mac
void save_cost_per_kWh100(uint16_t cost_per_kWh100);
static void update_uptime();
void save_uptime();
static void println_version();
static void flash_status_led(int8_t n);
void dump_starttime();
void setup();
void loop(void);
void inputClear();
static void set_status_leds(uint8_t status);
static void set_status_leds_analog(uint8_t value);
static void save_statistics();
//////////////////////////////////// </prototypes>

typedef enum { V_SILENT=0, V_NORMAL=1, V_VERBOSE=2, V_DEBUG=3 } e_verbosity;
const static e_verbosity V_DEFAULT = V_NORMAL;
static e_verbosity verbosity = V_DEFAULT;

// tsop on pin 2 (int0)
const static byte PIN_IR_RECVR    =  2;

// not connected but could be used in the future
const static byte PIN_STATUS_LED  =  3;

const static byte PIN_ONBOARD_LED = 13;
const static byte PIN_RESET       = 11;

// Data wire is plugged into pin 2 on the Arduino
const static byte PIN_PWM_R =  9;
const static byte PIN_PWM_G =  9;
const static byte PIN_PWM_B =  9;

// default pins that connect to the relays controlling the heating strands
static const uint8_t RELAY_PINS[] = { 4, 7, 8, 10 };

static boolean eeprom_reset   = false;
static boolean severe_error   = false;

#ifdef WITH_PING
static uint32_t millis_ping     = 0;
#endif

static uint32_t ts_optime       = 0; // total operation time in minutes
static uint32_t ts_starttime    = 0; // current optime when uC started
static uint32_t ts_last         = 0; // last timestanp seen in main loop()

#ifdef WITH_SERIAL
static boolean   input_complete = false;
static char      input_buffer[16]= { 0 };         // a string to hold incoming serial data
static char    * inputPos  = input_buffer;
static char    * inputEnd  = input_buffer+sizeof(input_buffer)-1;
#endif

static const s_eeprom_data * EE = 0;
eEE_CHECKSIZE(*EE); 

//const  uint16_t MAGIC = 0x2347;
// Magic number to write to the beginniing of EEProm to check whether it was initialized before.
// The value will change whenever n umber of sensors or strands was changed or when
// the structure s_eeprom_data has been changed significantly (thus the position ofstrand_infos changed).
const  uint16_t MAGIC = 4711; //(uint16_t)((MAX_SENSORS<<12) + (MAX_STRANDS<<8) + (long)&(EE->strand_infos));

static uint8_t D = 2; // debug

int8_t  presetToLoad = -1;

// timout to tell appart whether the DIY button was just pressed or held down
uint32_t presetLoadTime  = 0;
uint32_t lastChangeSaveTime = 0;


#ifndef OSX
PROGMEM
#endif
s_rgb  color_sequence[] =
{
  { r:255, g:0, b:0 },     { r:0, g:255, b:0 },     { r:0, g:0, b:255 },
  { r:255, g:255, b:0 },   { r:0, g:255, b:255 },   { r:255, g:0, b:255 },
  { r:255, g:255, b:128 }, { r:128, g:255, b:255 }, { r:255, g:128, b:255 },
  { r:255, g:255, b:255 }
};

NecIR necIR(PIN_IR_RECVR);

/*******************************************/
void sendCurrent();
/*******************************************/


static uint16_t curr_op_day()
{
	return ts_optime/(24*60);
}

static float read_cost_per_kWh()
{
	return 0.27f;
}

void save_cost_per_kWh100(uint16_t cost_per_kWh100)
{
}

s_preset preset =
{
	mode       :   0,
        mode_bits  : 255,
        randomTreshold : 0,
	offsetR    : 255,
	offsetG    : 255,
	offsetB    : 255,
	brightness : 255,
        delayTime  : 0,
        fadePrescale : 1,      
};


uint8_t brightness_apply(uint8_t value, uint8_t brightness)
{
	uint16_t i = value;
	return constrain((i<<8)/brightness, 0, 255);
}

void mode_apply()  
{
  digitalWrite(RELAY_PINS[0], preset.mode_bits & BIT_DOWN);
  digitalWrite(RELAY_PINS[1], preset.mode_bits & BIT_UP1);
  digitalWrite(RELAY_PINS[2], preset.mode_bits & BIT_UP2);
  digitalWrite(RELAY_PINS[3], preset.mode_bits & BIT_STRIP1);
  
  if (preset.mode_bits & BIT_STRIP2)
  {
    analogWrite(PIN_PWM_R, brightness_apply(preset.offsetR, preset.brightness));
    analogWrite(PIN_PWM_G, brightness_apply(preset.offsetG, preset.brightness));
    analogWrite(PIN_PWM_B, brightness_apply(preset.offsetB, preset.brightness));
  }
  else
  {
    digitalWrite(PIN_PWM_R, 0);
    digitalWrite(PIN_PWM_G, 0);
    digitalWrite(PIN_PWM_B, 0);
  }
  sendCurrent();
}

void mode_next()
{
  preset.mode = (preset.mode+1)%sizeof(mode_sequence);
  preset.mode_bits  = mode_sequence[preset.mode];
  mode_apply();
}
  
static void eeprom_init()
{
	//const boolean debug = false;
	//if (debug) PPRINTLN("D.eeprom_init");

	uint16_t magic = eEE_READN(EE->magic);
	//BZERO(strand_infos);

	PPRINT("E.MAGIC."); Serial.print(magic, HEX);
	if (MAGIC!=magic)
	{
		PPRINTLN(".inv");

		magic = MAGIC;
		update_uptime(); // this will set: ts_optime = approx. 0;
		ts_last = 0;

		save_uptime();
		eEE_WRITEN(EE->magic, magic);
		eeprom_reset = true;
	}
	else
	{
		PPRINTLN(".val");

		ts_optime = eEE_READN(EE->optime_minutes);
		ts_last = ts_optime;

		eEE_READ(EE->presets[0], preset);
	}
}


void reboot()
{
	#ifdef OSX
		doRestart = true;
	#else
        digitalWrite(PIN_RESET, LOW);
        asm volatile ("jmp 0");
	#endif
}

static void reset()
{
    // explicit cast necessary to uint16_t to make the templating work
	uint16_t wrong_magic = ~MAGIC;
	eEE_WRITEN(EE->magic, wrong_magic);
	reboot();
}



#ifdef WITH_STATS
void dump_stats()
{
//	dump_stats_hours();
//	dump_stats_days();
//	dump_stats_weeks();
	PPRINTLN("ST!");
}
static void save_statistics()
{
}
#else
static void save_statistics() {}
void dump_stats() { PPRINTLN("D. stats=n.a."); }
#endif


#ifdef WITH_STATS
void dump_current_stats()
{
}
#endif


#ifdef WITH_SAVE_UPTIME
void save_uptime()
{
//	PPRINT("save_uptime");
	eEE_WRITE(ts_optime, EE->optime_minutes);
}
#else
void save_uptime() {}
#endif


#ifdef WITH_HELP
static void dump_help()
{
	// from WaterStrip project:
	PPRINTLN(
			"H:H|?|G - help,features,get curr\n"
			"H:K=ir_key\n"
			"H:P=power (lamp 0)\n"
			"H:P=lp    (l=lamp>0,p=power)\n"
			"H:M=mode\n"
			"H:C=rrggbb\n"
			"H:B=brightn\n"
			//"H:A=ampl\n"
			//"H:S=speed\n"
			//"H:F=fade\n"
			//"H:R=random\n"
			//"H:T=strength\n"
			"H:L=preset\n"  // load
			"H:W=preset\n"  // write
			//"H:E=1|0 est/end conn\n"
			//"H:P3=90.0 set pwr=90W\n"
			//"H:C0.2 cost/kWh to 0.2\n"
			"H:V=1 set verbosity 0-2\n"
			"H:K=xys ackn\n"
			"H:B=4711 reboot uC\n"
			"H:Z=4711 zap+reset uC\n"
			"H!"
	);
#endif
}

/*
   BIT_UP1 | BIT_UP2,
   BIT_UP1 | BIT_UP2 | BIT_DOWN,
   BIT_UP2 | BIT_DOWN,
   BIT_DOWN,
   BIT_STRIP1,
   BIT_STRIP1 | BIT_STRIP2,
   BIT_STRIP2,
   BIT_FAKETV
 */
void sendModeName(int mode)
{
	switch (mode)
	{
	case 0  : PPRINT("UP1+2");      break;
	case 1  : PPRINT("UP1+2/DOWN"); break;
	case 2  : PPRINT("UP1/DOWN");   break;
	case 3  : PPRINT("DOWN");       break;
	case 4  : PPRINT("STRIP1");     break;
	case 5  : PPRINT("STRIP1+2");   break;
	case 6  : PPRINT("STRIP2");     break;
	case 7  : PPRINT("FAKETV");     break;
	default : PPRINT("INVAL");      break;
	}
}


void sendHello()
{
  // Version, Kind, Power, Bright, Amplitude, Speed, Fade, Brightness, Random, sTrength, Color
  Serial.print("H:V=1 K=OTURMA");
  //Serial.print(" P=0-2 B=0-FF A=0-FF S=0-FFF F=0-FFF R=0-FF T=0-FFF C=0-FFFFFF ");
  // no amplitude, strength
  Serial.print(" LAMPS=5 P=0-2 B=0-FF C=0-FFFFFF ");

  // supported mode numbers and names
  Serial.print("M=");
  for (int i=MODE_MIN; i<=MODE_MAX; i++)
  {
     Serial.print(i,16); Serial.print("=");
     sendModeName(i);
     Serial.print("\t");
  }
  Serial.println();
}


static char to_upper(char c)
{
	if ('a'<=c && c<='z') c='A'+(c-'a');
	return c;
}

void dump_general_info()
{
#ifdef WITH_DUMPING
	uint32_t uptime = ts_optime-ts_starttime;
	  PPRINT("I:");           println_version();
	  PPRINT("I:OpMins=");    Serial.println(ts_optime);
	  PPRINT("I:OpDays=");    Serial.println(curr_op_day());
	  PPRINT("I:StrtMin=");   Serial.print(ts_starttime);    //PPRINTLN("m");
	PPRINT("\nI:UpMins=");    Serial.print(uptime);          //PPRINTLN("m");
	PPRINT("\nI:MxStrtTms="); Serial.print(MAX_START_HOURS); //PPRINTLN("m");
	PPRINT("\nI:Cost=");      Serial.println((int)(100*read_cost_per_kWh()));
	PPRINT("I:Verb=");        Serial.println(verbosity);
	PPRINTLN("I!");
#endif
}


#define MILLIS_PER_UPTIME (60*1000UL)

uint32_t mins_last = 0;

static void update_uptime()
{
	uint32_t mins = millis()/MILLIS_PER_UPTIME;
	//printf("update_uptime: mins: %i, mins_last: %i\n", mins, mins_last);
	// ignore millis overrun, upon next invocation things will be right
	if (mins_last<mins)
	{
		ts_optime += mins-mins_last;
		//printf("ts_optime:=%i\n", ts_optime);
	}
	mins_last=mins;
	return;
}


// indicate that a connection was established or it ends
void indicate_conn(boolean connected)
{
  flash_status_led(connected ? 2 : 3);
}


#ifdef WITH_IR

uint8_t limit(int i)
{
  return constrain(i,0,255);
}

void brightnessIncrease()
{
	preset.brightness = limit(((int)preset.brightness)+20);
	preset.mode_bits |= BIT_STRIP2;
	mode_apply();
}

void brightnessDecrease()
{
	preset.brightness = limit(((int)preset.brightness)-20);
	preset.mode_bits |= BIT_STRIP2;
	mode_apply();
}

void rgbIncrease(uint32_t i)
{
	preset.offsetR = limit(((int)preset.offsetR) + ((0xff0000 & i)>>16) );
	preset.offsetG = limit(((int)preset.offsetG) + ((0x00ff00 & i)>> 8));
	preset.offsetB = limit(((int)preset.offsetB) + ((0x0000ff & i)    ));
}

void rgbDecrease(uint32_t i)
{
	preset.offsetR = limit(((int)preset.offsetR) - ((0xff0000 & i)>>16));
	preset.offsetG = limit(((int)preset.offsetG) - ((0x00ff00 & i)>> 8));
	preset.offsetB = limit(((int)preset.offsetB) - ((0x0000ff & i)    ));
	preset.mode_bits |= BIT_STRIP2;
	mode_apply();
}

void setRGBBase(uint32_t rgb)
{
	preset.offsetR = (rgb & 0xff0000)>>16;
	preset.offsetG = (rgb & 0x00ff00)>>8;
	preset.offsetB = (rgb & 0x0000ff);
	preset.mode_bits |= BIT_STRIP2;
	mode_apply();
}

static void check_ir()
{
  uint16_t rep;
  uint8_t  cmd;
  
  cmd = necIR.get_command(&rep);
  if (0==cmd) return;
  
  switch (cmd)
  {    
        case CMD_POWER    : mode_next();
/*    
        case CMD_PAUSE    : break;
          
        case CMD_AUTO     : modeToggle();                   break;
        case CMD_LIGHTER  : brightnessIncrease();           break;
        case CMD_DARKER   : brightnessDecrease();           break;
        case CMD_QUICK    : parameterIncrease();            break;
        case CMD_SLOW     : parameterDecrease();            break;

        case CMD_JUMP3    : break;
        case CMD_JUMP7    : parameterSelect(PARAM_VISU_AMPLITUDE);          break;
        case CMD_FADE3    : parameterSelect(PARAM_DIST_STRENGTH);  break;
        case CMD_FADE7    : parameterSelect(PARAM_FADE_SPEED);      break;
        case CMD_FLASH    : parameterSelect(PARAM_SPEED);           break;
*/
        case CMD_B_UP     : rgbIncrease(0x00000f); break;
        case CMD_B_DOWN   : rgbDecrease(0x00000f); break;
        case CMD_G_UP     : rgbIncrease(0x000f00); break;
        case CMD_G_DOWN   : rgbDecrease(0x000f00); break;
        case CMD_R_UP     : rgbIncrease(0x0f0000); break;
        case CMD_R_DOWN   : rgbDecrease(0x0f0000); break;
/*
        case CMD_DIY1     : handlePreset(1, repeat); break;
        case CMD_DIY2     : handlePreset(2, repeat); break;
        case CMD_DIY3     : handlePreset(3, repeat); break;
        case CMD_DIY4     : handlePreset(4, repeat); break;
        case CMD_DIY5     : handlePreset(5, repeat); break;
        case CMD_DIY6     : handlePreset(6, repeat); break;
*/
        case CMD_R1       : setRGBBase(0xff0000); break;
        case CMD_G1       : setRGBBase(0x00ff00); break;
        case CMD_B1       : setRGBBase(0x0000ff); break;
        case CMD_WHITE    : setRGBBase(0xff96a5); break; // calibrated

        case CMD_R2       : setRGBBase(0xff2000); break;
        case CMD_G2       : setRGBBase(0x00ff40); break;
        case CMD_B2       : setRGBBase(0x4000ff); break;
        case CMD_PINKISH1 : setRGBBase(0xff7070); break;

        case CMD_R3       : setRGBBase(0xff3000); break;
        case CMD_G3       : setRGBBase(0x00ff80); break;
        case CMD_B3       : setRGBBase(0x8000ff); break;
        case CMD_PINKISH2 : setRGBBase(0xff5050); break;

        case CMD_R4       : setRGBBase(0xff4000); break;
        case CMD_G4       : setRGBBase(0x00ffc0); break;
        case CMD_B4       : setRGBBase(0xc000ff); break;
        case CMD_BLUISH1  : setRGBBase(0xf0f0ff); break;

        case CMD_R5       : setRGBBase(0xff6000); break;
        case CMD_G5       : setRGBBase(0x00ffff); break;
        case CMD_B5       : setRGBBase(0xff00ff); break;
        case CMD_BLUISH2  : setRGBBase(0xccccff); break;
  }
}
#endif


void setPower(uint8_t lamp, uint8_t value)
{
	uint8_t bit = (value<<lamp);
	preset.mode_bits ^= bit;
}

void presetLoad(int n, struct s_preset & preset)
{
	eEE_READ(EE->presets[n], preset);
}


boolean presetSave(int n)
{
	eEE_WRITE(preset, EE->presets[n]);
    return true;
}

void indicateFeedbackCmd()
{
}

void handlePreset(int i, int repeat)
{
	if (repeat<8)
	{
		// if (D) { Serial.print("D:PRE_SCHED("); Serial.print(i); Serial.println(")"); }
		presetToLoad = (byte) i;
		presetLoadTime = millis() + 100;
	}
	else if (8==repeat)
	{
		presetSave(i);
		indicateFeedbackCmd();
	}
}

void presetCheckLoad()
{
	if (presetToLoad>-1 && presetLoadTime>0 && millis()>presetLoadTime)
	{
	    if (D>0) { Serial.print("D:PRE_LD("); Serial.print(presetToLoad); Serial.println(")"); }
	    presetLoad(presetToLoad, preset);
	    mode_apply();
	}
}

void changeCheckSave()
{
	if (lastChangeSaveTime>0 && millis()>lastChangeSaveTime)
	{
		if (D>0) { Serial.println("D:CHG_SAV"); }
		if (presetSave(0))
		{
			// flash RGB strip to indicate things have been saved
//			delay(10);
//			stripClear(255,255,255);
//			delay(10);
		}
		lastChangeSaveTime = 0;
	}
}


inline void scheduleSavingOfChanges()
{
  if (D>0) { Serial.println("D:CHG_SCHED"); }
  // save changes after 5s of user inactivity
  lastChangeSaveTime = millis()+5000;
}

void modeToggle()
{
}


void parameterIncrease()
{
}

void parameterDecrease()
{
}

uint32_t rgbToUint32(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r)<<16) | (((uint32_t)g)<<8) | ((uint32_t)b);
}

void sendCurrent()
{
	PPRINT("G:");
	PPRINT("M="); Serial.print(preset.mode, 10); PPRINT(",");
	PPRINT(" BITS="); Serial.print(preset.mode_bits, 2); PPRINT(",");
	uint32_t rgb = rgbToUint32(preset.offsetR, preset.offsetG, preset.offsetB);
	PPRINT(" C="); Serial.print(rgb, 16);
	PPRINT(" B="); Serial.println(preset.brightness, 16);
	return;
	sendModeName(preset.mode);

	//PPRINT(" S="); Serial.print(preset.delayTime,       16);
	//PPRINT(" F="); Serial.print(preset.fadePrescale,    16);
	PPRINT(" B="); Serial.print(preset.brightness,      16);
	//PPRINT(" A="); Serial.print(preset.amplitude,       16);
	//PPRINT(" R="); Serial.print(preset.randomTreshold,  16);
	//PPRINT(" T="); Serial.print(preset.disturbStrength, 16);
	PPRINT("");

}

void toggle_pause()
{
}

/**
 * Callback function for IR commands.
 */
typedef boolean (*t_nec_ir_func)(int code, int repeat, void * user_data);


/**
 * A structure for registering function pointers for specific IR command codes.
 */
typedef struct s_nec_ir_vector
{
	uint8_t         code;
	t_nec_ir_func   func;
	void          * user_data;
} s_ir_vector;


boolean ir_power(int code, int repeat, void * user_data)
{
	if (5==(repeat%6)) mode_next();
	return true;
}


boolean ir_pause(int code, int repeat, void * user_data)
{
	if (repeat<6) toggle_pause();
	return true;
}


boolean ir_auto(int code, int repeat, void * user_data)
{
	//modeToggle();
	return true;
}

boolean ir_lighter_darker(int code, int repeat, void * user_data)
{
	if (user_data)
		brightnessIncrease();
	else
		brightnessDecrease();
	return true;
}

boolean ir_quick_slow(int code, int repeat, void * user_data)
{
	if (user_data)
		parameterIncrease();
	else
		parameterDecrease();
	return true;
}


boolean ir_up(int code, int repeat, void * user_data)
{
	int i = (uint64_t)user_data;
	rgbIncrease(i);
	return true;
}

boolean ir_down(int code, int repeat, void * user_data)
{
	int i = (uint64_t)user_data;
	rgbDecrease(i);
	return true;
}

boolean ir_diy(int code, int repeat, void * user_data)
{
	int i = (uint64_t)user_data;
	handlePreset(i, repeat);
	return true;
}

boolean ir_set(int code, int repeat, void * user_data)
{
	int rgb = (uint64_t)user_data;
	setRGBBase(rgb);
	return true;
}

boolean ir_jump(int code, int repeat, void * user_data)
{
	int i = (uint64_t)user_data;
	if (3==i)
	{
		//parameterSelect(PARAM_RANDOM_TRESHOLD);
		if (repeat>10)
		{
			//resetRipples();
			//preset.randomTreshold = 0;
		}
	}
	else
	{
		//parameterSelect(PARAM_VISU_AMPLITUDE);
	}
	return true;
}

boolean ir_fade(int code, int repeat, void * user_data)
{
	int i = (uint64_t)user_data;
	if (3==i)
	{
		//parameterSelect(PARAM_DIST_STRENGTH);
	}
	else
	{
		//parameterSelect(PARAM_FADE_SPEED);
	}
	return true;
}

boolean ir_flash(int code, int repeat, void * user_data)
{
	return true;
}


boolean ir_unknown(int code, int repeat, void * user_data)
{
	PPRINT("E:KEY:"); Serial.println(code, 16);
	return true;
}


const s_ir_vector IR_VECTORS[] =
{
	{ CMD_POWER    , ir_power,          (void*)0 },
	{ CMD_PAUSE    , ir_pause,          (void*)0 },
	{ CMD_AUTO     , ir_auto,           (void*)0 },
	{ CMD_LIGHTER  , ir_lighter_darker, (void*)1 },
	{ CMD_DARKER   , ir_lighter_darker, (void*)0 },
	{ CMD_QUICK    , ir_quick_slow,     (void*)1 },
	{ CMD_SLOW     , ir_quick_slow,     (void*)0 },
	{ CMD_B_UP     , ir_up,      (void*)0x00000f },
	{ CMD_G_UP     , ir_up,      (void*)0x000f00 },
	{ CMD_R_UP     , ir_up,      (void*)0x0f0000 },
	{ CMD_B_DOWN   , ir_down,    (void*)0x00000f },
	{ CMD_G_DOWN   , ir_down,    (void*)0x000f00 },
	{ CMD_R_DOWN   , ir_down,    (void*)0x0f0000 },
	{ CMD_DIY1     , ir_diy,            (void*)1 },
	{ CMD_DIY2     , ir_diy,            (void*)2 },
	{ CMD_DIY3     , ir_diy,            (void*)3 },
	{ CMD_DIY4     , ir_diy,            (void*)4 },
	{ CMD_DIY5     , ir_diy,            (void*)5 },
	{ CMD_DIY1     , ir_diy,            (void*)6 },

	{ CMD_R1       , ir_set,     (void*)0xff0000 },
    { CMD_G1       , ir_set,     (void*)0x00ff00 },
    { CMD_B1       , ir_set,     (void*)0x0000ff },
    { CMD_WHITE    , ir_set,     (void*)0xff96a5 },
    { CMD_R2       , ir_set,     (void*)0xff2000 },
    { CMD_G2       , ir_set,     (void*)0x00ff40 },
    { CMD_B2       , ir_set,     (void*)0x4000ff },
    { CMD_PINKISH1 , ir_set,     (void*)0xff7070 },
    { CMD_R3       , ir_set,     (void*)0xff3000 },
    { CMD_G3       , ir_set,     (void*)0x00ff80 },
    { CMD_B3       , ir_set,     (void*)0x8000ff },
    { CMD_PINKISH2 , ir_set,     (void*)0xff5050 },
    { CMD_R4       , ir_set,     (void*)0xff4000 },
    { CMD_G4       , ir_set,     (void*)0x00ffc0 },
    { CMD_B4       , ir_set,     (void*)0xc000ff },
    { CMD_BLUISH1  , ir_set,     (void*)0xf0f0ff },
    { CMD_R5       , ir_set,     (void*)0xff6000 },
    { CMD_G5       , ir_set,     (void*)0x00ffff },
    { CMD_B5       , ir_set,     (void*)0xff00ff },
    { CMD_BLUISH2  , ir_set,     (void*)0xccccff },

    { CMD_JUMP3    , ir_jump,           (void*)3 },
    { CMD_JUMP7    , ir_jump,           (void*)7 },
    { CMD_JUMP3    , ir_fade,           (void*)3 },
    { CMD_JUMP7    , ir_fade,           (void*)7 },
    { CMD_FLASH    , ir_flash,          (void*)0 },
    { 0            , ir_unknown,        (void*)0 }
};


void handleIRCommand(int code, int repeat)
{
	boolean done = (code==0);
	for (const s_ir_vector * vector=IR_VECTORS; !done; vector++)
	{
		if (0==vector->code)
		{
			if (vector->func) vector->func(code, repeat, vector->user_data);
			return;
		}
		else if (vector->code==code)
		{
			PPRINT("O:KEY:"); Serial.println(code, 16);
			done = vector->func(code, repeat, vector->user_data);
		}
	}

	scheduleSavingOfChanges();
	sendCurrent();
}

/*
void ackCmd(const char * cmd, long arg)
{
  PPRINT("O:"); Serial.print(cmd); PPRINT(":");
  Serial.print(arg,16); PPRINT(","); Serial.println(arg);
}
*/


void ackCmd(const char * cmd, long arg)
{
  Serial.print("O:"); Serial.print(cmd); Serial.print(":"); 
  Serial.print(arg,16); Serial.print(','); Serial.println(arg);
}


#ifdef WITH_SERIAL

// from WaterStrip project (stay compatible with this)
void check_serial()
{
	if (!input_complete)
	{
		return;
	}

	if (D>1) { PPRINT("D:LINE:'"); Serial.print(input_buffer); PPRINTLN("'"); }

	char cmd   = to_upper(input_buffer[0]);
	char delim = input_buffer[1];
	long arg   = 0;

	if (('='==input_buffer[1] || ':'==input_buffer[1]) && 0!=input_buffer[2])
	{
		char * c = input_buffer+2;

		input_buffer[1]=0;
		sscanf(c, "%lx", &arg);

		switch (cmd)
		{
		case 'P':
			if (arg>=10)
			{
				int lamp  = arg/10; // which lamp: 1,...,5
				int value = arg%10; // 0-off, 1-on 2-toggle
				setPower(lamp, value);
			}
			else
			{
				mode_next();
			}
			break;
		case 'K':
			handleIRCommand(arg, 0);
			break;
		case 'M':
			preset.mode = constrain(arg, MODE_MIN, MODE_MAX);
			preset.mode_bits  = mode_sequence[preset.mode];
			mode_apply();
			ackCmd("M", preset.mode);
			break;
		case 'C':
			arg &= 0xffffff;
			preset.offsetR = (arg & 0xff0000)>>16;
			preset.offsetG = (arg & 0x00ff00)>> 8;
			preset.offsetB = (arg & 0x0000ff);
			mode_apply();
			ackCmd("C", arg);
			break;
		case 'B':
			preset.brightness = constrain(arg, 0, 255);
			mode_apply();
			ackCmd("B", preset.brightness);
			break;
		case 'A':
			//preset.amplitude = constrain(arg, 0, 255);
			//ackCmd("A", preset.amplitude);
			break;
		case 'S':
			preset.delayTime = constrain(arg, 0, 1000);
			ackCmd("S", preset.delayTime);
			break;
		case 'F':
			preset.fadePrescale = constrain(arg, 0, 2000);
			ackCmd("F", preset.fadePrescale);
			break;
		case 'R':
			preset.randomTreshold = constrain(arg, 0, 100);
			ackCmd("R", preset.randomTreshold);
			break;
		case 'T':
			//preset.disturbStrength = constrain(arg, 0, 400);
			//ackCmd("S", preset.disturbStrength);
			break;
		case 'D':
			D = constrain(arg, 0, 2);
			ackCmd("D", D);
			break;
		case 'L':
			presetLoad(arg, preset);
			//preset.power = 1;
			ackCmd("L", arg);
			break;
		case 'W':
			presetSave(arg);
			ackCmd("W", arg);
			break;
		case 'Y':
		case 'y':
		{
//			int pin, value;
//			sscanf(c, "%i,%i", &pin, &value);
//			long x = togglePin(pin, value);
//			ackCmd("Y", x);
			break;
		}
		}

		scheduleSavingOfChanges();
	}
	else if (!strcasecmp("F", input_buffer))
	{
		//Serial.print("F:"); Serial.println(frameCount,16);
	}
	else if (!strcasecmp("H", input_buffer))
	{
		//dump_help();
		sendHello();
	}
#ifdef WITH_HELP
	else if (!strcmp("?", input_buffer))
	{
		dump_help();
	}
#endif
	else if (!strcasecmp("G", input_buffer))
	{
		sendCurrent();
	}
	else
	{
		Serial.print("E:INVAL:'"); Serial.print(input_buffer); Serial.println("'");
	}

	inputClear();
}

/* from Frotect project
static void check_serial()
{
	if (input_complete)
	{
		//if (verbosity>=V_VERBOSE) { PPRINT("CMD. "); Serial.println((const char*)input_buffer); }
		#ifdef OSX
		fprintf(stderr, "OSX: CMD.'%s'\n", input_buffer);
		#endif
		
		boolean valid = true;

		char first = to_upper(input_buffer[0]);
		char c = input_buffer[1];
		uint8_t  n = c-'0'; // '1' -> 1

		if ('E'==first)
		{
			indicate_conn('1'==c);
		}
		else if ('C'==first)
		{
			char * str = &input_buffer[1];
			uint16_t cost100 = atoi(str);
			PPRINT("SET.cost="); Serial.println(cost100, DEC);
			save_cost_per_kWh100(cost100);
		}

//		else if ('M'==first)
//		{
//			if ((valid = (1<=n && n<=MAX_SENSORS)))
//			{
//				PPRINT("SET.nstr="); Serial.println(n);
//				save_number_of_strands(n);
//			}
//		}
//		else if ('N'==first)
//		{
//			if ((valid = (1<=n && n<=MAX_SENSORS)))
//			{
//				PPRINT("SET.nsens="); Serial.println(n);
//				save_number_of_sensors_expected(n);
//			}
//		}

		else if ('V'==first)
		{
			// commented out first part of condition to avoid compiler
			//  warning: comparison is always true due to limited range of data type
			valid =  (n<=V_DEBUG); // avoid
			if (valid)
			{
				verbosity = (e_verbosity)n;
			}
		}
#ifdef WITH_STATS
		else if ('D'==first)
		{
			char sec = to_upper(input_buffer[1]);
			if ('T'==sec)
			{
				dump_starttime();
			}

//			else if ('H'==sec)
//			{
//				dump_stats_hours();
//			}
//			else if ('D'==sec)
//			{
//				dump_stats_days();
//			}
//			else if ('W'==sec)
//			{
//				dump_stats_weeks();
//			}
//			else if ('S'==sec)
//			{
//				// show complete information, not only updated strand and include ramping info as well
//				dump_strand_infos(false, true);
//			}

			else if ('G'==sec)
			{
				dump_general_info();
			}
			else if (0==sec)
			{
				dump_general_info();
				dump_starttime();
				dump_stats();
				dump_current_stats();
				#ifdef WITH_MIN_MAX
				dump_minmax();
				#endif
			}
			else
			{
				valid = false;
			}
		}
#endif
#ifdef WITH_REAL_SENSORS
		else if ('S'==first)
		{
			dump_sensors();
		}
#endif
		else if ('K'==first)
		{
			PPRINT("ACK."); Serial.println(input_buffer+1);
		}
#ifdef WITH_TESTMODE
		else if ('T'==first)
		{
			test_mode();
		}
#endif
#ifdef WITH_AUTOCONFIG
		else if ('A'==first)
		{
			auto_config();
		}
#endif
		else if ('?'==first || 'H'==first)
		{
			dump_help();
		}
		else if (!strncmp(input_buffer, "B=4711", 6))
		{
			reboot();
		}
		else if (!strncmp(input_buffer, "Z=4711", 6))
		{
			reset();
		}
		else if (0==first)
		{
			// empty
		}
		else
		{
			valid = false;
		}

		if (!valid)
		{
			PPRINT("E.CMD["); Serial.print(input_buffer); PPRINTLN("]");
		}
		else
		{
			if (verbosity>=V_VERBOSE) { PPRINT("CMD!"); Serial.println(input_buffer);  }
		}

		inputClear();
	}
}
*/

void inputClear()
{
   // clear the string:
   *(inputPos=input_buffer) = 0;
   input_complete = false;
   //fprintf(stderr, "inputClear: cleared: '%s'\n", input_buffer);
}


void serialEvent()
{
  //PPRINT("serialEvent\n");
	int a=0;
  if (!Serial.available())
  {
    return;
  }

  while (1)
  {
	a=1;
	//fprintf(stderr, "serialEvent: serial available\n");
	char c = (char)Serial.read();
	//fprintf(stderr, "serialEvent: read: %i (%c)\n", c, c);

         if (0x1b==c)
         {
             reboot();
         }
         
	if (c=='\n' || c=='\r')
	{
	  *inputPos = 0; // strip CR at end for convenience
	  input_complete = true;
	  //fprintf(stderr, "complete\n");
	  return;
	}
	else if (inputPos<inputEnd)
	{
	  *(inputPos++) = c;
	  if (inputPos<inputEnd)
	  {
		  //fprintf(stderr, "serialEvent: not full: '%s'\n", input_buffer);
		  *inputPos = 0;
	  }
	}
  }
  //if (a) fprintf(stderr, "serialEvent: leaving\n");
}
#else
void check_serial() {}
#endif

/*
static void print_lit(uint8_t n, boolean lit)
{
	PPRINT("F:n="); Serial.print(n+1); PPRINT(",lit="); Serial.println(lit);
} 
*/

static void print_hb(uint8_t status, uint8_t cond, int16_t temp) //, byte err, byte cmpl, byte none)
{
	if (verbosity>V_SILENT)
	{
		PPRINT("HB.l=");  Serial.print(status);
		PPRINT(",c=");    Serial.print(cond);
		PPRINT(",t=");    Serial.println(temp);
	}
}

static void flash_status_led(int8_t n)
{
	for (uint8_t i=0; i<n; i++)
	{
		set_status_leds(HIGH);
		delay(50);

		set_status_leds(LOW);
		delay(100);
	}
}

#ifdef WITH_INDICATE_STARTUP
static void indicate_startup()
{
	#ifdef OSX 
	#define multiply 1
	#else
	#define multiply 1
	#endif

	PPRINTLN("F!");
}
#else
static void indicate_startup()
{
	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		int pin = strand_infos[s].pin;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
}
#endif




#ifdef WITH_HEARTBEAT
static struct
{
	unsigned long last_millis;
	unsigned long last_warn;
	unsigned long sequence_start; // ms
	unsigned long sequence_end; // ms
	unsigned int  light_up_len;
	unsigned int  dim_down_len;
	byte          last_status;
	int16_t       last_temp;
} hb =
{
		last_millis:     -1,
        last_warn:        0,
		sequence_start:   0,
		sequence_end:   100,
		light_up_len:    10,
		dim_down_len:    20,
		last_status:      0,
		last_temp:        0
};
#endif

void show_status(boolean working)
{
#ifdef WITH_HEARTBEAT
	unsigned long now = millis();

	if (now==hb.last_millis)
	{
		return;
	}
	hb.last_millis = now;

	byte new_status = hb.last_status;
	byte cond = 0;
	byte complete = true; //sensors_complete();
	byte error    = false; // severe_error;
	byte none     = false; // sensors_none();

	if (complete && !error)
	{
		cond = 1;
		int value = 0;
//		printf("-----\n");
//		printf("D.hb.now: %li\n", now);
//		printf("D.hb.sequence_end: %li\n", hb.sequence_end);

		if (now>hb.sequence_end)
		{
			// compute new value for whole range when sequence exceeded
			hb.last_temp = 100*30; // get_hb_temp100()/100;
			int16_t temp = constrain(hb.last_temp, -30, 30);

			// for temp=-30C,...,30C yield f=60,...,0
			hb.sequence_start = now;

			// len: delay value: for -30C,...,30C yield 300+120=420,...,300-120=80 (beat faster when hotter)
			int16_t len = 300-4*temp;
			hb.light_up_len = 2*len;
			hb.dim_down_len = 4*len;

			// from temp=-30,...,30 yield 60,...,0 (shorter sequence when hotter)
			int16_t f = (30-temp);
			//hb.sequence_end = now+4*len+2.5*f*f;
			hb.sequence_end = now+4*len+(5*f*f)/2;
		}
		else
		{
 			//printf("D.hb.sequence_start: %li\n", hb.sequence_start);
			long delta = now-hb.sequence_start;
 			//printf("D.delta: %li\n", delta);
			if (delta<hb.light_up_len)
			{
                // heartbeat idicator starting to face => set status to 1 (inverse logic)
				value = constrain(250*delta/hb.light_up_len, 0, 250);
  			    new_status = value>180;
			}
			else if (delta<(hb.light_up_len+hb.dim_down_len))
			{
				value = 250-constrain(250*(delta-hb.light_up_len)/hb.dim_down_len, 0, 250);
				new_status = value>180;
			}
		}

		set_status_leds_analog(5+value);
	}
	else if (none || error)
	{
		cond = 3;
		new_status = (now%200)<100 ? LOW : HIGH;
		set_status_leds(new_status);
	}
	else // some sensors missing but no severe error
	{
		cond = 2;
		unsigned long seq = now%4000; // 0,...,6999

		unsigned int missing = 0; //num_sensors-num_sensors_available;
		int value = 255;
		if (seq<511*missing)
		{
			new_status = 1;
			// will yield "missing" times a sequence 0,...,255
			//analogWrite(PIN_STATUS_LED, (seq%512)/2);
			int value = (seq%511); // 0,..,510
			value = value-255;  // -255,...,255
			value = abs(value); // 255,...,0,...255
		}
		set_status_leds_analog(value);
	}

	if (new_status != hb.last_status)
	{
		print_hb(new_status, cond, hb.last_temp); //, error, complete, none);
		hb.last_status = new_status;
	}

	if (cond!=1 && millis()-hb.last_warn>5000)
	{
	  PPRINT("W.cnd=");  Serial.print(cond);
//	  PPRINT(",compl="); Serial.print(complete);
//	  PPRINT(",none=");  Serial.print(none);
//	  PPRINT(",err=");   Serial.print(error);
//	  PPRINT(",exp=");   Serial.print(num_sensors);
//	  PPRINT(",avl=");   Serial.println(num_sensors_available);
	  hb.last_warn=millis();
	}
#else
  //PPRINTLN("D. No LED ind.");
#endif
}

#ifdef WITH_STARTTIMES
void dump_starttime()
{
	uint16_t start_hour;
	uint16_t curr_hour = ts_optime/60;

	uint8_t idx = eEE_READN(EE->start_hours.idx);
	idx = constrain(idx, 0, MAX_START_HOURS-1);

	// start reading one after the current index (this is the oldest entry)
	// and move on to younger ones with every iteration.
	uint8_t total = 0;
	for (uint8_t n=1; n<=MAX_START_HOURS; n++)
	{
		uint8_t i = (idx+n)%MAX_START_HOURS;
		start_hour = eEE_READN(EE->start_hours.hours[i]);
		if (start_hour>0)
		{
			total++;
			PPRINT("T:n="); Serial.print(total); PPRINT(",i="); Serial.print(i); PPRINT(",t="); Serial.print(start_hour); PPRINT("h,ago="); Serial.print(curr_hour-start_hour); PPRINTLN("h");
		}
	}
	PPRINT("T:n="); Serial.print(total+1); PPRINT(",i=-1,t="); Serial.print(ts_starttime); PPRINT("m,ago="); Serial.print(ts_optime-ts_starttime); PPRINTLN("m");
	PPRINTLN("T!");
}

void setup_starttime()
{	
	ts_starttime = ts_optime;
	uint8_t idx = eEE_READN(EE->start_hours.idx);
	idx = constrain(idx, 0, MAX_START_HOURS-1);

	uint16_t hour_curr = (ts_optime+30)/60;
	uint16_t hour_last = eEE_READN(EE->start_hours.hours[idx]);
	if (hour_curr>hour_last)
	{
		idx = (idx+1)%MAX_START_HOURS;
		eEE_WRITE(hour_curr, EE->start_hours.hours[idx]);
		eEE_WRITE(idx, EE->start_hours.idx);
	}
}
#endif


static void mydelay(long ms)
{
  uint32_t now, end = millis()+ms;
  do
  {
    now   = millis();

    #ifdef WITH_PING
    uint32_t delta = now-millis_ping;
    if (delta>1000)
    {
    	if (verbosity>V_SILENT)
    	{
    		PPRINT("P.s="); Serial.println(0.001f*now);
    	}
    	millis_ping = now;
    }
    #endif

    delay(1);
    serialEvent();
    check_serial();
    show_status(true);
  }
  while (now<end);
}

static void println_version()
{
	PPRINT("OturmaOdasi=v0.1."); Serial.print(MAGIC, HEX);
	PPRINT(",d="); PPRINT(__DATE__);
	Serial.println();
}

static void set_status_leds(uint8_t status)
{
	digitalWrite(PIN_STATUS_LED,  status);
	digitalWrite(PIN_ONBOARD_LED, status);

}

static void set_status_leds_analog(uint8_t value)
{
	analogWrite(PIN_STATUS_LED,  value);
	analogWrite(PIN_ONBOARD_LED, value);
}


void setup()
{
	// start serial port at the same baud rate avrdude uses to flash the pro mini
    // this way we'll be able to flash over bluetooth (pressing the reset button manually)
	Serial.begin(57600);
	PPRINT("I."); println_version();
            
	digitalWrite(PIN_RESET, HIGH);
	pinMode(PIN_RESET,       OUTPUT);

	pinMode(PIN_STATUS_LED,  OUTPUT);
	pinMode(PIN_ONBOARD_LED, OUTPUT);

        pinMode(RELAY_PINS[0], OUTPUT);
        pinMode(RELAY_PINS[1], OUTPUT);
        pinMode(RELAY_PINS[2], OUTPUT);
        pinMode(RELAY_PINS[3], OUTPUT);
  
        pinMode(PIN_PWM_R, OUTPUT);
        pinMode(PIN_PWM_G, OUTPUT);
        pinMode(PIN_PWM_B, OUTPUT);

	eeprom_init();
	#ifdef WITH_STARTTIMES
	setup_starttime();
	#endif

    mode_apply();
	dump_general_info();

	// indicate everything is okay: flicker status LED for 1 second and let strands
	// flash up for a second (after eeprom was read and actual pins are known)
	indicate_startup();

	#ifdef WITH_STATS
	/*
	int hour_index = get_current_hour();
	int day_index  = get_day_index();
	int week_index = get_week_index();
	PPRINT("D. INDEXES: hour="); Serial.print(hour_index); PPRINT(", day="); Serial.print(day_index); PPRINT(", week="); Serial.println(week_index);
	*/
	dump_stats();
	#endif

	#ifdef WITH_STARTTIMES
	dump_starttime();
	#endif

	sendHello();
}


void loop()
{
	//PPRINT("ts_optime: "); Serial.println(ts_optime);
	show_status(true);

	// check the temperature limits when the current ts changed (thus every full minute)
	if (ts_last==ts_optime) // minute did not change
	{
		long pause_len = MILLIS_PER_UPTIME/3;

		// start measuring the temepratures only if heartbeat is currently not beating
		// (i.e. showing steady state 0=LED lit) otherwise the animation will become a little bit bumpy
		boolean do_compute_temps = true;
		#ifdef WITH_HEARTBEAT
		do_compute_temps = (hb.last_status==0);
		#endif

		show_status(false);

		// wait a tiny amount of time if there was no measure
		//PPRINT("D. mydelay "); Serial.println(pause_len);
		mydelay(pause_len);
	}
	else // minute changed
	{
		ts_last = ts_optime;

		if (0==(millis()/10000))
		{
			if (severe_error)
			{
				PPRINTLN("E.SEVERR");
			}
		}

		// update stats every full hour. since actual write cycles are distributed equally
		// over MAX_HOURS (24) entries (see: s_stats_hour), there is only 1 write cycle per day
		// thus max. write cycles now exceeded before 270 years
		//if (0==ts_optime%60)
		if (0==ts_optime%60)
		{
			save_statistics();
			#ifdef WITH_SHOW_EVENTS
			if (verbosity>V_SILENT)
			{
				PPRINTLN("EV.e=stats.savd");
			}
			#endif

			#ifdef OSX
			usleep(30000);
			#endif
		}

		show_status(false);
	}

	check_serial();
    check_ir();
	update_uptime();

    changeCheckSave();
}





