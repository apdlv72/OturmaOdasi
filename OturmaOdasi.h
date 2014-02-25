#ifndef OturmaOdasi_H_
#define OturmaOdasi_H_

#include "Arduino.h"
#include "eEEPROM.h"
#define MAX_START_HOURS 15

#define STRIP_TYPE_RGB

typedef struct s_preset
{
	uint8_t mode;          // the mode of operation
	uint8_t mode_bits;
	uint8_t randomTreshold; // defined likelihood for disturbances: 0-never, 100-always (in terms of frames)
	//uint8_t disturbWidth;

	//int16_t disturbStrength; // aplitude/strength allows for random disturbances
	uint8_t  offsetR;       // base value for R channel (0-255)
	uint8_t  offsetG;       // base value for G channel
	uint8_t  offsetB;       // base value for B channel
	//int8_t amplitude;     // factor 0-255 defines the impact of disturbances on the outpul values
	uint8_t  brightness;    // dimming factor: 0-dark, 255-max. brightness

	uint16_t delayTime;    // main delay controls effecive Fps rate
	uint16_t fadePrescale; // defines after how many sub ticks a tick of the fading counter gets incremented
	//uint16_t magic;        // magic 4711 saved in EEprom to check if anything was saved at all in a slot before
	//uint8_t  power;
}
s_preset;


typedef struct 
{
  uint16_t r : 8;
  uint16_t g : 8;
  uint16_t b : 8;
  uint16_t brightness : 8;
}
s_rgb;

typedef enum 
{
  BIT_DOWN   =  1,
  BIT_UP1    =  2,
  BIT_UP2    =  4,
  BIT_STRIP1 =  8,
  BIT_STRIP2 = 16,
  BIT_FAKETV = 32,
} e_mode;


uint8_t mode_sequence[] =
{
   BIT_UP1 | BIT_UP2,
   BIT_UP1 | BIT_UP2 | BIT_DOWN,
   BIT_UP2 | BIT_DOWN,
   BIT_DOWN,
   BIT_STRIP1,
   BIT_STRIP1 | BIT_STRIP2,
   BIT_STRIP2,
   BIT_FAKETV
};

#define MODE_MIN 0
#define MODE_MAX sizeof(mode_sequence)-1

/*
#ifdef STRIP_TYPE_RGB
  #define MODE_WATER     0 // water simulation,  monochromatic, brightness varies
  #define MODE_WATER2    1 // mostly monochromatic, but slight displacement on G/B channel causes some coloring
  #define MODE_FADE      2 // like WATER2, however base color fades over time
  #define MODE_RAINBOW   3 // displays whole rainbow range on the strip, otherwise like WATER
  #define MODE_CONST     4 // constant color only, but brightness may be set by the user
  #define MODE_RAINFLOW  5 // like RAINBOW2, ripple data used as color displacement
  #define MODE_MAX       5
#endif
*/


typedef struct
{
        uint8_t  idx;
        uint16_t hours[MAX_START_HOURS];

} s_start_hours; // 81 bytes


// contains everything that goes to EEprom
typedef struct
{
    uint16_t      magic;            // 2 bytes
    uint16_t      optime_minutes;
    s_start_hours start_hours;    
    
    uint8_t       mode_index;
    uint8_t       mode_bits;
    s_rgb         rgb;

    s_preset 	  preset;
    s_preset 	  presets[6];
        
} s_eeprom_data; // 998 bytes < 1024

//#define DUMP_EESIZE_COMPILE_TIME
#ifdef DUMP_EESIZE_COMPILE_TIME
template<int s> struct total_size_is_;
#warning For total size of EEPROM data see error in the next line
total_size_is_<sizeof(s_eeprom_data)> size_s_eeprom_data; // 998
//total_size_is_<sizeof(s_sensor_info)>  size_; // 45
//total_size_is_<sizeof(strand_infos)>  size_; // 135
//total_size_is_<sizeof(s_strand_info_eeprom)> size_; // 27
//total_size_is_<sizeof(s_statistics)>  size_; // 104
#endif

//end of add your includes here
#ifdef __cplusplus
//extern "C" {
#endif

void loop();
void setup();

#ifdef __cplusplus
//} // extern "C"
#endif

//add your function definitions for the project FrostProtect here


//Do not add code below this line
#endif /* FrostProtect_H_ */
