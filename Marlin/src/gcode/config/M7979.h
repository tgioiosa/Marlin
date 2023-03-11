/** //TG MODIFIED BY T.GIOIOSA
 * Support for Custom G Code additions for CNC and AVR Triac Controller
 * 
 * 7/23/22 - 10/4/22
 */

#ifndef M7979_H_
#define M7979_H_

#include "../../inc/MarlinConfig.h"
#include "../gcode.h"
#include "../../feature/spindle_laser.h"

// this struct must be the same alignment and total size in all three project modules
// BigTree TFT35, Marlin, and AVR CNC Control
typedef struct 
{
  uint8_t PIDFLAG;
  uint8_t Reset_Flag;
  uint8_t Display_Page;
  uint8_t PID_Speed;
  uint8_t Data_Interval;
  uint8_t PID_Interval;
  float KP;
  float KI;
  float KD;
  uint8_t Update_EEPROM;
  uint8_t Reload_Preset;
  uint16_t EE_chksum;
} __attribute__ ((packed)) AVRINFOTYPE;   //TG packed attribute forces smallest size for structure 
extern AVRINFOTYPE AVRInfoBlock;

uint16_t get_chksum();

extern volatile uint8_t RecvAVRBlockComplete;
extern volatile uint8_t sendAVRBlockComplete;

#endif