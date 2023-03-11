/** //TG MODIFIED BY T.GIOIOSA
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "../gcode.h"
#include "../../module/temperature.h"           //TG 9/21/21 added
#include "../../feature/spindle_laser.h"
#include "../../module/stepper.h"
#include "../../module/TG_I2C/TG_I2CSlave.h"    //TG 5/12/22 added for I2C comm with AVR Triac Controller board.
//#include "../../module/rpmSensor/RPMTimer.h"  //TG 8/31/21 added, removed 5/12/22 - no longer needed with AVR Triac Controller board.

/**
 * Laser:
 *  M3 - Laser ON/Power (Ramped power)
 *  M4 - Laser ON/Power (Continuous power)
 *
 * Spindle:
 *  M3 - Spindle ON (Clockwise)
 *  M4 - Spindle ON (Counter-clockwise)
 *
 * Parameters:
 *  S<power> - Set power. S0 will turn the spindle/laser off, except in relative mode.
 *  O<ocr>   - Set power and OCR (oscillator count register)
 *
 *  If no PWM pin is defined then M3/M4 just turns it on.
 *
 *  At least 12.8KHz (50Hz * 256) is needed for Spindle PWM.
 *  Hardware PWM is required on AVR. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENA_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */

//***************************************************************************************************************************
//TG ***** NOTE: With the new AVR Triac Controller board, the actual spindle speed is controlled there, and Marlin PWM pin P1_23
// is no longer used for speed control. The following code needs only to read the speed from M3 and M4 commands and forward
// it to the AVR Triac Controller via TARGET_RPM variable over I2C comm.
//***************************************************************************************************************************

void GcodeSuite::M3_M4(const bool is_M4) {      //TG set cutter.unitPower from 'S' parameter or default SPEED_POWER_STARTUP
  #if EITHER(SPINDLE_LASER_USE_PWM, SPINDLE_SERVO)
  //WRITE(P4_28,1);
  auto get_s_power = [] {       //TG this lambda function inside a function gets the SXXXXX value, i.e. 7000 for S7000, and sets unitPower
    if (parser.seenval('S')) {
      const float spwr = parser.value_float();  //TG spwr is the RPM or PWM or % Target Value from the LCD display
            
      #if ENABLED(SPINDLE_SERVO)
        cutter.unitPower = spwr;
      #else
        cutter.unitPower = TERN(SPINDLE_LASER_USE_PWM,                              //TG if PWM in use then
                              cutter.power_to_range(cutter_power_t(round(spwr))),   //set unitPower to range-limited spwr in RPM or PWM or %
                              spwr > 0 ? SPINDLE_LASER_PWM_RES : 0);                //else zero or full power - 9/30/21 changed max OCR, was 255
      #endif
    }
    else    // if no S value given, convert SPEED_POWER_STARTUP to unitPower
        cutter.unitPower = cutter.cpwr_to_upwr(SPEED_POWER_STARTUP);                //TG default to SPEED_POWER_STARTUP if no 'S' parameter
    return cutter.unitPower;
  };
  #endif

  #if ENABLED(LASER_POWER_INLINE)
    if (parser.seen('I') == DISABLED(LASER_POWER_INLINE_INVERT)) {
      // Laser power in inline mode
      cutter.inline_direction(is_M4); // Should always be unused
      #if ENABLED(SPINDLE_LASER_USE_PWM)
        if (parser.seen('O')) {
          cutter.unitPower = cutter.power_to_range(parser.value_byte(), 0);
          cutter.inline_ocr_power(cutter.unitPower); // The OCR is a value from 0 to 255 (uint8_t)
        }
        else
          cutter.inline_power(cutter.upower_to_ocr(get_s_power()));
      #else
        cutter.set_inline_enabled(true);
      #endif
      return;
    }
    // Non-inline, standard case
    cutter.inline_disable(); // Prevent future blocks re-setting the power
  #endif

  planner.synchronize();   // Wait for previous movement commands (G0/G0/G2/G3) to complete before changing power
  cutter.set_reverse(is_M4);

  #if ENABLED(SPINDLE_LASER_USE_PWM)                                      
    if (parser.seenval('O')) {
      cutter.unitPower = cutter.power_to_range(parser.value_byte(), 0);   //TG get value after 'O' for OCR (PWM) power
      cutter.set_hires_ocr_power(cutter.unitPower);                       //TG Set OCR to value from 0 to SPEED_POWER_STARTUP (uint16_t)
    }
    else{
      cutter.set_power(cutter.upower_to_ocr(get_s_power()));    //TG get 'S' power (or default speed if no 'S') to OCR power, then set it at PWM
    }
  #elif ENABLED(SPINDLE_SERVO)
    if (cutter.spindle_use_pid == false)  //TG 9/15/21 if using classic PID algorithm in RPMTimer.cpp, it will set the power, so skip below
      cutter.set_power(get_s_power());
  #else
    cutter.set_enabled(true);
  #endif
  
  cutter.menuPower = cutter.unitPower;                  // update menuPower for display, unitPower is RPM or PWM or % 
  SpindleLaser::isReady = true;
  
  TARGET_RPM = cutter.menuPower;                        //TG added 5/12/22 to update TARGET_RPM for I2C code to send to AVR CNC Controller
  Temperature::set_spindle_speed(0,cutter.unitPower);   //TG added 9/21/21 to set value in spindle_speed array
  //WRITE(P4_28,0);
}

/**
 * M5 - Cutter OFF (when moves are complete)
 */
void GcodeSuite::M5() {
  #if ENABLED(LASER_POWER_INLINE)
    if (parser.seen('I') == DISABLED(LASER_POWER_INLINE_INVERT)) {
      cutter.set_inline_enabled(false); // Laser power in inline mode
      return;
    }
    // Non-inline, standard case
    cutter.inline_disable(); // Prevent future blocks re-setting the power
  #endif
  planner.synchronize();
  cutter.set_enabled(false);                    // also sets power to zero (but not unitPower!)
  
  cutter.unitPower = 0;                         //TG make sure to zero the unitPower, otherwise it holds last target
  cutter.menuPower = 0;                         // update menuPower for display, unitPower is RPM or PWM or %
  TARGET_RPM = 0;                               //TG added 5/12/22 to update TARGET_RPM for I2C code to send to AVR CNC Controller
  Temperature::set_spindle_speed(0,0);          //TG - 9/24/21 added
}

#endif // HAS_CUTTER
