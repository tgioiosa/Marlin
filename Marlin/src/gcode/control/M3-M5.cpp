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
 *
 * Updated: 12/16/22
 */

#include "../../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "../gcode.h"
#include "../../module/temperature.h"           //TG 9/21/21 added
#include "../../feature/spindle_laser.h"
#include "../../module/stepper.h"
#include "../../module/planner.h"
#if ENABLED(TG_I2C_SUPPORT)	//TG 12/16/22
  #include "../../module/TG_I2C/TG_I2CSlave.h"    //TG 5/12/22 added for I2C comm with AVR Triac Controller board.
#endif
#if ENABLED(USE_RPM_SENSOR)
  #include "../../module/rpmSensor/RPMTimer.h"  //TG 12/20/22 put #if clause
#endif 
#if ENABLED(VFD_CONTROLLER)	//TG 12/16/22
  #include "../../module/vfd.h"
#endif

/**
 * Laser:
 *  M3 - Laser ON/Power (Ramped power)
 *  M4 - Laser ON/Power (Ramped power)
 *  M5 - Set power output to 0 (leaving inline mode unchanged).
 *
 *  M3I - Enable continuous inline power to be processed by the planner, with power
 *        calculated and set in the planner blocks, processed inline during stepping.
 *        Within inline mode M3 S-Values will set the power for the next moves e.g. G1 X10 Y10 powers on with the last S-Value.
 *        M3I must be set before using planner-synced M3 inline S-Values (LASER_POWER_SYNC).
 *
 *  M4I - Set dynamic mode which calculates laser power OCR based on the current feedrate.
 *
 *  M5I - Clear inline mode and set power to 0.
 *
 * Spindle:
 *  M3 - Spindle ON (Clockwise)
 *  M4 - Spindle ON (Counter-clockwise)
 *  M5 - Spindle OFF
 *
 * Parameters:
 *  S<power> - Set power. S0 will turn the spindle/laser off, except in relative mode.
 *  O<ocr>   - Set power and OCR (oscillator count register)
 *
 *  If no PWM pin is defined then M3/M4 just turns it on or off.
 *
 *  At least 12.8kHz (50Hz * 256) is needed for Spindle PWM.
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

/****************************************************************************************************************************
*TG ***** NOTE: With the new AVR Triac Controller board, the actual spindle speed is controlled there, and Marlin PWM pin P1_23
* is no longer used for speed control. The following code needs only to read the speed from M3 and M4 commands and forward
* it to the AVR Triac Controller via TARGET_RPM variable over I2C comm.
*
* This is changing as of 12/16/22, the code will now have to handle different options:
* 1= original Marlin Spindle with SPINDLE_LASER_USE_PWM enabled  to control speed
* 2= AVR_TRIAC_CONTROLLER used for speed control and PID, RPM sensor is handled there
* 3= VFD_CONTROLLER used for speed control, RPM sensor handled by Marlin
*************************************************************************************************************************/

void GcodeSuite::M3_M4(const bool is_M4) {      //TG set cutter.unitPower from 'S' parameter or default SPEED_POWER_STARTUP
  #if LASER_SAFETY_TIMEOUT_MS > 0
    reset_stepper_timeout(); // Reset timeout to allow subsequent G-code to power the laser (imm.)
  #endif
  
  if (cutter.cutter_mode == CUTTER_MODE_STANDARD)
    planner.synchronize();   // Wait for previous movement commands (G0/G1/G2/G3) to complete before changing power
  
  #if ENABLED(LASER_FEATURE)
    if (parser.seen_test('I')) {
      cutter.cutter_mode = is_M4 ? CUTTER_MODE_DYNAMIC : CUTTER_MODE_CONTINUOUS;
      cutter.inline_power(0);
      cutter.set_enabled(true);
    }
  #endif
  //WRITE(P4_28,1);
  auto get_s_power = [] {  //TG this lambda function inside a function gets the SXXXXX value, i.e. 7000 for S7000, and sets unitPower
    float u = 0;
    if (parser.seenval('S')) {								// speed was given
      const float v = parser.value_float();					// v is the RPM or PWM or % Target Value from the LCD display
      u = TERN(LASER_POWER_TRAP, v, cutter.power_to_range(v));
	  }
    else if (cutter.cutter_mode == CUTTER_MODE_STANDARD) 	// if no S value given, use SPEED_POWER_STARTUP
      u = cutter.cpwr_to_upwr(SPEED_POWER_STARTUP);

	cutter.menuPower = cutter.unitPower = u;				// update menuPower for display, unitPower is RPM or PWM or %

    // PWM not implied, power converted to OCR from unit definition and on/off if not PWM.
    cutter.power = TERN(SPINDLE_LASER_USE_PWM, cutter.upower_to_ocr(u), u > 0 ? SPINDLE_LASER_PWM_RES : 0);
    return u;
  };

  if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS || cutter.cutter_mode == CUTTER_MODE_DYNAMIC) {  // Laser power in inline mode
    #if ENABLED(LASER_FEATURE)
      planner.laser_inline.status.isPowered = true;                                                 // M3 or M4 is powered either way
      get_s_power();                                                                                // Update cutter.power if seen
      #if ENABLED(LASER_POWER_SYNC)
        // With power sync we only set power so it does not effect queued inline power sets
        planner.buffer_sync_block(BLOCK_BIT_LASER_PWR);                                            // Send the flag, queueing inline power
      #else
        planner.synchronize();		// Wait for previous movement commands (G0/G0/G2/G3) to complete before changing power
        cutter.inline_power(cutter.power);
      #endif
    #endif
  }
  else {
    cutter.set_enabled(true);		// turn on ENABLE signal
    get_s_power();
    if (cutter.spindle_use_pid == false)  //TG 9/15/21 if using classic PID algorithm in RPMTimer.cpp, it will set the power, so skip below
    {
      cutter.apply_power(
        #if ENABLED(SPINDLE_SERVO)
          cutter.unitPower
        #elif ENABLED(SPINDLE_LASER_USE_PWM)
          cutter.upower_to_ocr(cutter.unitPower)				// set a PWM value
        #else
          cutter.unitPower > 0 ? SPINDLE_LASER_PWM_RES : 0	// set on=1023 or off=0
        #endif
      );
    }
    TERN_(SPINDLE_CHANGE_DIR, cutter.set_reverse(is_M4));
  }

  SpindleLaser::isReadyForUI = true;
    
  //TG added 5/12/22 - update TARGET_RPM variable used for I2C reply to AVR Controller request or Serial/RS485 send to VFD Controller
  cutter.TARGET_RPM = cutter.menuPower;               
  //TG added 9/21/21 to also set value in spindle_speed array, index 0 (there's only 1 spindle), this also echoes the Target/Actual to serial ports
  Temperature::set_spindle_speed(0,cutter.unitPower);   
  
  #if ENABLED(VFD_CONTROLLER)   //TG 12/16/22 - VFD Controller needs to be sent the TARGET_RPM value and set to RUN
          
    uint16_t data = (10 * cutter.TARGET_RPM)/60;  // Vevor frequency specified 1 dec point so * 10
    if (VFDpresent==true)							            // if VFD connected
    { 
      statusPollingAllowed = false;               // stop status polling we need the RS485 port
      // First, set target frequency cmd to serial port to VFD and get success or fail response  
      bool status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_FREQUENCY, data);

      // check is_M4 and RUN REVERSE to VFD if needed, otherwise RUN FORWARD
      // if all good then run forward SEND TO SERIAL PORT AND OPTIONAL WAIT FOR RESPONSE
      if (status==true) {
        status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_MAIN_CONTROL_BITS, (is_M4 ? sRUN_REV : sRUN_FWD));
      }
      statusPollingAllowed = true;                // allow status polling, we're done
    }
  
  #endif

  //WRITE(P4_28,0);
}

/**
 * M5 - Cutter OFF (when moves are complete)
 */
void GcodeSuite::M5() {
  planner.synchronize();
  cutter.power = 0;
  cutter.apply_power(0);                // M5 just kills power, leaving inline mode unchanged
  cutter.set_enabled(false);            // also sets power to zero (but not unitPower!)
  cutter.unitPower = 0;                 //TG make sure to zero the unitPower, otherwise it holds last target
  cutter.menuPower = 0;                 //update menuPower for display, unitPower is RPM or PWM or %
  
  if (cutter.cutter_mode != CUTTER_MODE_STANDARD) {
    if (parser.seen_test('I')) {
      TERN_(LASER_FEATURE, cutter.inline_power(cutter.power));
      cutter.set_enabled(false);                  // Needs to happen while we are in inline mode to clear inline power.
      cutter.cutter_mode = CUTTER_MODE_STANDARD;  // Switch from inline to standard mode.
    }
  } 
  cutter.set_enabled(false);                      // Disable enable output setting
  
  //TG added 5/12/22 - update TARGET_RPM for I2C to reply to AVR Controller request or Serial/RS485 send to VFD Controller
  cutter.TARGET_RPM = 0;                              
  Temperature::set_spindle_speed(0,0);  //TG added 9/24/21 to set value in spindle_speed array, index 0, 
                                        //this also echoes the Target/Actual to serial ports
 
 #if ENABLED(VFD_CONTROLLER)            //TG 12/16/22 - VFD Controller needs to be sent the TARGET_RPM value and set to STOP
  if (VFDpresent==true)                 // if VFD connected
  {							            
    statusPollingAllowed = false;       // stop status polling we need the RS485 port
    // Send STOP command to VFD 
    bool status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_MAIN_CONTROL_BITS, sRUN_STOP);
  
    // if not successful, send one more time
    if (status == false)
      status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_MAIN_CONTROL_BITS, sRUN_STOP);
    statusPollingAllowed = true;        // allow status polling, we're done
  }
 #endif // ENABLED(VFD_CONTROLLER)  

}

#endif // HAS_CUTTER
