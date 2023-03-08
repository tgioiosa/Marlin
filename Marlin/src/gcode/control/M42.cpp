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

#if ENABLED(DIRECT_PIN_CONTROL)

#include "../gcode.h"
#include "../../MarlinCore.h" // for pin_is_protected

#if HAS_FAN
  #include "../../module/temperature.h"
#endif

#ifdef MAPLE_STM32F1
  // these are enums on the F1...
  #define INPUT_PULLDOWN INPUT_PULLDOWN
  #define INPUT_ANALOG INPUT_ANALOG
  #define OUTPUT_OPEN_DRAIN OUTPUT_OPEN_DRAIN
#endif

void protected_pin_err() {
  SERIAL_ERROR_MSG(STR_ERR_PROTECTED_PIN);
}

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *          For LPC1768 specify pin P1_02 as M42 P102,
 *                                  P1_20 as M42 P120, etc.
 *
 *  S<byte> Pin status from 0 - 255
 *  I       Flag to ignore Marlin's pin protection
 *
 *  T<mode> Pin mode: 0=INPUT  1=OUTPUT  2=INPUT_PULLUP  3=INPUT_PULLDOWN
 */
void GcodeSuite::M42() {
  const int pin_index = PARSED_PIN_INDEX('P', GET_PIN_MAP_INDEX(LED_PIN));
  if (pin_index < 0) return;

  const pin_t pin = GET_PIN_MAP_PIN(pin_index);

  if (!parser.boolval('I') && pin_is_protected(pin)) return protected_pin_err();

  bool avoidWrite = false;
  if (parser.seenval('T')) {
    switch (parser.value_byte()) {
      case 0: pinMode(pin, INPUT); avoidWrite = true; break;
      case 1: pinMode(pin, OUTPUT); break;
      case 2: pinMode(pin, INPUT_PULLUP); avoidWrite = true; break;
      #ifdef INPUT_PULLDOWN
        case 3: pinMode(pin, INPUT_PULLDOWN); avoidWrite = true; break;
      #endif
      #ifdef INPUT_ANALOG
        case 4: pinMode(pin, INPUT_ANALOG); avoidWrite = true; break;
      #endif
      #ifdef OUTPUT_OPEN_DRAIN
        case 5: pinMode(pin, OUTPUT_OPEN_DRAIN); break;
      #endif
      default: SERIAL_ECHOLNPGM("Invalid Pin Mode"); return;
    }
  }

  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();

  #if HAS_FAN
    switch (pin) {
      #if HAS_FAN0
        case FAN0_PIN: thermalManager.fan_speed[0] = pin_status; return;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: thermalManager.fan_speed[1] = pin_status; return;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: thermalManager.fan_speed[2] = pin_status; return;
      #endif
      #if HAS_FAN3
        case FAN3_PIN: thermalManager.fan_speed[3] = pin_status; return;
      #endif
      #if HAS_FAN4
        case FAN4_PIN: thermalManager.fan_speed[4] = pin_status; return;
      #endif
      #if HAS_FAN5
        case FAN5_PIN: thermalManager.fan_speed[5] = pin_status; return;
      #endif
      #if HAS_FAN6
        case FAN6_PIN: thermalManager.fan_speed[6] = pin_status; return;
      #endif
      #if HAS_FAN7
        case FAN7_PIN: thermalManager.fan_speed[7] = pin_status; return;
      #endif
    }
  #endif

  if (avoidWrite) {
    SERIAL_ECHOLNPGM("?Cannot write to INPUT");
    return;
  }
  //TG - 10/7/21 this is old version of below
  //pinMode(pin, OUTPUT);
  //extDigitalWrite(pin, pin_status);
  //analogWrite(pin, pin_status);
  
  //***************************************************************************************************************
  //TG 6/27/21 Modified the original code (3 lines above) with new code. Added new parameter 'A' to specify
  //if pin should be treated as analog PWM, example: "M42 P123 S127 A1" where A1 specifies analog PWM. If the
  //'A1' is left off, pin will default to GPIO digital output pin. The '1' after 'A' can be any number, just
  //needs to be at least one digit. In PWM mode, S=0 to 255. In Digital mode S=0 is OFF, S=1 to 255 is ON.
  //The analog mode can write to Hardware or Software PWM. If the pin is not a Hardware PWM pin on the LPC,
  //then the Software PWM will be used. Both PWM frequencies are 50Hz by default (in pwm.h), but the Hardware
  //PWM(LPC_PWM1) freq is set higher to (SPINDLE_LASER_FREQUENCY) if SPINDLE_FEATURE or LASER_FEATURE is enabled.
  //To change Software PWM frequency, make a call to SoftwarePWM::set_frequency(frequency) or change pwm.h.
  if (parser.seenval('A'))    
  {                           
    LPC176x::pwm_attach_pin(pin, pin_status); // re-attach pin as PWM (when pin was set to GPIO by extDigitalWrite)
    analogWrite(pin, pin_status);             // treat as PWM (hardware or software)
  }
  else
  {  
    //if (LPC176x::HardwarePWM::active(pin) || LPC176x::SoftwarePWM::active(pin))
    LPC176x::pwm_detach_pin(pin);             // if previously mapped as a PWM pin, detach it first 
  // An OUTPUT_OPEN_DRAIN should not be changed to normal OUTPUT (STM32)
  // Use M42 Px M1/5 S0/1 to set the output type and then set value
  #ifndef OUTPUT_OPEN_DRAIN
    pinMode(pin, OUTPUT);
  #endif
    extDigitalWrite(pin, pin_status);         // treat as GPIO pin
  }

  //TG 9/25/22 - added this to update TFT whenever Marlin sets vacuum on or off during an SD print or Remote Terminal
  // M42 command. Otherwise without this, the TFT would only know the vacuum state if changed on the TFT screen itself.
  if(pin == VACUUM_ENA_PIN)
  {
    SERIAL_ECHOPGM("M7985 S", pin_status);    // signal the TFT that vacuum changed state
    SERIAL_EOL();
  }
 
  //TG 6/27/21 this line was useful for debugging to see the Function mode of the pin
  //LPC176x::Function pf = (LPC176x::Function)LPC176x::pin_type{pin}.function();

  //TG 6/26/21 ALSO ! Be sure to use modified versions of SoftwarePWM.cpp and SoftwarePWM.h
  //from "C:\Users\tony\.platformio\packages\framework-arduino-lpc176x\system\lpc176x\"
  //They are modified to use Timer2 for SoftPWM (instead of Timer3 as in the original files) !
  //Timer3 has been reassigned for use as RPM sensor capture.
  //***************************************************************************************************************

  #ifdef ARDUINO_ARCH_STM32
    // A simple I/O will be set to 0 by set_pwm_duty()
    if (pin_status <= 1 && !PWM_PIN(pin)) return;
  #endif
  hal.set_pwm_duty(pin, pin_status);
}

#endif // DIRECT_PIN_CONTROL
