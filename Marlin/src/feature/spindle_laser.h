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
#pragma once

/**
 * feature/spindle_laser.h
 * Support for Laser Power or Spindle Power & Direction
 */
#include "math.h"
#include "../inc/MarlinConfig.h"

#include "spindle_laser_types.h"

#if USE_BEEPER
  #include "../libs/buzzer.h"
#endif

#if ENABLED(LASER_POWER_INLINE)
  #include "../module/planner.h"
#endif

#define PCT_TO_PWM(X) ((X) * (float)SPINDLE_LASER_PWM_RES / 100.0f + 0.5f)  //TG 10/3/21 fixed this to return correct value by using (float)
#define PCT_TO_SERVO(X) ((X) * 180 / 100)

#ifndef SPEED_POWER_INTERCEPT
  #define SPEED_POWER_INTERCEPT 0
#endif

// #define _MAP(N,S1,S2,D1,D2) ((N)*_MAX((D2)-(D1),0)/_MAX((S2)-(S1),1)+(D1))

class SpindleLaser {
public:
  //TG - takes a float as percent and returns 0 to SPINDLE_LASER_PWM_RES, minimum resolution is 1 count
  static const inline uint16_t pct_to_ocr(const_float_t pct) { return uint16_t(PCT_TO_PWM(pct)); }  //TG 9/30/21 changed to uint16_t for more resolution
  // cpower = configured values (e.g., SPEED_POWER_MAX)

  // Convert configured power range to a percentage
  //TG - !!!!warning returned an int8_t so inputs under 1% get lost (returns 0)!!!!
  //TG - 9/30/21 Changed this sub to return a float to allow percentages under 1%
  static const inline float cpwr_to_pct(const cutter_cpower_t cpwr) {
    constexpr cutter_cpower_t power_floor = TERN(CUTTER_POWER_RELATIVE, SPEED_POWER_MIN, 0),
                              power_range = SPEED_POWER_MAX - power_floor;
    //return cpwr ? round(100.0f * (cpwr - power_floor) / power_range) : 0;   //TG - 9/30/21 no rounding for less than 1%
    return cpwr ? 100.0f * (cpwr - power_floor) / power_range : 0;
  }

  // Convert a cpower (e.g., SPEED_POWER_STARTUP) to unit power (upwr, upower),
  // which can be PWM, Percent, Servo angle, or RPM (rel/abs).
  static const inline cutter_power_t cpwr_to_upwr(const cutter_cpower_t cpwr) { // STARTUP power to Unit power
    const cutter_power_t upwr = (
      #if ENABLED(SPINDLE_FEATURE)
        // Spindle configured values are in RPM
        #if CUTTER_UNIT_IS(RPM)
          cpwr                            // to RPM
        #elif CUTTER_UNIT_IS(PERCENT)     // to PCT
          cpwr_to_pct(cpwr)
        #elif CUTTER_UNIT_IS(SERVO)       // to SERVO angle
          PCT_TO_SERVO(cpwr_to_pct(cpwr))
        #else                             // to PWM
          PCT_TO_PWM(cpwr_to_pct(cpwr))
        #endif
      #else
        // Laser configured values are in PCT
        #if CUTTER_UNIT_IS(PWM255)
          PCT_TO_PWM(cpwr)
        #else
          cpwr                            // to RPM/PCT
        #endif
      #endif
    );
    return upwr;
  }

  static const cutter_power_t mpower_min() { return cpwr_to_upwr(SPEED_POWER_MIN); }
  static const cutter_power_t mpower_max() { return cpwr_to_upwr(SPEED_POWER_MAX); }

  #if ENABLED(LASER_FEATURE)
    static cutter_test_pulse_t testPulse; // Test fire Pulse ms value
  #endif

  static bool isReady;                    // Ready to apply power setting from the UI to OCR
  static uint16_t power;                  //TG - the current power(always in PWM counts) to apply (in OCR) //TG 9/30/21 changed to uint16_t for more resolution
  static bool spindle_use_pid;            //TG 9/27/21 added to control whether PID speed control is used or not

  #if ENABLED(MARLIN_DEV_MODE)
    static cutter_frequency_t frequency;  // Set PWM frequency; range: 2K-50K
  #endif

  static cutter_power_t menuPower,        // Power as set via LCD menu in PWM, Percentage or RPM
                        unitPower;        // Power as displayed status in PWM, Percentage or RPM

  static void init();

  #if ENABLED(MARLIN_DEV_MODE)
    static inline void refresh_frequency() { set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), frequency); }
  #endif

  // Modifying this function should update everywhere, it just returns 1 if power > 0
  static inline bool enabled(const cutter_power_t opwr) { return opwr > 0; }
  static inline bool enabled() { return enabled(power); }

  static void apply_power(const uint16_t inpow);              //TG - 9/30/21 changed to 16-bit for hires PWM > 8 bit

  FORCE_INLINE static void refresh() { apply_power(power); }
  FORCE_INLINE static void set_power(const uint16_t upwr) { power = upwr; refresh(); }  //TG - 9/30/21 changed to 16-bit for hires PWM > 8 bit

  #if ENABLED(SPINDLE_LASER_USE_PWM)

    private:

    static void _set_ocr16(const uint16_t ocr16);         //TG - 9/30/21 customized for hires PWM 16-bit

    public:

    static void set_ocr16(const uint16_t ocr16);          //TG - 9/30/21 customized for hires PWM 16-bit
    static inline void set_hires_ocr_power(const uint16_t ocr) { power = ocr; set_ocr16(ocr); }  //TG - 9/30/21 customized for hires PWM 16-bit
    static void ocr16_off();                              //TG - 9/30/21 customized for hires PWM 16-bit

    // Used to update output for power->OCR translation
    //TG 9/30/21 changed to return 16-bit OCR. The called cpwr_to_pct() now returns a true float 
    static inline uint16_t upower_to_ocr(const cutter_power_t upwr) {  //TG 9/30/21 changed to uint16_t for more resolution
      return (
        #if CUTTER_UNIT_IS(PWM255)
          uint16_t(upwr)
        #elif CUTTER_UNIT_IS(PERCENT)
          pct_to_ocr(upwr)
        #else
          uint16_t(pct_to_ocr(cpwr_to_pct(upwr))) //TG - 9/30/21 both called functions have been modified to handle floats now!
        #endif                                   
      );
    }

    /**
     * Correct power to configured range
     */
    static cutter_power_t power_to_range(const cutter_power_t pwr) {
      return power_to_range(pwr, _CUTTER_POWER(CUTTER_POWER_UNIT));
    }

    static cutter_power_t power_to_range(const cutter_power_t pwr, const uint8_t pwrUnit) {
      static constexpr float
        min_pct = TERN(CUTTER_POWER_RELATIVE, 0, TERN(SPINDLE_FEATURE, round(100.0f * (SPEED_POWER_MIN) / (SPEED_POWER_MAX)), SPEED_POWER_MIN)),
        max_pct = TERN(SPINDLE_FEATURE, 100, SPEED_POWER_MAX);
      if (pwr <= 0) return 0;
      cutter_power_t upwr;
      switch (pwrUnit) {
        case _CUTTER_POWER_PWM255:
          upwr = cutter_power_t(
              (pwr < pct_to_ocr(min_pct)) ? pct_to_ocr(min_pct) // Use minimum if set below
            : (pwr > pct_to_ocr(max_pct)) ? pct_to_ocr(max_pct) // Use maximum if set above
            :  pwr
          );
          break;
        case _CUTTER_POWER_PERCENT:
          upwr = cutter_power_t(
              (pwr < min_pct) ? min_pct                         // Use minimum if set below
            : (pwr > max_pct) ? max_pct                         // Use maximum if set above
            :  pwr                                              // PCT
          );
          break;
        case _CUTTER_POWER_RPM:
          upwr = cutter_power_t(
              (pwr < SPEED_POWER_MIN) ? SPEED_POWER_MIN         // Use minimum if set below
            : (pwr > SPEED_POWER_MAX) ? SPEED_POWER_MAX         // Use maximum if set above
            : pwr                                               // Calculate OCR value
          );
          break;
        default: break;
      }
      return upwr;
    }

  #endif // SPINDLE_LASER_USE_PWM

  /**
   * Enable/Disable spindle/laser
   * @param enable true = enable; false = disable
   */
  //TG - 9/30/21 - if enabled=1 sets power to SPEED_POWER_STARTUP (if PWM), or sets to max OCR
  static void set_enabled(const bool enable) {
    uint8_t value = 0;
    if (enable) {
      #if ENABLED(SPINDLE_LASER_USE_PWM)
        if (power)
          value = power;
        else if (unitPower)
          value = upower_to_ocr(cpwr_to_upwr(SPEED_POWER_STARTUP));
      #else
        value = SPINDLE_LASER_PWM_RES;  //TG - 10/7/21
      #endif
    }
    set_power(value);
  }

  static void disable() { isReady = false; set_enabled(false); }

  /**
   * Wait for spindle to spin up or spin down
   *
   * @param on true = state to on; false = state to off.
   */
  static void power_delay(const bool on) {
    #if DISABLED(LASER_POWER_INLINE)
      safe_delay(on ? SPINDLE_LASER_POWERUP_DELAY : SPINDLE_LASER_POWERDOWN_DELAY);
    #endif
  }

  #if ENABLED(SPINDLE_CHANGE_DIR)
    static void set_reverse(const bool reverse);
    static bool is_reverse() { return READ(SPINDLE_DIR_PIN) == SPINDLE_INVERT_DIR; }
  #else
    static void set_reverse(const bool) {}
    static bool is_reverse() { return false; }
  #endif

  #if ENABLED(AIR_EVACUATION)
    static void air_evac_enable();         // Turn On Cutter Vacuum or Laser Blower motor
    static void air_evac_disable();        // Turn Off Cutter Vacuum or Laser Blower motor
    static void air_evac_toggle();         // Toggle Cutter Vacuum or Laser Blower motor
    static bool air_evac_state() {  // Get current state
      return (READ(AIR_EVACUATION_PIN) == AIR_EVACUATION_ACTIVE);
    }
  #endif

  #if ENABLED(AIR_ASSIST)
    static void air_assist_enable();         // Turn on air assist
    static void air_assist_disable();        // Turn off air assist
    static void air_assist_toggle();         // Toggle air assist
    static bool air_assist_state() {  // Get current state
      return (READ(AIR_ASSIST_PIN) == AIR_ASSIST_ACTIVE);
    }
  #endif

  //TG this is for Marlin mode LCD, not TFT touchscreen
  #if HAS_LCD_MENU
    static void enable_with_dir(const bool reverse) {
      isReady = true;
      const uint8_t ocr = TERN(SPINDLE_LASER_USE_PWM, upower_to_ocr(menuPower), SPINDLE_LASER_PWM_RES);   //TG 9/30/21 updated max OCR
      if (menuPower)
        power = ocr;
      else
        menuPower = cpwr_to_upwr(SPEED_POWER_STARTUP);
      unitPower = menuPower;
      set_reverse(reverse);
      set_enabled(true);
    }
    FORCE_INLINE static void enable_forward() { enable_with_dir(false); }
    FORCE_INLINE static void enable_reverse() { enable_with_dir(true); }
    FORCE_INLINE static void enable_same_dir() { enable_with_dir(is_reverse()); }

    #if ENABLED(SPINDLE_LASER_USE_PWM)
      static void update_from_mpower() {
        if (isReady) power = upower_to_ocr(menuPower);
        unitPower = menuPower;
      }
    #endif

    #if ENABLED(LASER_FEATURE)
      /**
       * Test fire the laser using the testPulse ms duration
       * Also fires with any PWM power that was previous set
       * If not set defaults to 80% power
       */
      static void test_fire_pulse() {
        TERN_(USE_BEEPER, buzzer.tone(30, 3000));
        enable_forward();                  // Turn Laser on (Spindle speak but same funct)
        delay(testPulse);                  // Delay for time set by user in pulse ms menu screen.
        disable();                         // Turn laser off
      }
    #endif

  #endif // HAS_LCD_MENU

  #if ENABLED(LASER_POWER_INLINE)
    /**
     * Inline power adds extra fields to the planner block
     * to handle laser power and scale to movement speed.
     */

    // Force disengage planner power control
    static void inline_disable() {
      isReady = false;
      unitPower = 0;
      planner.laser_inline.status.isPlanned = false;
      planner.laser_inline.status.isEnabled = false;
      planner.laser_inline.power = 0;
    }

    // Inline modes of all other functions; all enable planner inline power control
    static void set_inline_enabled(const bool enable) {
      if (enable)
        inline_power(255);
      else {
        isReady = false;
        unitPower = menuPower = 0;
        planner.laser_inline.status.isPlanned = false;
        TERN(SPINDLE_LASER_USE_PWM, inline_ocr_power, inline_power)(0);
      }
    }

    // Set the power for subsequent movement blocks
    static void inline_power(const cutter_power_t upwr) {
      unitPower = menuPower = upwr;
      #if ENABLED(SPINDLE_LASER_USE_PWM)
        #if ENABLED(SPEED_POWER_RELATIVE) && !CUTTER_UNIT_IS(RPM) // relative mode does not turn laser off at 0, except for RPM
          planner.laser_inline.status.isEnabled = true;
          planner.laser_inline.power = upower_to_ocr(upwr);
          isReady = true;
        #else
          inline_ocr_power(upower_to_ocr(upwr));
        #endif
      #else
        planner.laser_inline.status.isEnabled = enabled(upwr);
        planner.laser_inline.power = upwr;
        isReady = enabled(upwr);
      #endif
    }

    static void inline_direction(const bool) { /* never */ }

    #if ENABLED(SPINDLE_LASER_USE_PWM)
      static void inline_ocr_power(const uint8_t ocrpwr) {
        isReady = ocrpwr > 0;
        planner.laser_inline.status.isEnabled = ocrpwr > 0;
        planner.laser_inline.power = ocrpwr;
      }
    #endif
  #endif // LASER_POWER_INLINE

  static void kill() {
    TERN_(LASER_POWER_INLINE, inline_disable());
    disable();
  }
};

extern SpindleLaser cutter;
