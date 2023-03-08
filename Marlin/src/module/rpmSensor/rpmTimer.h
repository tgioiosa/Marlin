/** //TG MODIFIED BY T.GIOIOSA
 * Based in Marlin/HAL/timers.h adapted for using LPC_TMR3 to count rpm pulses with 2 different methods
 * Also adds PID Speed control algorithm
 * T. Gioiosa 2/19/21, 9/16/21
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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
 * HAL For LPC1768
 */

#if ENABLED(USE_RPM_SENSOR)

#include <stdint.h>
#include "../../core/macros.h"
#include <LPC17xx.h>

#pragma region  - Bit position names for use in SBI, CBI, or other bit manipulations
#define SBIT_TIMER2 22  // Timer 2 power/clock control bit position in Power Control for Peripherals Register (PCONP)
#define SBIT_TIMER3 23  // Timer 3 power/clock control bit position in Power Control for Peripherals Register (PCONP)
#define SBIT_CNTEN 0    // Counter Enable control bit position in Timer Control Register (TCR)
#define SBIT_MR0 0      // Timer Interrupt Register flag bit position for match channel 0
#define SBIT_MR1 1      // Timer Interrupt Register flag bit position for match channel 1

// These are for the Match Control register bit positions only!
#define SBIT_MR0I  0 // Timer 0 Interrupt when TC matches MR0
#define SBIT_MR0R  1 // Timer 0 Reset TC on Match
#define SBIT_MR0S  2 // Timer 0 Stop TC and PC on Match
#define SBIT_MR1I  3
#define SBIT_MR1R  4
#define SBIT_MR1S  5
#define SBIT_MR2I  6
#define SBIT_MR2R  7
#define SBIT_MR2S  8
#define SBIT_MR3I  9
#define SBIT_MR3R 10
#define SBIT_MR3S 11
#pragma endregion

#pragma region  - Defines to support the PID calculations
//TG - these vars supports the PID calculations
extern uint16_t hysteresis;
extern uint16_t new_power;
extern float_t error;
extern uint16_t SP;                      
extern uint16_t PV;                      
extern float_t proportional;
// struct definition for PID routine
typedef struct {
	float Kp;                   /* Controller gains */
	float Ki;
	float Kd;
	float tau;                  /* Derivative low-pass filter time constant */
	float limMin;               /* Output limits */
	float limMax;
	float limMinInt;            /* Integrator limits */
	float limMaxInt;
	float T;                    /* Sample time (in seconds) */
	float integrator;           /* Controller "memory" */
	float prevError;			      /* Required for integrator */
	float differentiator;
	float prevMeasurement;		  /* Required for differentiator */
	float prevPower;
  float out;                  /* Controller output */
  float KpFactor;             /* experimental dynamic scale factor */
  uint8_t newTarget;          /* flag indicates when a new Setpoint has been set */
} PIDController;

/* Controller parameters */             //   T = 0.25s          T = 0.50s         T = 1.00s
#define PID_KP  0.30f                   // PID_KP  0.16f      PID_KP  0.17f     PID_KP  0.10f                   
#define PID_KI  0.35f                   // PID_KI  0.28f      PID_KI  0.23f     PID_KI  0.18f
#define PID_KD  0.00f                   // PID_KD  0.00f      PID_KD  0.00f     PID_KD  0.00f
#define PID_TAU 0.02f
#define PID_LIM_MIN -SPEED_POWER_MAX
#define PID_LIM_MAX  SPEED_POWER_MAX
#define PID_LIM_MIN_INT -SPEED_POWER_MAX
#define PID_LIM_MAX_INT  SPEED_POWER_MAX
#define SAMPLE_TIME_S 0.001f * RPM_SAMPLE_TIME       // sample rate in ms      
extern PIDController pid_RPM;

typedef enum  {       //TG 9/3/21 added to support unit conversions for RPM routines
  CPWM = 0,
  CPCT,
  CRPM,  
  CSERVO,
  CONVERT_PWR_SIZE
} CTYPE;
#pragma endregion

#pragma region  - Defines to Setup Timer 3 for RPM use
//TG - 9/17/21  Marlin HAL has already assigned PWM to Timer 3, but we need Timer3 for RPM sensing,
//     so we will reassign PWM to Timer 2 and define Timer 3 as the RPM timer
#ifdef MF_TIMER_PWM
  #undef MF_TIMER_PWM
  #define MF_TIMER_PWM   2   //TG Re-define Index for PWM to Timer 2 since HAL has already defined it as Timer 3 (use modified SoftwarePWM.cpp !!)
#endif
#define MF_TIMER_RPM  3      //TG We assign RPM functions to Timer 3
typedef uint32_t rpm_timer_t;
//TG 9/1/21 The following provide functionality for timer 3 use as RPM_Timer, which are not found in HAL timer.h
#define ENABLE_RPM_INTERRUPT() RPM_timer_enable_interrupt(MF_TIMER_RPM)
#define DISABLE_RPM_INTERRUPT() RPM_timer_disable_interrupt(MF_TIMER_RPM)
#define RPM_ISR_ENABLED() RPM_timer_interrupt_enabled(MF_TIMER_RPM)
#ifndef HAL_RPM_TIMER_ISR
  #define HAL_RPM_TIMER_ISR() _HAL_TIMER_ISR(MF_TIMER_RPM)   //TG 9/1/21 fixed this
#endif
#define RPM_TIMER_PTR _HAL_TIMER(MF_TIMER_RPM)
#pragma endregion

#pragma region - Defines For RPM measurement
// For RPM measurement using the gate method, 1sec gate width gives best compromise bewteen quick response and accuracy
// Here are the numbers for other settings
//  GATE_WIDTH    Response Time   Accuracy
//   500ms        Very good       +/- 120 rpm  poor
//  1000ms        good            +/- 60 rpm   fair
//  2000ms        fair            +/- 30 rpm   good
//  4000ms        poor            +/- 15 rpm   very good
// RPM sensor pulses are counted during RPM_SAMPLE_TIME/RPM_TICK_RATE in seconds

//#define RPM_TICK_USES_TEMP_TIMER_FREQUENCY_2    //TG - enable this if you want RPM_TICK to occur faster than TEMP_TIMER rate of 1ms
#ifndef RPM_TICK_USES_TEMP_TIMER_FREQUENCY_2
  #define RPM_SAMPLE_TIME         250           // Interval in msec to process PID and send speed updates to TFT
  #define RPM_TICK_RATE  TEMP_TIMER_FREQUENCY   // RPM_tick Rate pps = TEMP_TIMER_RATE of 1000 pps, ISR every 1ms)
#else
  #define RPM_SAMPLE_TIME         250           // Interval in msec to process PID and send speed updates to TFT
  #define TEMP_TIMER_FREQUENCY_2 2000           // a frequency higher than TEMP_TIMER_FREQUENCY, using MR1 of TEMP_TIMER
  #define RPM_TICK_RATE  TEMP_TIMER_FREQUENCY_2 // RPM_tick Rate pps = TEMP_TIMER_RATE of 1000 pps, ISR every 1ms)
#endif
#pragma endregion

#pragma region - Public functions

extern float measured_RPM;
void RPM_timer_init(uint8_t timer_num);
void RPM_tick(void);
uint16_t convertUnits(float_t val, CTYPE unit_in, CTYPE unit_out);
uint16_t MarlinUnitsToRPM(float_t val);
void send_M117_msg(char * msg);
void computePID_Classic(void);
void computePID_WindowMethod(void);
void copy_PID_constants(void);
//void TIMER3_IRQHandler(void);

#pragma endregion


#pragma region - RPM timer start, stop, reset, setcompare, getcompare, getcount, enable/disable interrupts, interruptenabled?, isr_prologue
FORCE_INLINE static void RPM_timer_start(const uint8_t timer_num) {
  if(timer_num == 2)
    LPC_TIM2->TCR = 0x1; // Counter Start
  else if(timer_num == 3)
    LPC_TIM3->TCR = 0x1; // Counter Start
}

FORCE_INLINE static void RPM_timer_stop(const uint8_t timer_num) {
  if(timer_num == 2)
    LPC_TIM2->TCR = 0x0; // Counter Stop
  else if(timer_num == 3) 
    LPC_TIM3->TCR = 0x0; // Counter Stop
}

FORCE_INLINE static void RPM_timer_reset(const uint8_t timer_num) {
  if(timer_num == 2)
    LPC_TIM2->TCR = 0x2; // Counter Stop
  else if(timer_num == 3) 
    LPC_TIM3->TCR = 0x2; // Counter Stop
}

FORCE_INLINE static void RPM_timer_set_compare(const rpm_timer_t compare) {
   RPM_TIMER_PTR->MR0 = compare;  // RPM Timer Match Register 0
}

FORCE_INLINE static rpm_timer_t RPM_timer_get_compare(void) {
  return RPM_TIMER_PTR->MR0;
}

FORCE_INLINE static rpm_timer_t RPM_timer_get_count(void) {
  return RPM_TIMER_PTR->TC; //    Rpm Timer Count
}

//TG these duplicate the same functions as in Marlin\src\HAL\LPC1768\timers.h, but for RPM_timer use
FORCE_INLINE static void RPM_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: break;
    case 1: break;
    case 2: NVIC_EnableIRQ(TIMER2_IRQn); break; // Enable interrupt handler
    case 3: NVIC_EnableIRQ(TIMER3_IRQn); break; // Enable interrupt handler
  }
}

FORCE_INLINE static void RPM_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: break;
    case 1: break;
    case 2: NVIC_DisableIRQ(TIMER2_IRQn); break; // Disable interrupt handler
    case 3: NVIC_DisableIRQ(TIMER3_IRQn); break; // Disable interrupt handler
  }

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

FORCE_INLINE static bool RPM_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return 0;
    case 1: return 0;
    case 2: return NVIC_GetEnableIRQ(TIMER2_IRQn); // Check if interrupt is enabled or not
    case 3: return NVIC_GetEnableIRQ(TIMER3_IRQn); // Check if interrupt is enabled or not
  }
  return false;
}

FORCE_INLINE static void RPM_timer_isr_prologue(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: break;
    case 1: break;
    case 2: 
    case 3: SBI(RPM_TIMER_PTR->IR, SBIT_CNTEN); break;    //TG SBI-set bit at position SBIT_CNTEN(=0) in IR register
  }
}

#define HAL_RPM_timer_isr_epilogue(TIMER_NUM)
#pragma endregion

#pragma region - Other miscellaneous functions
// if you want RPM_TICK to occur faster than TEMP_TIMER rate of 1ms
#ifdef RPM_TICK_USES_TEMP_TIMER_FREQUENCY_2
  FORCE_INLINE static void setupFastRPMTickTimer(void)
  {
  NVIC_DisableIRQ(TIMER1_IRQn);
  LPC_TIM1->TCR = 0x0;                                              //TG pause timer to make change
  LPC_TIM1->MCR = _BV(SBIT_MR0I) | _BV(SBIT_MR0R) | _BV(SBIT_MR1I); // add Int on MR1 match to MR0 match and reset on MR0 
  LPC_TIM1->TCR = _BV(SBIT_CNTEN); // Counter Enable
  TEMP_TIMER_PTR->MR1 = uint32_t(TEMP_TIMER_RATE) / RPM_TICK_RATE;; // set Match INT1 ISR to occur at TEMP_TIMER_FREQUENCY_2
  NVIC_EnableIRQ(TIMER1_IRQn);
  }
#endif

#pragma endregion

#endif // #if ENABLED(USE_RPM_SENSOR)
