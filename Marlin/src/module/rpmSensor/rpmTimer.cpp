/** //TG MODIFIED BY T.GIOIOSA
 * Based in Marlin/HAL/timers.h adapted for using LPC_TMR3 to count rpm pulses
 * T. Gioiosa 2/19/21
 * modified 12/24/22 for VFD
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

/**
 * Description:
 *
 * Timers for LPC1768
 */

#ifdef TARGET_LPC1768
  #include "../../inc/MarlinConfig.h" // needed for the next line #if ENABLED
  #if ENABLED(USE_RPM_SENSOR)
  
  #pragma region Includes ----------------------------------------
  #include "../../../src/lcd/marlinui.h"
  //#include "../../LPC1768/timers.h"
  #include "rpmTimer.h"
  #include "../../../feature/spindle_laser.h"
  #include "../temperature.h"
  #include "stdio.h"
  #include "pwm.h"
  //#include <src/module/rpmSensor/TG_I2CSlave.h>
  #pragma endregion // Includes

  #pragma region - TG 9/17/21 - choose config options below, comment out those not wanted
  // good resolution but longer measure time, typically 250ms to 1000msec lower limit=60RPM
  //#define FREQUENCY_COUNTER_MODE        
  // good resolution and faster, varies with RPM  2msec@30000RPM  600msec@100RPM  60sec@1RPM
  #define PERIOD_MEASURE_MODE             
  #pragma endregion

  #pragma region - Defines for PERIOD_MEASURE_MODE
  #ifdef PERIOD_MEASURE_MODE
    int limitFlag = 0;
    #define PERIOD_TMR_RATE 1000000             // for 1 us resolution
    #define TIMER_RES 1.0f/PERIOD_TMR_RATE      // in usec Depends on Timer PCLK and PR. Used to convert measured period to frequency. 
  #endif
  #pragma endregion

  #pragma region - //TG - Variables for Spindle/RPM sensor ******************************************************************
  extern SpindleLaser cutter;
  float measured_RPM = 0;
  uint16_t RPM_NO_SIGNAL = 0;        //TG 12/21/22 added
  uint16_t VFD_STATUS_COUNTER = 0;
  #define RPM_LOSS_TIMEOUT 2 * RPM_TICK_RATE   // if timer tick is 1ms then for 2s timeout would be 2*1000 = 2000
  #define VFD_STATUS_INTERVAL 3 * RPM_TICK_RATE // // time in ms to fetch and display status = 3*1000 = 3000ms
  //std::array<char,60> logMsg;      // a CPP style array, use str.Data() for pointer to actual characters
  char logMsg[60];
  #pragma endregion

  #pragma region - //TG - ***** Variables for PID speed control algorithm *************************************************
  #if SPINDLE_USE_PID
    float_t error = 0;
    uint16_t SP = 0;                      // setpoint (target to reach)
    uint16_t PV = 0;                      // process value (actual output)
    float proportional = 0;
    PIDController pid_RPM = {PID_KP,PID_KI,PID_KD,PID_TAU,PID_LIM_MIN,PID_LIM_MAX,PID_LIM_MIN_INT,PID_LIM_MAX_INT,SAMPLE_TIME_S};
    int8_t errorSign;
  #endif // SPINDLE_USE_PID
  #pragma endregion

  #pragma region - Timer Initialization for Timer2 or Timer3, handles either FREQUENCY_COUNTER_MODE or PERIOD_MEASURE_MODE for Timer3
    void RPM_timer_init(uint8_t timer_num){
      switch(timer_num)
    {
      case 2: 
        SBI(LPC_SC->PCONP, SBIT_TIMER2); 
        LPC_TIM2->PR = 0;  //Using lowest PR gives most accurate results
        LPC_SC->PCLKSEL1 |= (1<<12); //Set bits[13:12] = [01] to select PCLK_TIMER2 = CCLK i.e. 120Mhz in our case. 
        LPC_PINCON->PINSEL0 |= (1<<9) | (1<<8); //Set Bits[9:8] = [11] to Select CAP2.0 for P0.4
        LPC_TIM2->CTCR = 0x1; //Increment TC on rising edges of External Signal for CAP2.0  (P0_4)
        LPC_TIM2->CCR = 0x0;  //No Capture Register load on event and no int on event for CAP2.0 or CAP 2.1
        LPC_TIM2->TCR = 0x2; //Reset & Disable Timer2 Initially
        break;  // Power ON Timer 2
      case 3: 
        #ifdef FREQUENCY_COUNTER_MODE
        SBI(LPC_SC->PCONP, SBIT_TIMER3); 
        LPC_TIM3->PR = 0;  //Using lowest PR gives most accurate results
        LPC_SC->PCLKSEL1 |= (1<<14); //Set bits[15:14] = [01] to select PCLK_TIMER3 = CCLK i.e. 120Mhz in our case. 
        LPC_PINCON->PINSEL1 |= (1<<15) | (1<<14); //Set Bits[15:14] = [11] to Select CAP3.0 for P0.23
        LPC_TIM3->CTCR = 0x1; //Increment TC on rising edges of External Signal for CAP3.0  (P0_23)
        LPC_TIM3->CCR = 0x0;  //No Capture Register load on event and no int on event for CAP3.0 or CAP 3.1
        LPC_TIM3->TCR = 0x2; //Reset & Disable Timer3 Initially
        #endif
        #ifdef PERIOD_MEASURE_MODE
        SBI(LPC_SC->PCONP, SBIT_TIMER3); //Power-Up Timer3 module. It is disabled by default.
        LPC_TIM3->PR = F_CPU/(PERIOD_TMR_RATE - 1);    // set prescalar for desired PERIOD_TMR_RATE
        LPC_TIM3->PC = F_CPU/(PERIOD_TMR_RATE - 1);
        LPC_SC->PCLKSEL1 |= (1<<14); //Set bits[15:14] = [01] to select PCLK_TIMER3 = CCLK i.e. 120Mhz in our case. 
        LPC_PINCON->PINSEL1 |= (1<<15) | (1<<14); //Set Bits[15:14] = [11] to Select CAP3.0 for P0.23
        LPC_TIM3->CTCR = 0x0; //Timer Mode clocked by PreScale Counter
        LPC_TIM3->CCR = (1<<0) | (1<<2); //Capture on Rising Edge(0->1) and generate an interrupt for CAP3.0
        LPC_TIM3->TCR = 0x2; //Reset & Disable Timer3 Initially
        RPM_timer_enable_interrupt(MF_TIMER_RPM);
        RPM_timer_start(MF_TIMER_RPM);
        #endif

        break;  // Power ON Timer 3
      default: 
        break;
    }
    }
  #pragma endregion

  // the HAL_TEMP_TIMER_ISR calls here every 1ms, when 1 second has elapsed we update the measured_RPM.
  // execution is under 100us with PID running (longer if processing M3/M4 gcodes from TFT speed changes)
  // RPM_tick and PID code             <100us  
  // Logging PV strings to terminal    ~3000us
  // FAST_RPM_REPORTING on             ~360us to 620us varies
  // all turned on takes about 3760us, so HAL_TEMP_TIMER_ISR is modified to every 4ms if Logging or FAST_RPM_REPORTING is on.
  #pragma region - RPM_tick code called by HAL_TEMP_ISR in temperature.c, RPM_SAMPLE_TIME sets interval to perform action
  // In FREQUENCY_COUNTER_MODE, RPM measurement/measured_RPM and PID sample time are handled at RPM_SAMPLE_TIME interval
  // In PERIOD_MEASURE_MODE, RPM measurement/measured_RPM are at the Timer 3 ISR rate, PID action remains at the RPM_SAMPLE_TIME interval
  // The Timer 3 ISR rate is much faster than FREQUENCY_COUNTER_MODE and runs at the RPM sensor rate (2ms to 600ms). 
    void RPM_tick(void) {
      static uint16_t gate_counter = 0;
      static uint32_t pulse_count = 0;
      static uint8_t smode = 0;
      static uint8_t bytecount;
      static uint8_t I2C_timer;

      //WRITE(P2_12,1);     //TG - ***** for scope measurement normally commented out  
      if(gate_counter++ == 0){                        // when gate interval counter is at reset (0)
        #ifdef FREQUENCY_COUNTER_MODE
        RPM_timer_start(MF_TIMER_RPM);             // start counting rising edge of pulses on CAP3.0 (P0_23)
        #endif
      } 
      else if(gate_counter > ((RPM_SAMPLE_TIME*RPM_TICK_RATE/1000) - 1))   // when gate interval has elapsed
      {
        //WRITE(P4_28,1);    //TG - ***** for scope measurement normally commented out
        // Measure frequency from number of pulses counted in one gate interval 
        #ifdef FREQUENCY_COUNTER_MODE
        // calc measured_RPM and update the RPM from number of pulses counted in one gate interval
        // if in PERIOD_MEASURE_MODE the measured_RPM variable is set by the Timer3 ISR code
        //WRITE(P2_12,1);
        RPM_timer_stop(MF_TIMER_RPM);        // stop the timer
        pulse_count = RPM_timer_get_count(); // get the pulses counted over gate interval
        RPM_timer_reset(MF_TIMER_RPM);       // reset the timer counter
        measured_RPM = 60 * (double)pulse_count * RPM_TICK_RATE / RPM_SAMPLE_TIME;
        cutter.ACTUAL_RPM = measured_RPM;    //TG 12/20/22 added because this is what TFT displays
        #endif

        //sprintf(str, "Speed: %5.2f  Gate: %u", measured_RPM, gate_counter); 
        //send_M117_msg(str);
        gate_counter = 0;                     // reset the gate interval counter

        //TG 2/21/21 don't wait for autotemp reporting cause it is every 3 sec by default, instead send a
        //speed msg "S0:ttttt/aaaaa" every gate interval for faster updates(gate is currently 500 msec)
        //when FAST_RPM_REPORTING is defined in Configuration_adv.h
        #ifdef FAST_RPM_REPORTING
          #ifdef SERIAL_FLOAT_PRECISION
            #define SFP _MIN(SERIAL_FLOAT_PRECISION, 2)
          #else
            #define SFP 2
          #endif
          //WRITE(P2_12,1);     //TG - ***** for scope measurement normally commented out
          //TG one-line method
          sprintf(logMsg," S0:%d / %d\n", (int)Temperature::spindle_speed[0].target, (int)measured_RPM);
          SERIAL_ECHOPGM(logMsg);

          //TG original method
          //SERIAL_CHAR(' ', 'S', '0',':');
          //SERIAL_PRINT(Temperature::spindle_speed[0].target, SFP); //TG 5/25/21 added (double) to fix errors
          //SERIAL_ECHOPGM(" /");
          // SERIAL_PRINT(measured_RPM, SFP);
          //SERIAL_EOL();
          //WRITE(P2_12,0);     //TG - ***** for scope measurement normally commented out
        #endif
    
        // PID speed correction if enabled
        // perform speed control only if spindle is commanded on and ready, and spindle_use_pid is set true, skip otherwise
        #if SPINDLE_USE_PID
        if(SpindleLaser::isReady && SpindleLaser::enabled() && cutter.spindle_use_pid==true)
        {
          // Marlin's CUTTER_POWER_UNIT is what determines unitPower and menuPower units(not the settings at the TFT).
          // The TFT settings affect TFT displayed units and what units to expect from Marlin to display them properly!
          // Let's convert any Marlin units to RPM for better accuracy and precision.
          PV = MarlinUnitsToRPM(measured_RPM);
          SP = MarlinUnitsToRPM(SpindleLaser::menuPower);
          errorSign = SP>PV ? 1 : SP<PV ? -1 : 0;
          error = SP - PV;
          //TG - 9/21/21 copy temp_bed.pid values to pid_RPM struct values (used in the computePID_Classic() call), 
          // since temp_bed.pid values can be user-set with M304 and stored/recalled in EEPROM
          copy_PID_constants(); 
          
          // code for logging values to terminal during test or development to study waveforms (adds 300usec to execution time)
         // #ifdef PID_WAVEFORM_LOGGING
            if(pid_RPM.newTarget==1)            // if first cycle after a new setpoint, print these info values one time
            {   
              //str.fill('\0');
              sprintf(logMsg,"Kp:%3.2f, Ki:%3.2f, Kd:%3.2f, SP:%u, T:%2.2f \n",pid_RPM.Kp,pid_RPM.Ki,pid_RPM.Kd,SP,pid_RPM.T);
              SERIAL_ECHO(logMsg);
              
              //str.fill('\0');
              sprintf(logMsg," PV:%u :P1:%d :E:%5.0f \n",SP, SpindleLaser::power,error);  // to indicate a start point    
              SERIAL_ECHO(logMsg);
              //str.fill('\0');
              sprintf(logMsg," PR:%5.0f :I:%5.0f :P2:%3.0f :KP:%1.2f :KI:%1.2f\n",proportional,pid_RPM.integrator,pid_RPM.out,pid_RPM.Kp,pid_RPM.KpFactor);            // to indicate a start point        
              SERIAL_ECHO(logMsg);
            }
            else{
              //str.fill('\0');
              sprintf(logMsg," PV:%u :P1:%3.0f :E:%5.0f \n",PV, pid_RPM.prevPower,error);   // to post current data        
              SERIAL_ECHO(logMsg);
              //str.fill('\0');
              sprintf(logMsg," PR:%5.0f :I:%5.0f :P2:%3.0f :KP:%1.2f :KI:%1.2f\n",proportional,pid_RPM.integrator,pid_RPM.out,pid_RPM.Kp,pid_RPM.KpFactor);             // to post current data        
              SERIAL_ECHO(logMsg);
            }   
          //#endif

          computePID_Classic();               // do the speed control work
          //computePID_WindowMethod();
           
          pid_RPM.newTarget=0;                    // clear flag so log values only printed on first entry after setpoint change
        } // if(SpindleLaser::isReady
        else
        { // reset all PID variables if spindle is off
          pid_RPM.integrator = 0;
          pid_RPM.prevError = 0;
          pid_RPM.prevMeasurement = 0;
          pid_RPM.prevPower = 0;
          pid_RPM.out=0;
          pid_RPM.newTarget = 1;
          measured_RPM = 0;
        }
        //WRITE(P4_28,0);    //TG - ***** for scope measurement normally commented out
        #endif // SPINDLE_USE_PID
      
      } // end if(gate_counter > (RPM_MEAS_TIME - 1))

      //TG 12/21/22 inc the no-rpm counter, if it doesn't get cleared by Timer3 ISR before reaching RPM_LOSS_TIMEOUT then we 
      //can assume rpm has dropped to less than the 60/(timeout period) so consider motor stopped, and force RPM to zero.
      if(RPM_NO_SIGNAL++ > RPM_LOSS_TIMEOUT)
      {
        RPM_NO_SIGNAL = 0;
        cutter.ACTUAL_RPM = measured_RPM = 0;         //TG 12/21/22 added
      }

    //WRITE(P2_12,0);     //TG - ***** for scope measurement normally commented out      
    } // void RPM_tick

  #pragma endregion // RPM_tick code called by HAL_TEMP_ISR

  #pragma region - PID routines
    #if SPINDLE_USE_PID
    // Classic PID discrete-time calculation using pid_RPM structure, works best when M3/M4 speed command starts out with an initial
    // minimum ocr speed of 25% SP (if it was currently zero) and leaving it at previous ocr if non-zero. Good values for K-factors are:
    //   T = 0.25s          T = 0.50s         T = 1.00s
    // PID_KP  0.16f      PID_KP  0.17f     PID_KP  0.10f
    // PID_KI  0.28f      PID_KI  0.23f     PID_KI  0.18f
    // PID_KD  0.00f      PID_KD  0.00f     PID_KD  0.00f
    void computePID_Classic(void) 
  {
    //TG - optional compensate error scale dynamically according to setpoint range, corrects for non-symetric overshoots/undershoots
        //if (errorSign < 0)
        //     error = error * (0.60f + SP/50000.0f); // decrease falling error rate with SP
        //if (errorSign > 0)
        //    error = error * (0.55f + SP/20000.0f);  // increase rising error rate with SP
    
    proportional = pid_RPM.Kp * error;                                                                      // Proprtional
    
    // alternate averaged integral (error + prevError)/2
    pid_RPM.integrator = pid_RPM.integrator + pid_RPM.Ki * pid_RPM.T * (error + pid_RPM.prevError);         // Intgeral
        //pid.integrator = pid.integrator + pid.Ki * pid.T * error;                               // Intgeral
    /* Anti-wind-up via integrator clamping */
    LIMIT(pid_RPM.integrator,pid_RPM.limMinInt,pid_RPM.limMaxInt);
                                    
    /* Note: derivative on measurement, therefore minus sign in front of equation! */
    pid_RPM.differentiator = -pid_RPM.Kd * (PV - pid_RPM.prevMeasurement) / pid_RPM.T;                      // Derivative
    // alternate band-limited-derivative  (low pass filtered)
    //pid_RPM.differentiator = -(2.0f * pid_RPM.Kd * (PV - pid_RPM.prevMeasurement)
    //                     + (2.0f * pid_RPM.tau - pid_RPM.T) * pid_RPM.differentiator) / (2.0f * pid_RPM.tau + pid_RPM.T);
    
  
    /* Compute output and apply limits */
    pid_RPM.out = proportional + pid_RPM.integrator + pid_RPM.differentiator;	
    LIMIT(pid_RPM.out,pid_RPM.limMin,pid_RPM.limMax);
    
    /* Store error and measurement for later use */
    pid_RPM.prevError       = error;
    pid_RPM.prevMeasurement = PV;
    pid_RPM.prevPower = SpindleLaser::power;    // only needed for logging data during testing

    pid_RPM.out = convertUnits(pid_RPM.out, CRPM, CPWM);
    LIMIT(pid_RPM.out,1,SPINDLE_LASER_PWM_RES);
        
    // pwr unit is always PWM range  
    // tried bypassing wrappers with apply_power() which took from 38 to 68us, and calling direct
    // to LPC176x::pwm_write_ratio(), but it only reduced to 48us, so not worth it.
    SpindleLaser::apply_power(pid_RPM.out);

  }


    //TG - 9/28/21 copies temp.bed PID constants to pid_RPM struct constants
    void copy_PID_constants(void){
      pid_RPM.Kp =  Temperature::temp_bed.pid.Kp;
      pid_RPM.Ki =  unscalePID_i(Temperature::temp_bed.pid.Ki);
      pid_RPM.Kd =  unscalePID_d(Temperature::temp_bed.pid.Kd);

      // scale according to setpoint range, corrects for non-symetric overshoots/undershoots
      // try some compensation to the K factors based on where the motor is heading
      if (errorSign<0){         // for speed going down
        if(SP>19999){           // needs more than standard gain 
         pid_RPM.Kp *= 1.9;
         pid_RPM.Ki *= 1.9; 
        }
        else if (SP>14999){     // needs standard gain
          pid_RPM.Kp *= 1.0;
          pid_RPM.Ki *= 1.0;
        }
        else{                   // below 15000 wants less than standard gain
          pid_RPM.Kp *= 0.55;
          pid_RPM.Ki *= 0.55;
        }
      }

      if (errorSign>0){         // for speed going up
        // everything below 15000 has standard gain
        if(SP>15000){           // needs a little extra gain above 15000
          pid_RPM.Kp *= 1.3;
          pid_RPM.Ki *= 1.3;
        }
        else if(SP>20000){      // needs a lot more gain above 20000
          pid_RPM.Kp *= 1.8;
          pid_RPM.Ki *= 1.8;
        }                       
      }
      return;

      //#define EXP_TEST_ENABLED
      //TG - 9/30/21 This is the experimental hack section to fine tune the PID to the DeWalt 660 (other motors will need different tweaks)
      #if ENABLED(EXP_TEST_ENABLED)

      //if (errorSign<0 && SP<15000 && abs(error)<11000){ // for negative errors(decreasing speed needed) only, the DeWalt seems to undershoot the
      //  pid_RPM.Kp *= 0.50;         // setpoint more when the setpoint is below mid speed (150000)
      //  pid_RPM.Ki *= 0.50;         // so we decrease the gain factors till we get acceptable undershoot
      //  pid_RPM.Kd *= 0.50;
      //}
      //if (SP>15000){                // for positive errors(increasing speed needed), the DeWalt seems to undershoot the
      //  pid_RPM.Kp *= 1.70;         // setpoint more when the setpoint is above mid-speed (15000)
      //  pid_RPM.Ki *= 1.70;         // so we increase the gain factors, it needs this increased gain in the negative direction
      //  pid_RPM.Kd *= 1.70;         // also till below mid-speed (15000), so we don't test for errorSign here
      //}

      //TG - 10/1/21  This compensation method applies a variable factor based on size of error, to Kp,Ki,Kd as follows:
      // if negative error, KpFactor is scaled with error from 225% to 15% (equation mx + b), but no less than 15%
      // the gain needs to be reduced as the error diminishes to avoid severe undershoot when target setpoint goes down
      if (errorSign<0)  {
      pid_RPM.KpFactor = (7.5e-5 * abs(error) + .15);                       // get factor according to size of error and equation
      pid_RPM.KpFactor = pid_RPM.KpFactor<0.15 ? 0.15 : pid_RPM.KpFactor;   // limit low end
      
      //NOT SURE ABOUT THIS YET - BUT ANY DROPS IN SPEED WHEN SP IS ABOVE 10000 ARE VERY SLOW!!
      //This speeds up Kfactor according to target SP, higher SP needs faster speed down.
      if(SP>24000)                             
        pid_RPM.KpFactor = pid_RPM.KpFactor * 7.0;
      if(SP>18000)                             
        pid_RPM.KpFactor = pid_RPM.KpFactor * 5.0;                    
      else if(SP>12000)               
        pid_RPM.KpFactor = pid_RPM.KpFactor * 1.8; 
      else if(SP>8000)               
        pid_RPM.KpFactor = pid_RPM.KpFactor * 1.0;       
      else 
        pid_RPM.KpFactor = pid_RPM.KpFactor * 0.65; 

      pid_RPM.Kp *= pid_RPM.KpFactor;                       // apply factor
      pid_RPM.Ki *= pid_RPM.KpFactor;
      pid_RPM.Kd *= pid_RPM.KpFactor;
    }

      // if positive error, KpFactor is scaled with error from 351% to 85% (equation mx + b), but no less than 100%
      // the gain needs to be increased for large positive error to avoid severe undershoot when target setpoint goes up
      if (errorSign>0)  {
      pid_RPM.KpFactor = (9.5e-5 * abs(error) + 0.9);      // get factor according to size of error and equation
      pid_RPM.KpFactor = pid_RPM.KpFactor<1.0 ? 1.0 : pid_RPM.KpFactor;     // limit low end
      
      //NOT SURE ABOUT THIS YET - BUT ANY INCREASES IN SPEED WHEN SP IS ABOVE 12000 ARE VERY SLOW!
      //TRY TO SPEED IT UP?? 
      if(SP>20000 && abs(error) < 12000)                             
        pid_RPM.KpFactor = pid_RPM.KpFactor * 1.6;
      //if(SP>18000)                                                     
      //  pid_RPM.KpFactor = pid_RPM.KpFactor * 1.02;                    
      //else if(SP>12000)              
      //  pid_RPM.KpFactor = pid_RPM.KpFactor * 1.01; 
      //else if(SP>8000)               
      //  pid_RPM.KpFactor = pid_RPM.KpFactor * 0.90;       
      //else
      //  pid_RPM.KpFactor = pid_RPM.KpFactor * 0.80;

     pid_RPM.Kp *= pid_RPM.KpFactor;                      // apply factor 
     pid_RPM.Ki *= pid_RPM.KpFactor;
     pid_RPM.Kd *= pid_RPM.KpFactor; 
    }    
      #endif // #if ENABLED(EXP_TEST_ENABLED)


    }

    // My original method check error from target and decide how to trim the PWM (ocr), SpindleLaser::power is always in PWM (ocr) units 
    void computePID_WindowMethod(void)
    {
    uint16_t hysteresis = 150;
    int16_t ocr_step;         // must handle +/- need signed int
    uint16_t OPR;
    uint16_t new_OCR=0;
    static uint16_t integral;
  
    if (pid_RPM.newTarget == 1)
      integral = 0;
  
    OPR = convertUnits(SpindleLaser::power, CPWM, CRPM);              // get current power in RPM units
    
    integral += error/8;
    ocr_step = error /18 + integral;                                  // calc step, it will be + or - as needed
    
    if (abs(PV-SP) > hysteresis)                                      // if outside of band, compensate                                      
      new_OCR = ocr_step;
    else
      new_OCR =  OPR;                                                 // otherwise just hold steady
    
    new_OCR = convertUnits(new_OCR, CRPM, CPWM);
    LIMIT(new_OCR,1,SPINDLE_LASER_PWM_RES);
    SpindleLaser::apply_power(new_OCR);     // change pwr unit from RPM to PWM range
    }
    #endif // SPINDLE_USE_PID
  #pragma endregion  
  
  #pragma region Timer3 ISR handler for PERIOD_MEASURE_MODE
    // PERIOD_MEASURE_MODE, capture timer count every interrupt (RPM sensor pulse), subtract previous count to get period in usec
    #ifdef PERIOD_MEASURE_MODE
    //void TIMER3_IRQHandler(void)
    HAL_RPM_TIMER_ISR()
    {
    	unsigned int current = 0 ;
      unsigned int period = 0;
      static unsigned int previous = 0;

      LPC_TIM3->IR |= (1<<4);                         //Clear Interrupt Flag for Capture Channel 0
    	current = LPC_TIM3->CR0;                  

      if(current < previous)                          //TC has overflowed, so adjust for it
    		period = 0xFFFFFFFF + current - previous;
    	else
    		period = current - previous;                  //otherwise get delta from last interrupt
      
      // filter out any erroneous period value which may occur from an invalid previous value from last time spindle stopped (ISR stopped)
      if(period>2000U && period<60000000U){           // only update measured_RPM if period is reasonable 60,000,000(1rpm) to 2,000(30000rpm)
        measured_RPM = (60.0/(period * TIMER_RES));   // Convert to frequency, 1us is Timer resolution
        LIMIT(measured_RPM,0,30000);
        cutter.ACTUAL_RPM = measured_RPM;             //TG 12/20/22 added because this is what TFT displays
      }
      previous = current;                            //LPC_TIM3->CR0, save for next interrupt calculation
      RPM_NO_SIGNAL =0;                              //TG 12/21/22 keep this counter at zero when we have valid RPM signal
    }	
    #endif // PERIOD_MEASURE_MODE
  #pragma endregion // Timer3 ISR handler for PERIOD_MEASURE_MODE
  
  #pragma region - Miscellaneous function for units conversion
  uint16_t MarlinUnitsToRPM(float_t val)
  {
    return  CUTTER_UNIT_IS(PWM255) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / SPINDLE_LASER_PWM_RES  + 0.5f:
            CUTTER_UNIT_IS(PERCENT) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / 100.0f  + 0.5f:
            CUTTER_UNIT_IS(SERVO) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / 180  + 0.5f:
            val;
  }

  //TG - 9/3/21 new function
  // convert input value from one unit to another unit. Works best when both use the same unit.
  uint16_t convertUnits(float_t val, CTYPE unit_in, CTYPE unit_out)
  {
    uint16_t cval;
    
    switch(unit_out)
    {
      case CPWM:  //output is PWM
        cval = (unit_in == CPCT) ? val * SPINDLE_LASER_PWM_RES / 100.0f  + 0.5f :
               (unit_in == CRPM) ? val * SPINDLE_LASER_PWM_RES / (SPEED_POWER_MAX-SPEED_POWER_MIN) + 0.5f:
               (unit_in == CSERVO) ? val: 
                val;
        break;

      case CPCT:  //targeting Marlin PCT mode
        cval = (unit_in == CPWM) ? val * 100.0f / SPINDLE_LASER_PWM_RES  + 0.5f:
               (unit_in == CRPM) ? val * 100.0f / (SPEED_POWER_MAX-SPEED_POWER_MIN) + 0.5f:
               (unit_in == CSERVO) ? val * 100.0f / 180  + 0.5f:
                val;
        break;

      case CRPM:  //targeting Marlin RPM mode
        cval = (unit_in == CPWM) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / SPINDLE_LASER_PWM_RES  + 0.5f:
               (unit_in == CPCT) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / 100.0f  + 0.5f:
               (unit_in == CSERVO) ? val * (SPEED_POWER_MAX-SPEED_POWER_MIN) / 180  + 0.5f:
                val;
        break;

      case CSERVO:  //targeting Marlin RPM mode
        cval = (unit_in == CPWM) ? val:
               (unit_in == CPCT) ? val * 180 / 100.0f + 0.5f:
               (unit_in == CRPM) ? val * 180 / (SPEED_POWER_MAX-SPEED_POWER_MIN)  + 0.5f:
                val;
        break;

      default:
        cval = val;
       break;
    }
    return cval;
  }

  void send_M117_msg(char * msg){
    ui.set_status(msg);
  }

  #pragma endregion

  #endif // #if ENABLED(USE_RPM_SENSOR)
#endif // TARGET_LPC1768
