/** //TG MODIFIED BY T.GIOIOSA  - adds many M79xx codes for Spindle control and VFD/AVR support
 * original 10/3/21
 * updated  12/24/22, 2/16/23
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * M codes for data exchange between Marlin and TFT35 screen over UART
 * The direction is from the TFT point of view
 * 
 *  gcode   function            TFT SEND to Marlin         Marlin RETURN to TFT       Description                                             If CMD issued from REMOTE USB host
 * -------  --------------    ---------------------      ------------------------     ------------------------------------                    -------------------------------------------------
 * M7900    AVRBlockInfo(-PID) use M7900 F....                    M7900 R             receive/send AVRInfoBlock.PIDFLAG, 
 *                                                                                    AVRInfoBlock.Reset_Flag, AVRInfoBlock.Display_Page, 
 *                                                                                    AVRInfoBlock.PID_Speed,  AVRInfoBlock.Update_EEPROM,
 *                                                                                    AVRInfoBlock.EE_chksum, AVRInfoBlock.dummy_pad_byte     *will echo to REMOTE SERIAL
 * M7979    PID flag           use M7979 Sx                       M7900 (blockinfo)   spindle_use_pid flag, pid on/off                        *also echoes to REMOTE SERIAL
 * M7980    RESET AVR flag     use M7980                          none                send reset AVR cmd to Marlin                            *will reset the AVR, echo ok only
 * M7981    PID Kp,Ki,Kd       use M7981 Px Ix Dx                 M7981 R             receive/send P,I,D constants Kp, Ki, Kd                 *also echoes to REMOTE SERIAL
 * M7982    AVR Display Page   use M7982 Px                       M7900 (blockinfo)   send AVR LCD display page # to Marlin                   *will change page, echo ok only
 * M7983    AVR PID speed      use M7983 Sx (0, 1, 2)             M7900 (blockinfo)   change the selected AVR PID speed                       *will change preset, echo ok only
 * M7984    AVR PID Reload     use M7984 Sx (1=current 2=default) M7900 (blockinfo)   reload a pid speed - TFT never needs to read back       *will reload, echo ok only
 * M7985    Vacuum Enable      happens via M42 P122               see M42()           Vacuum Enable state changed in Marlin                   *no response (must use M42 cmd)
 * M7986    Stock Top Z-axis   from printing M7986 Rx             M7986 Tx Zx         Sent during Printing Stock Top Z-axis value             *instructs Marlin to:
 *                                                                                        R=Get Stock_Top from current Z and subtract probe plate thickess(value after R)
 *                                                                                        S=Get Stock_Top from print gcode (val after S)(already corrected for probe thickness)
 *                                                                                        *Results are echoed only to the TFT SERIAL_PORT!
 * M7987    VFD Input Registers nothing	                    >>>>> auto-sent           Marlin sends every 2s (10s when printing)(see vfd.cpp)    
 * M7988    VFD sw and comm     nothing	                          M7988 R             returns VFD sw_ver, cpu_ver, baudrate, format         
 * M7989    TFT print state     M7990 Px 		                      M7990 ok            TFT print state sent to Marlin (0=printing,1=printing)
 *
 *
 *
 * The info sent from TFT to Marlin is stored in Marlin's AVRInfoBlock struct and avrpid[]
 * which can be exchanged with the AVR Triac controller via I2C commands.
 *  
 * The main `loop()` gets the command at the front the G-code queue (if any) and runs it. Each G-code command blocks the main loop, preventing the queue
 * from advancing until it returns. To keep essential tasks and the UI running, any commands that run a long process need to call `idle()` frequently.
 * Don't use idle(); in loops waiting on I2C comm, it's not necessary and may actually hurt I2C comm. Use a NOP to insure while() allows CPU some time.
 * REMEMBER - Whenever the G-code processor is busy processing a command, the G-code queue cannot advance.
 */

#include <src/MarlinCore.h>
#include "../gcode.h"
#include "../../inc/MarlinConfig.h"
#include "M7979.h"
#include "../../module/vfd.h"
#include "../../feature/spindle_laser.h"
#include "../../module/motion.h"
#include "../../feature/host_actions.h"

char Msg[120];   //TG 12/27/22 increased from 60, for use in some of the code below that needs a buffer

#if ENABLED(AVR_TRIAC_CONTROLLER) //TG 12/16/22 only if using AVR Triac Controller
AVRINFOTYPE AVRInfoBlock;
volatile uint8_t RecvAVRBlockComplete = 0;
volatile uint8_t sendAVRBlockComplete = 0;

// The functions below are called from gcode.cpp function process_parsed_command()


//************************  UART CODES for TFT/MARLIN AVRBlockInfo DATA ONLY *********************************
//M7900  - Msg from TFT to update AVR InfoBlock 
void GcodeSuite::M7900() {    // includes data for all flags and checksum in one message (not including PID values) 
  sendAVRBlockComplete = 0;
  if (parser.seenval('F'))    // TFT is sending AVRInfoBlock update to us
  {                           AVRInfoBlock.PIDFLAG = parser.value_byte();
    if (parser.seenval('R'))  AVRInfoBlock.Reset_Flag = parser.value_byte();
    if (parser.seenval('N'))  AVRInfoBlock.Display_Page = parser.value_byte();  
    if (parser.seenval('S'))  AVRInfoBlock.PID_Speed = parser.value_byte();  
    if (parser.seenval('U'))  AVRInfoBlock.Update_EEPROM = parser.value_byte();
    if (parser.seenval('C'))  AVRInfoBlock.EE_chksum = parser.value_int();
    if (parser.seenval('B'))  AVRInfoBlock.Reload_Preset = parser.value_byte();
    if (parser.seenval('D'))  AVRInfoBlock.Data_Interval = parser.value_byte();
    if (parser.seenval('P'))  AVRInfoBlock.PID_Interval = parser.value_byte();
    cutter.spindle_use_pid =  AVRInfoBlock.PIDFLAG;   // update Marlin's internal pid flag   
    while(sendAVRBlockComplete == 0){NOP};            // wait till AVR has picked up the new data block via I2C
    
    /* no other response to wait for with this command */
    
    sprintf(Msg,"M7900 OK\n");                        // respond back to TFT that AVRInfoBlock update is completed
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
  }
    
  if (parser.seen('R'))       // TFT requesting we send AVRInfoBlock to it
  {
    uint16_t chksum = get_chksum();
    sprintf(Msg,"M7900 F%d R%d N%d S%d U%d C%u B%d D%d P%d\n", AVRInfoBlock.PIDFLAG, AVRInfoBlock.Reset_Flag, 
                  AVRInfoBlock.Display_Page, AVRInfoBlock.PID_Speed, AVRInfoBlock.Update_EEPROM, AVRInfoBlock.EE_chksum,
                  AVRInfoBlock.Reload_Preset, AVRInfoBlock.Data_Interval, AVRInfoBlock.PID_Interval);
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
  }
}

//**************************************************************************************************
//M7979  - UART send from TFT the PID_FLAG for Marlin & AVR - TFT reads this in M7900 request
void GcodeSuite::M7979() {            // and AVR I2C Master reads when requesting AVRInfoBlock
  if (parser.seen('S')) 
  {  
    cutter.spindle_use_pid = parser.value_byte()==1 ? true : false;
    AVRInfoBlock.PIDFLAG = cutter.spindle_use_pid;
  }
  else // no parameters - just emit a message to the serial port
    SERIAL_ECHOLNPGM("Spindle Use PID: M7979: ", cutter.spindle_use_pid);

  sendAVRBlockComplete = 0;
  while(sendAVRBlockComplete == 0){NOP};        // wait till AVR has picked up the new data block via I2C
  
  /* no other response to wait for with this command */

  sprintf(Msg,"M7979 OK\n");                    // signal TFT that PIDFLAG update is completed
  SERIAL_ECHOPGM(Msg);
  SERIAL_EOL();
}

//************************  UART CODES for TFT SENDING DATA ONLY ***********************************
//M7980  - UART send from TFT reset flag to AVR   - the TFT will never request this value  
void GcodeSuite::M7980() {    
  
  AVRInfoBlock.Reset_Flag = 1;        // AVR I2C Master will see flag via AVRInfoBlock
  
  /* don't wait for (sendAVRBlockComplete == 0) it will never come, instead wait for (RecvAVRBlockComplete == 0) since
     the AVR sends I2C RCV_AVR_INFO_BLOCK on restart. This delays the M7980 OK to the TFT until the restart is completed */
  RecvAVRBlockComplete = 0;
  while(RecvAVRBlockComplete == 0){NOP};  // AVR asks us to receive only on startup(reset) & PID_Speed change
  
  sprintf(Msg,"M7980 OK\n");          // signal TFT AVR Reset is completed
  SERIAL_ECHOPGM(Msg);
  SERIAL_EOL();
}

//************************  UART CODES for TFT SENDING & RECEIVING DATA ****************************
//M7981  - UART send from TFT / Read back to TFT for P,I,D updates
/**SEND AND RECEIVE PID** constants to/from the TFT LCD and Marlin's buffer array avr_pid[] (source for I2C data exchange with AVR Controller*/
void GcodeSuite::M7981() {    
  if (parser.seenval('P'))    // TFT is sending us PID constants from TFT avrTriac menu to update Marlin's copies 
  {                           AVRInfoBlock.KP = parser.value_float();  // receive data from TFT into array buffer
    if (parser.seenval('I'))  AVRInfoBlock.KI = parser.value_float();
    if (parser.seenval('D'))  AVRInfoBlock.KD = parser.value_float();
    
    sendAVRBlockComplete = 0;
    while(sendAVRBlockComplete == 0){NOP};    // wait till AVR has picked up the new data block via I2C
    
    /* no other response to wait for with this command */

    sprintf(Msg,"M7981 OK\n");        // signal TFT that KP,KI,KD update is completed
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
  }
  
  if (parser.seen('R'))       // TFT requesting PID constants, we  will send Marlin's current copies
  {
    sprintf(Msg,"M7981 P%3.4f I%3.4f D%3.4f\n", AVRInfoBlock.KP, AVRInfoBlock.KI, AVRInfoBlock.KD);
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
  }
}
//**************************************************************************************************

//M7982  - UART send from TFT to cycle AVR LCD display   - TFT reads this in M7900 request
void GcodeSuite::M7982() {    
  if (parser.seenval('P'))  // TFT sending us page number for AVR display
  {
    AVRInfoBlock.Display_Page = parser.value_byte();// get page 0,1,2 gets read when Master requests AVRBlockInfo
    sendAVRBlockComplete = 0;
    while(sendAVRBlockComplete == 0){NOP};// wait till AVR has picked up the new data block
    
    /* no response to wait for with this command */

    sprintf(Msg,"M7982 OK\n");          // signal TFT page change is completed
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();  
  }
}

//M7983  - UART send  from TFT to update AVR pid speed - TFT reads this in M7900 request
void GcodeSuite::M7983() {     
  if (parser.seenval('S'))  // TFT sending us PID Speed preset number
  {
    AVRInfoBlock.PID_Speed = parser.value_byte();   // get speed 0,1,2
    while(sendAVRBlockComplete == 0){NOP};          // wait till it's been sent over I2C 

    RecvAVRBlockComplete = 0;
    while(RecvAVRBlockComplete == 0){NOP};          // Wait for AVR's response (only on startup & PID_Speed change)    
    
    sprintf(Msg,"M7983 OK\n");                      // signal TFT change is completed
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
  }
}

//M7984  - UART send  from TFT to reload a pid speed - TFT should never need to read this back
void GcodeSuite::M7984() {     
    if(parser.seenval('S'))   //TFT sending S1=reload pidSpeed or S2=reload default 
      AVRInfoBlock.Reload_Preset = parser.value_byte();
    
    // pass AVRInfoBlock.Reload_Preset command to AVR on next I2C ExchangeMarlinData pass
    sendAVRBlockComplete = 0;
    RecvAVRBlockComplete = 0;
    while(sendAVRBlockComplete == 0){NOP};    // wait till it's been sent over I2C 
    
    
    while(RecvAVRBlockComplete == 0){NOP}     // Wait for AVR's response (only on startup & PID_Speed change)
    
    AVRInfoBlock.Reload_Preset = 0;     // be sure to reset it to avoid multiple sends
    sprintf(Msg,"M7984 OK\n");          // signal the TFT that reload is completed
    SERIAL_ECHOPGM(Msg);
    SERIAL_EOL();
}

uint16_t get_chksum()
{	
	// create a unique checksum, some byte values can be ambiguous (i.e. suppose PIDFLAG=1 and
	// Reset_Flag=0, if they each toggled state there's no change to a simple summing checksum)
	// therefore we multiply each item by different amounts to make them unique.
	uint8_t i;
	uint16_t cs = 0;
	uint8_t * ifb = &AVRInfoBlock.PIDFLAG;

	uint8_t max = sizeof(AVRInfoBlock) - 2;		// don't include cs word, last 2 bytes
	for (i=0; i<max; i++)
	{
		cs += (*ifb++) * (max-i);				// give each item a different "weight"
	}
	return cs;
}

#endif // #if ENABLED(AVR_TRIAC_CONTROLLER)


/*M7986  - This Gcode sent during Printing - the Z-Axis value of the Stock Top - TFT can read this back
  NOTE: The gcode parser always replies to the source of a gcode command, which is fine most of the time.
  However, since M7986 is expected to come from a print file, the source may not always be the SD Card on
  the TFT, it could be the SD on the SGEN board, OctoPrint, USB port, etc. We have to send the message below
  always to the TFT no matter what source sent the M7986 Rx. The PORT_REDIRECT macro let's us temporarily
  switch where the serial output will be directed.
*/
void GcodeSuite::M7986() {
  float stockTop = 0, stockTopNative = 0;     
    if(parser.seenval('S')){   // S=Stock_Top from print gcode (already corrected for probe thickness)
    // echo gcode file-sent value as Stock_Top and native machine Zmax to TFT immediately, TFT will not ask
    sprintf(Msg,"M7986 T%6.3f Z%6.3f ok\n", parser.value_float(), (float)Z_MAX_POS);
    PORT_REDIRECT(SERIAL_PORTMASK(SERIAL_PORT));  // ensure we send to the TFT (in case source was not the TFT)
    SERIAL_ECHOPGM(Msg);
    }
    if(parser.seenval('R')){   // R=Get Stock_Top from current Z pos minus any probe thickess(value after R)
    stockTop = (float)current_position.z - parser.value_float();
    // echo current Z position as Stock_Top and native machine Zmax to TFT immediately, TFT will not ask
    sprintf(Msg,"M7986 T%6.3f Z%6.3f ok\n", stockTop, (float)Z_MAX_POS);
    PORT_REDIRECT(SERIAL_PORTMASK(SERIAL_PORT));  // ensure we send to the TFT (in case source was not the TFT)
    SERIAL_ECHOPGM(Msg);
    } 
    if(parser.seen('P')){ //TG 3/24/23 added to show the stock position as an //action:prompt, no buttons
      stockTopNative = stockTop - (float)Z_MIN_POS; 
      sprintf(Msg, "     Current Z: %7.3f             STOCK_POS: %7.3f             Z_MAX_POS: %7.3f             Z_MIN_POS: %7.3f ", stockTop, stockTopNative, (float)Z_MAX_POS, (float)Z_MIN_POS);
      hostui.prompt_do(PROMPT_INFO, (const char *const)Msg, (FSTR_P)NULL,(FSTR_P)NULL);
    }
  }

#if ENABLED(VFD_CONTROLLER)			 //TG 12/16/22 only if using VFD controller

//***** THIS IS NOT IMPLEMENTED HERE ***********************************************
// M7987 used to auto-report VFD Input Register values to TFT - handled in vfd.cpp *
//**********************************************************************************

void GcodeSuite::M7988() {       //TG 12/23/22 added 
    if(parser.seen('R')){
      uint16_t baudrate, format, swver, cpuver;

      baudrate=format=swver=cpuver=0;
      if (VFDpresent==true)							// if VFD connected
      {
        statusPollingAllowed = false;   // halt status polling so we don't have collision
        bool status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, 164, 0x0001);
        if (status==true)
          baudrate = mbResponseMsg[4];
        status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, 165, 0x0001);
        if (status==true)
          format = mbResponseMsg[4];
        statusPollingAllowed = true;    // allow status polling, we're done
      }
      sprintf(Msg,"M7988 SW%d CP%d BR%d FT%d ok\n",sw_ver, cpu_ver, baudrate, format);
      PORT_REDIRECT(SERIAL_PORTMASK(SERIAL_PORT));  // ensure we send to the TFT (in case source was not the TFT)
      SERIAL_ECHOPGM(Msg);
      PORT_RESTORE(); 
    }
}

void  GcodeSuite::M7989() {   //TG 2/13/23 added - receive TFT printing state from TFT
    if (parser.seen('P')) 
    {  
      tftPrinting = parser.value_byte()==1 ? true : false;  //get TFT SD printing state
      SERIAL_ECHOLNPGM("M7989 ok");                         // acknowledge receipt
    }

}


#endif // #if ENABLED(VFD_CONTROLLER)	