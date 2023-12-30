/** //TG MODIFIED BY T.GIOIOSA
 * 
 * VFD Controller support via LPC1769 UART3 TX/RX which connects to a TTL-to-RS485 adapter to provide Modbus communication
 * 
 * We need a UART to talk to the VFD over an RS485 interface to the VFD with Modbus protocol. 
 * On the SGEN LPC1769 board UART3(P0_0,P0_1) is the only UART left available.
 * You may need to find other available UART and pins on other boards.
 * 
 * Stock Marlin defines SERIAL_PORT , SERIAL_PORT_2, and SERIAL_PORT_3 as possible UART objects.
 * SERIAL_PORT is the main serial port on the board to use for communication with a host, and is = 0 usually the TFT port.
 * SERIAL_PORT_2 is a secondary serial port on the board to use for communication with a host and is = -1 the USB port.
 * SERIAL_PORT_3 is not assigned in stock Marlin, although it could be used for a serial LCD or an MMU device.
 * 
 * Each of the above three names(if defined) will get assembled into a MultiSerial object (see serial.h and serial_hook.h)
 * The MultiSerial object replicates output to all three ports but can selectively mask each one to hide the others, however, all
 * G-Code parsing and output work through this MultiSerial object. This also means that data on these ports is forwarded to
 * the gcode parser and makes it difficult to receive and use incoming bytes for other purposes(protocols).
 * 
 * For that reason we can't use the identifier SERIAL_PORT_3 for isolated communication to a connected device. We need the
 * channel to work with that device exclusively, and not in a shared situation (where all G-Code transactions will get mixed
 * in with device-only intended communication).
 * 
 * Instead we can use the identifier LCD_SERIAL_PORT or MMU2_SERIAL_PORT. These don't get included into the MultiSerial object,
 * and can be isolated to exclusive communication to/from a connected device).
 * 
 * Assigning a port number to LCD_SERIAL_PORT creates MSerial3 (as an instance of the ForwardSerial class) that 
 * references an instance of MarlinSerial named _MSerial3 using LPC_UART3, which inherits class HardwareSerial
 * in HardwareSerial.h. Finally an identifier LCD_SERIAL is #defined to be the _MSerial3 object. (whew!)
 *
 * T. Gioiosa 12/16/22
 * updated 2/16/23
 */

#include "../inc/MarlinConfig.h" // needed for the next line #if ENABLED
#if ENABLED(VFD_CONTROLLER)			 //TG 12/16/22

#include <src/MarlinCore.h>
#include "vfd.h"
#include <src/sd/cardreader.h>

// write/read buffers for Modbus serial
byte mbCommandMsg[maxCommandMsgSize] = {0}; 	// dynamic arrays    
byte mbResponseMsg[maxResponseMsgSize] = {0};   
byte mbCommandLen = 0;												// hold the size of the arrays(can change dynamically)
byte mbResponseLen = 0;

bool tftPrinting = false;					// this flag is received from TFT35 to indicate if it is printing from SD
bool VFDpresent = false;					// this flag is set from MarlinCore call to initialReadVFD() below
bool statusPollingAllowed = true;	// used to block the getVFDStatus() routine if needed
byte VFD_precision = 0;						// will be either 10 or 100, read at startup from the VFD
uint16_t sw_ver;
uint16_t cpu_ver;
iregBits inputReg;								// struct containing frequencies, voltages, current, temp, etc.
uint32_t next_status_time;				// interval that the status function will execute at
byte VFDstatusbyte;

// This code gets called from the MarliCore.cpp idle() loop. It will only execute every 10 secconds when printing,
// or every 2 seconds when not printing. It always tests the VFDpresent flag first, if false then try to re-connect.
// If unsuccessful, just exit till next interval.
// If VFDpresent flag is true, then VFD status is read into VFDstatusbyte and decoded, followed by reading
// the VFD input registers and placing that data into inputReg structure. 
// An M7987 msg(custom defined gcode) with all status data is then sent to the TFT35 so it can refresh it's data.
// If at any time the VFDpresent flag goes false, on next entry here we'll try to re-connect(stopping the VFD motor)
// and will keep trying to re-connect. Once the VFD reconnects, status reading resumes, otherwise no status
// reading will be done until the VFD is connected (status reading will be skipped).
void getVFDStatus()
{
	bool status;		// holds return flag for any Modbus write commands, true if write successful, false otherwise
	static char Msg[96] = {0};		// buffer for sprintf message strings 
	
	// do some checking to see if we should run or exit
	if (!ELAPSED(millis(),next_status_time)) {return;}	// skip and return if not time to run yet
	if (statusPollingAllowed == false) {return;}			  // somebody using RS485 comm at the moment (this is the only place this flag is tested)
	
	// so far ok, now set up for next run interval
	bool ANY_PRINTING = (card.flag.sdprinting && !card.flag.abort_sd_printing) || tftPrinting;	//TG state of On-board SD/USB printing or TFT SD/USB printing
	next_status_time = millis() + (ANY_PRINTING ? 10000UL : 2000UL);														//TG 2/16/23 next interval 10s if printing or 2s if idle

	// is the VFD connection status still good?
	if (VFDpresent == false) {													// if VFD disconnected then...
		initialReadVFD(true);															// try to re-connect, stops motor too, in case it kept running after disconnect
		statusPollingAllowed = true;											// allow us to try here again (statusPollingAllowed could =false if initialReadVFD() failed)
		SERIAL_ECHOPGM(Msg);
		SERIAL_EOL();
		if (VFDpresent == false) return;									// leave if still disconnected
	}
			
	// everything is good so let's get the status info.....
	
	// request VFD status byte, if successful the response will be in global mbResponseMsg[] array, index 4
	// VFD responds in 16ms with 7-byte msg (5+2 data) at 7.2ms width
	status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, VEVOR_PARM_ADDR_BITS, 0x0001);
	if (status==true)
	{
		VFDstatusbyte = mbResponseMsg[4];		// status returns as 2 bytes, msb[3] & lsb[4], we want lsb at index=4
	}
	else 
	{VFDpresent=false;}							// if the VFD has stopped responding change status to not present

	statusPollingAllowed = false; 				// prevent infinite loop, otherwise idle() will call back here recursively
	idle();																// allow background hardware tasks to run (prevents comm pile-up)
	statusPollingAllowed = true;

	// now read the VFD Input Register Area for various operating parameters, adjust for decimal point where needed. Request 4 registers
	// starting at VFD Read Holding Register 0220H for Input Registers 0000H-0003FH (freq_out, freq_set, current_out, speed_out)
	// VFD responds in 16ms with 13-byte msg (5+8 data) at 13.4ms width
	status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, VEVOR_INPUT_REG_BITS, 0x0004);
	if (status==true)
	{ //convert 2 bytes each to 16-bit values
		inputReg.freq_out = (mbResponseMsg[3]<<8) + mbResponseMsg[4];	
		inputReg.freq_set = (mbResponseMsg[5]<<8) + mbResponseMsg[6];
		inputReg.current_out = (mbResponseMsg[7]<<8) + mbResponseMsg[8];
		inputReg.speed_out = (mbResponseMsg[9]<<8) + mbResponseMsg[10];	
	}
	else {VFDpresent=false;}							// if the VFD has stopped responding change status to not present
	
	statusPollingAllowed = false; 				// prevent infinite loop, otherwise idle() will call back here recursively
	idle();																// allow background hardware tasks to run (prevents comm pile-up)
	statusPollingAllowed = true;

	// request 3 registers starting at Read Holding Register 0224H for Input Registers 0004H-0006FH (dc_voltage, ac_voltage, temperature)
	// VFD responds in 16ms with 11-byte msg (5+6 data) at 11.2ms width
	status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, VEVOR_INPUT_REG_BITS+4, 0x0003);
	if (status==true)
	{
		inputReg.dc_voltage = (mbResponseMsg[3]<<8) + mbResponseMsg[4];
		inputReg.ac_voltage = (mbResponseMsg[5]<<8) + mbResponseMsg[6];
		inputReg.temperature = (mbResponseMsg[7]<<8) + mbResponseMsg[8];
	}
	else {VFDpresent=false;}							// if the VFD has stopped responding change status to not present

	statusPollingAllowed = false; 				// prevent infinite loop, otherwise idle() will call back here recursively
	idle();																// allow background hardware tasks to run (prevents comm pile-up)
	statusPollingAllowed = true;

	// request 2 registers starting at Read Holding Register 022AH for Input Registers 000AH-000BFH (fault_code, total_hours)
	// VFD responds in 16ms with 9-byte msg (5+4 data) at 9.2ms width
	status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, VEVOR_INPUT_REG_BITS+10, 0x0002);
	if (status==true)
	{
		inputReg.fault_code = (mbResponseMsg[3]<<8) + mbResponseMsg[4];
		inputReg.total_hours = (mbResponseMsg[5]<<8) + mbResponseMsg[6];
	}
	else {VFDpresent=false;}							// if the VFD has stopped responding change status to not present
	
	statusPollingAllowed = false; 				// prevent infinite loop, otherwise idle() will call back here recursively
	idle();																// allow background hardware tasks to run  (prevents comm pile-up)
	statusPollingAllowed = true;

	// finally, format and send the latest VFD registers and status to TFT35 for display
  sprintf(Msg,"M7987 OF%d SF%d OC%d OR%d DC%d AC%u TP%d LF%d RH%d ST%d DP%d\n", 
          inputReg.freq_out, inputReg.freq_set, inputReg.current_out, inputReg.speed_out,
          inputReg.dc_voltage, inputReg.ac_voltage,inputReg.temperature, inputReg.fault_code,
          inputReg.total_hours, VFDstatusbyte, VFD_precision);
  
	PORT_REDIRECT(SERIAL_PORTMASK(SERIAL_PORT));    // redirect to ensure we send to the TFT35 (in case source port was not the TFT)
  //PORT_REDIRECT(SerialMask::All);								// for testing output with Pronterface or Repetier send to all serial ports
	SERIAL_ECHOPGM(Msg);
	PORT_RESTORE();																	// sets port redirection back to previous value 
}

// Read initial state of VFD and optionally stop the motor if stopMotor=true
// this gets called by from MarlinCore.cpp idle(), it accomplishes these tasks:
// 1. if stopMotor parm = true, stops the VFD motor if possible
// 2. read VFD's decimal precision setting to know what factor to apply when reading other VFD input registers
// 3. read the VFD software version
// 4. read the VFD motor CPU version
// ALL steps must be successful for the VFDpresent flag to be set true (and statusPollingAllowed also), if any one
// step fails(except the first motor Stop attempt), the rest are skipped and the VFDpresent flag is set to
// false (the statusPollingAllowed flag also stays false).
// The resulting VFDpresent flag gets sent to the TFT automatically at Temperature::AutoReportTemp::report()
// it is appended to the line sent by Marlin M155 Autoreport interval.
void initialReadVFD(bool stopMotor)
{
	bool status = true;							// start with true in case we skip motor stop code
	statusPollingAllowed = false; 	// don't allow interference from periodic status polling
  static char Msg[18] = {0};			// buffer for sprintf message strings 

	if(stopMotor)
	{
		// make sure first thing done is try to stop the motor in case it's running 
  	status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_MAIN_CONTROL_BITS, sRUN_STOP);
		// if first attempt failed, try one more time
		if(status==false)
		  status = writeVevorVFD(VFDnum, MODBUS_WRITE_FUNC_REG, VEVOR_MAIN_CONTROL_BITS, sRUN_STOP);
	}
	
	// try to read the VFD's decimal precision setting, mbResponseMsg[4] holds the return data
	if (status==true)
		status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, 169, 0x0001);
	if (status==true)
		VFD_precision = (mbResponseMsg[4] + 1) * 10;	// 10 if mbResponseMsg[4] = 0, or 100 if mbResponseMsg[4] = 1
	else
		VFD_precision = 10;														// if an error occurred set default at 10
	
	// read the VFD software version
	if (status==true)
		status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, 181, 0x0001);
	if (status==true)
		sw_ver = mbResponseMsg[4];
	
  // read the VFD motor CPU version  
	if (status==true)
	  status = writeVevorVFD(VFDnum, MODBUS_READ_HOLD_REG, 189, 0x0001);
	
	if (status==true)
	{	
		cpu_ver = mbResponseMsg[4];
		VFDpresent = true;														// made it through all the status checks, VFD is connected
		statusPollingAllowed = true;									// VFD is connected, OK to allow status polling now
	}
	else
		VFDpresent = false;														// set flag if no VFD responded
			
	PORT_REDIRECT(SERIAL_PORTMASK(SERIAL_PORT));  	// redirect to ensure we send to the TFT35 (in case source port was not the TFT)
	sprintf(Msg,"M7987 OF0 DP%d\n",VFD_precision);
	SERIAL_ECHOPGM(Msg);														// setting DP to 255 flags VFD no-response to TFT
	PORT_RESTORE();																	// sets port redirection back to previous value 
	    
}

/* Serial port routines begin here, we communicate to VFD via serial port connected to RS485 converter to VFD */
#ifdef LCD_SERIAL_PORT  //TG 12/16/22

// read bytes of length len into buffer at *data, if len=0 nothing will be done and just returns zero
uint8_t Serial_readNbytes(uint8_t* data, int len){
	uint8_t i;
	for( i=0; i < (len-1); i++)	{data[i]= 0;}				// clear buffer first
	
	while ((len > 0) & (LCD_SERIAL.available()>0))	// till len bytes or no more available			
	{
		*data = LCD_SERIAL.read();										// read bytes into buffer (actual code is in HardwareSerial.h)
		++data;
		--len;				
	}

	if (len > 0) 					// if len > 0, there weren't enough bytes available to read
		return  len;				// return #bytes that were not read
	else 									// return success (0) if all bytes read
		return 0;
}

// format the supplied variables into a Modbus message for the Vevor VFD, and store it in
// a global array with a calulated CRC as the last two bytes of the array. Then call to
// LCD_SERIAL.write with the address of the array and the size in bytes.
// Return true if ok, false if error

//TG 12/23/23 found that noise from VFD when spindle runs causes the RS485 comm to glitch occasionally,
//returning an error which then causes a new initialReadVFD() call (and stopping the motor).
//Added ferrite cylinders to the VFD<>Marlin board cable, which helped but did not completely eliminate.
//Finally decided to add a check of mbResponseLen when starting to read the available received bytes
//after the first byte has been detected to make sure it wasn't an erroneous 1st byte or a glitch in
//receiving the rest of the message. If the mbResponseLen is zero after the 1st byte was detected, we
//assume it's a bad read(glitch) error and jump to retry sending the cmd and reading the response again.
//We will retry to recover up to 3 times before finally registering an unrecoverable error.

bool writeVevorVFD(byte dev_addr, const byte func, uint16_t param_addr, uint16_t data)
{
	byte i,j,trycount;
	
	trycount = 0;		// clear the error retry counter

	// clear arrays
	for (i=0; i<maxCommandMsgSize;i++) {mbCommandMsg[i] = 0;}
	for (i=0; i<maxResponseMsgSize;i++) {mbResponseMsg[i] = 0;}
	
	// start by calculating size of byte array we'll need for the message, then create a dynamic array of that size
	mbCommandLen = 2 + sizeof(param_addr) + sizeof(data) + 2;		// address, cmd bytes(2) + parm + data + crc bytes(2), put in global var
  
	// fill the array with everything but the CRC bytes (dynamic array should be accessed by index not as a pointer)
	mbCommandMsg[0] = dev_addr;    					
	mbCommandMsg[1] = func;
	mbCommandMsg[2] = (param_addr & 0xFF00)>>8;
	mbCommandMsg[3] = param_addr & 0xFF;
	mbCommandMsg[4] = (data & 0xFF00)>>8;
	mbCommandMsg[5] = data & 0xFF;
		
	// next, calculate the 16-bit CRC for the above bytes and add them as last two bytes of the array
	uint16_t crc  = 0xFFFF;
	byte crclen = mbCommandLen - 2;							// create the Modbus CRC, but don't count the 2 bytes for the CRC
	i=j=0;
	while(crclen--)
	{  
	  crc = crc ^ mbCommandMsg[i++];
    for(j=0; j<8; j++)
    {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc >>= 1;
		}
	}
	mbCommandMsg[i++] = crc & 0xFF;							// put CRC lsbyte in dest at totalLen - 2
  mbCommandMsg[i]   = (crc & 0xFF00) >> 8;		// put CRC msbyte in dest at totalLen - 1	

	retry:		// jump back here on errors to retry 
	// next, write completed mbCommandMsg[] to the serial port and wait till TX buffer is empty
		_delay_ms(50);	 // VFD protocol needs about 50ms space between commands
		
	//WRITE(P2_12,1);    //TG - ***** for scope measurement normally commented out
	LCD_SERIAL.write((char*)&mbCommandMsg[0], (size_t)mbCommandLen);  // (actual code is in HardwareSerial.h)
  while(LCD_SERIAL.TXbufferEmpty()==1){;}			// 1=TX busy  0=TX empty
	//WRITE(P2_12,0);
	
	// the Vevor VFD takes about 16ms to respond at 9600baud
	mbResponseLen = 0;
	uint32_t timeout = millis() + 24UL;					// 16ms + 50% timeout for no response
	// wait until first byte is received or timeout occurs
	while(mbResponseLen==0 && !ELAPSED(millis(),timeout)) {mbResponseLen = LCD_SERIAL.available();}
	
	// got first byte so allow for longest expected response to be received into serial buffer
	_delay_ms(16);	// largest response message should be 13 bytes or 13.5ms, add 25%																					
	
	// read the received bytes into global mbResponseMsg[], caller can inspect it, but also return status boolean if error
	mbResponseLen = LCD_SERIAL.available();								// get response length
	if(mbResponseLen == 0 && trycount++ <3) goto retry;

	Serial_readNbytes(&mbResponseMsg[0], mbResponseLen);	// read that number of bytes (or zero) into global buffer
  if (mbResponseMsg[1] & 0x80 || mbResponseMsg[1]==0)		// check for error flag (bit7 set), or first byte null (timeout)
	{
		//WRITE(P4_28,0);    	//TG - ***** for scope measurement normally commented out
    //asm("nop");
		//asm("nop");
		return false;
	}
	else
	{
		//WRITE(P4_28,1);			//TG - ***** for scope measurement normally commented out
	  return true;
	  
	}
}	


// TG - Not really used anymore, it is all done in the writeVevorVFD() code above
// takes an input source array ptr, a dest array ptr, and the src array size and forms a
// Modbus message into dest array. The array will contain ADDR, SRC BYTES, and calculated CRC 
uint16_t createModbusMsg(byte *dest, byte cmd, uint16_t parm, uint16_t data)
{
	uint16_t crc  = 0xFFFF;
	byte j;
  
	byte totalLen = 2 + sizeof(parm) + sizeof(data) + 2;		// address, cmd bytes (2) + parm + data + crc bytes(2)
	byte *destcopy = dest;					// copy start of dest for later cause we manipulate it

	*dest++ = VFDnum;    						// put the address byte
	*dest++ = cmd;
	*dest++ = (parm & 0xFF00)>>8;
	*dest++ = parm & 0xFF;
	*dest++ = (data & 0xFF00)>>8;
	*dest++ = data & 0xFF;
		
	dest = destcopy;								// reset dest ptr back to first element
	totalLen -= 2;									// create the Modbus CRC, but don't count the 2 bytes for the CRC
	while(totalLen--)
	{  
	  crc = crc ^ *dest++;
    for(j=0; j<8; j++)
    {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc >>= 1;
		}
	}
	*dest++ = crc & 0xFF;						// put CRC lsbyte in dest at totalLen - 2
  *dest   = (crc & 0xFF00) >> 8;	// put CRC msbyte in dest at totalLen - 1	
	return crc;
}	

#endif // #ifdef LCD_SERIAL_PORT
#endif // #if ENABLED(VFD_CONTROLLER)




