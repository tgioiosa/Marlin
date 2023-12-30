/** //TG MODIFIED BY T.GIOIOSA
 * Serial UART routines based on LCD_SERIAL_PORT
 * T. Gioiosa 1216/22
 * 
 */

//#pragma once
#ifndef _VFD_H_
#define _VFD_H_

#include <src/MarlinCore.h>
#if ENABLED(VFD_CONTROLLER)  //TG 12/16/22

#include <stdint.h>
#define maxCommandMsgSize 8
#define maxResponseMsgSize 16

//On the VFD parameter F164=0(4800),1(9600),2(19200),3(38400)
#define VFD_BAUD_RATE 9600  //TG 12/23/23 set this here and Configuration_adv.h will use it for LCD_SERIAL_PORT

//TG note that for the following to have a SINGLE INSTANCE shared by all .cpp files that include vfd.h,
//they must be declared 'extern' here and defined in one .cpp file (vfd.cpp in this case).
//If they were declared and defined here as 'static byte mbCommandMsg[]', etc. then every .cpp file
//would get it's OWN COPY and thus it's not really shared properly. 
extern byte mbCommandMsg[maxCommandMsgSize];              // global array for writing Modbus msg
extern byte mbResponseMsg[maxResponseMsgSize];            // global array for receiving response msg
extern byte mbCommandLen;
extern byte mbResponseLen;

const byte VFDnum = 0x01;                       // specify Modbus address of the VFD
extern byte VFD_precision;                      // decimal precision of communication for VFD (tenths or hundredths)
extern byte VFDstatusbyte;                      // holds the 8 status bits from VFD operation
extern uint16_t sw_ver;
extern uint16_t cpu_ver;
extern bool VFDpresent;                         // this flag is set from MarlinCore call to initialReadVFD() below
extern bool statusPollingAllowed;               // used to block getVFDStatus if needed


const byte MODBUS_READ_HOLD_REG = 0x03;         // Modbus functions
const byte MODBUS_READ_INPUT_REG = 0x04;
const byte MODBUS_WRITE_FUNC_REG = 0x06;

const byte sRUN_FWD = 0x03;                     // VFD control bits
const byte sRUN_REV = 0x05;
const byte sRUN_STOP = 0x08;

typedef struct 
{
  uint16_t  freq_out;
  uint16_t  freq_set;
  uint16_t  current_out;
  uint16_t  speed_out;
  uint16_t  dc_voltage;
  uint16_t  ac_voltage;
  uint16_t  temperature;
  uint16_t  counter;
  uint16_t  PID_target;
  uint16_t  PID_feedback;
  uint16_t  fault_code;
  uint16_t  total_hours;
}iregBits;
extern iregBits inputReg;

// some Holding Register definitions for the Vevor VFD (these locations can be read/write anytime, they are not cycle limited EEPROM)
const uint16_t VEVOR_MAIN_CONTROL_BITS    = 0x0200;   // BIT0-BIT7 mapping parameter address 0048H～004FH (Write only, use 0x210 to read)
                                                      // 0048 Operation 0000—Invalid FF00（or 100，bit8 = 1）－Valid
                                                      // 0049 Forward 0000—Invalid FF00－Valid
                                                      // 004A Reverse 0000—Invalid FF00－Valid
                                                      // 004B Stop 0000—Invalid FF00－Valid
                                                      // 004C Forward/reverse switch 0000—Invalid FF00－Valid
                                                      // 004D JOG 0000—Invalid FF00－Valid
                                                      // 004E JOG Forward 0000—Invalid FF00－Valid
                                                      // 004F JOG Reverse 0000—Invalid FF00－Valid

const uint16_t VEVOR_FREQUENCY            = 0x0201;   // set frequency desired

const uint16_t VEVOR_PARM_ADDR_BITS       = 0x0210;   // param register mapping 0000H ~ 000FH (read only, use 0x200 to write)
                                                      // 0000 Operation 0–Stop 1–Operating
                                                      // 0001 JOG 0–Invalid 1–JOG
                                                      // 0002 Forward/reverse 0–Forward 1–Reverse
                                                      // 0003 In operation 0–Stop 1–In operation
                                                      // 0004 In jogging 0–Invalid 1–In jogging
                                                      // 0005 In forward/reverse rotation 0–In forward rotation 1–In reverse rotation
                                                      // 0006 In braking  0–Invalid 1–In braking
                                                      // 0007 Frequency tracking  0–Invalid 1–Frequency

const uint16_t VEVOR_INPUT_REG_BITS       = 0x0220;   // input register mapping 0000H ~ 000FH 
                                                      // 0x0220+0000 Output frequency
                                                      // 0x0220+0001 Set frequency
                                                      // 0x0220+0002 Output current
                                                      // 0x0220+0003 Output speed
                                                      // 0x0220+0004 DC voltage
                                                      // 0x0220+0005 AC voltage
                                                      // 0x0220+0006 temperature
                                                      // 0x0220+0007 Counter 
                                                      // 0x0220+0008 PID target value
                                                      // 0x0220+0009 PID feedback value
                                                      // 0x0220+000A Current fault 
                                                      // 0x0220+000B Total operating hours

// table of arrays containing the various commands for VFD, the last two bytes have to be filled in with CRC
const byte spindle_send_runfwd[] = {0x06, 0x02, 0x00, 0x00, 0x03};					// WriteSingleFuncReg 0x200 data=0x03
const byte spindle_send_runrev[] = {0x06, 0x02, 0x00, 0x00, 0x05};					// WriteSingleFuncReg 0x200 data=0x05
const byte spindle_send_stop[] = {0x06, 0x02, 0x00, 0x00, 0x08 };						// WriteSingleFuncReg 0x200 data=0x08
const byte spindle_send_speed[] = {0x06, 0x02, 0x01, 0x00, 0x00};						// WriteSingleFuncReg 0x201 data=sH,sL
const byte spindle_read_serial[] = {0x03, 0x00, 181, 0x00, 0x01};						// ReadHoldingRegister F181 1byte
const byte spindle_read_status[] = {0x03, 0x02, 0x10, 0x00, 0x01};					// ReadHoldingRegister 0x210 Main ctl bits 0-7
const byte spindle_read_fo_fi_cur_spd[] = {0x03, 0x02, 0x20, 0x00, 0x04};		// ReadHoldingRegister 0x220 4bytes
const byte spindle_read_dc_ac_temp[] = {0x03, 0x02, 0x24, 0x00, 0x03};			// ReadHoldingRegister 0x224 3bytes
const byte spindle_read_fault_hours[] = {0x03, 0x02, 0x2A, 0x00, 0x02};			// ReadHoldingRegister 0x22A 2bytes


extern bool tftPrinting;
uint8_t Serial_readNbytes( uint8_t* data, int len);
uint16_t createModbusMsg(byte* dest, byte cmd, uint16_t parm, uint16_t data);
bool writeVevorVFD(byte dev_addr, const byte func, uint16_t param_addr, uint16_t data);
void getVFDStatus();
void initialReadVFD(bool stopMotor);

#endif // #ifndef _VFD_H_

#endif // #if ENABLED(VFD_CONTROLLER)