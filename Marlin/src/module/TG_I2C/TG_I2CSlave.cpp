/** //TG MODIFIED BY T.GIOIOSA
 * Based on LPC17xx_I2C.c and adapted for Marlin using I2C1 in Slave Mode to send/receive rpm data packets
 * Note: lpc17xx_i2c.c and lpc17xx_i2c.h are still required because some of it's functions are called!!
 * T. Gioiosa 5/12/22
 * 
 * Currently only Slave Mode is fully implemented. Marlin is an addressed slave and the Master is the CNC
 * Speed Controller (AVR128DB28). The CNC Master reads TARGET_RPM (set from LCD via M3,M4,M5 commands) from
 * Marlin and sends ACTUAL_RPM to Marlin for relay to the LCD. The CNC Master also will read cutter.spindle_use_pid
 * flag and other flag info from Marlin for the CNC speed controller to use.

 * Description: Customized I2C driver for LPC1769
 */



#include <src/MarlinCore.h>
#include <src/inc/MarlinConfig.h>

#if ENABLED(TG_I2C_SUPPORT)	//TG 12/16/22
#include <lpc17xx_i2c.h>
#include <LPC17xx.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include "../../feature/spindle_laser.h"
#include "../../gcode/config/M7979.h"
#include "TG_I2CSlave.h"
#include <src/module/planner.h>
#include "src/HAL/shared/Delay.h"		// only needed if using delay_us()

#ifndef USEDI2CDEV_M								// Choose which LPC1769 I2C peripheral will be used (0, 1, or 2)
  #define USEDI2CDEV_M -1						// -1 means not selected
#endif
#ifndef USEDI2CDEV_S
	#define USEDI2CDEV_S 1						// using I2C1 interface for slave
#endif

#pragma region Set I2CDEV_M and I2CDEV_S to which I2C Peripheral is being used and set up IRQ handlers
#if (USEDI2CDEV_M == 0)
  #define I2CDEV_M LPC_I2C0
#elif (USEDI2CDEV_M == 1)
  #define I2CDEV_M LPC_I2C1
#elif (USEDI2CDEV_M == 2)
  #define I2CDEV_M LPC_I2C2
#else
  #define I2CDEV_M NULL
	#warning "Master I2C device not used!"
#endif

#if (USEDI2CDEV_S == 0)
  #define I2CDEV_S LPC_I2C0
#elif (USEDI2CDEV_S == 1)
  #define I2CDEV_S LPC_I2C1
#elif (USEDI2CDEV_S == 2)
  #define I2CDEV_S LPC_I2C2
#else
#define I2CDEV_S NULL
  #warning "Slave I2C device not used!"
#endif

#if (USEDI2CDEV_M == USEDI2CDEV_S)
  #error "Master and Slave I2C device are duplicated!"
#endif

#if ((USEDI2CDEV_M == 0) || (USEDI2CDEV_S == 0))
  #define I2C_ISR()  extern "C" void I2C0_IRQHandler()		// define the I2C handler to be I2C_ISR()
#elif ((USEDI2CDEV_M == 1) || (USEDI2CDEV_S == 1))
	#define I2C_ISR()  extern "C" void I2C1_IRQHandler()		// define the I2C handler to be I2C_ISR()
#elif ((USEDI2CDEV_M == 2) || (USEDI2CDEV_S == 2))
	#define I2C_ISR()  extern "C" void I2C2_IRQHandler()		// define the I2C handler to be I2C_ISR()
#endif
#pragma endregion

/** These global variables below used in interrupt mode -----------*/
#pragma region Global variables
__IO FlagStatus complete_M;
__IO FlagStatus complete_S;
I2C_S_SETUP_Type transferSCfg;      //TG data transfer struct for blocking interrupt or polled mode
																		// not needed for Full Interrupt mode
I2C_RX_buffer rx_int_buf;						// circular receive buffer
I2C_TX_buffer tx_int_buf;						// circular transmit buffer
volatile uint8_t COMMAND;						// command byte received from a Master device
char logMsg[100];										// string buffer for sprintf() messages
i2cStat RetStatus;									// used in mySlaveHandler, it's global here so we can view while debugging
volatile I2C_dir_type I2C_dir;			// direction, made global in case other code needs to use it

typedef union  {
    float f;
		uint16_t i;
		uint8_t b;
    unsigned char a[sizeof (float) ];
}floatunion_t;

//volatile uint16_t TARGET_RPM = 0;	//TG 12/16/22  moved to SpindleLaser class
//volatile uint16_t ACTUAL_RPM = 0; //TG 12/16/22  moved to SpindleLaser class

floatunion_t TRPM_u;				// union is used so we can access members as integer, byte, float, or float as a byte array
floatunion_t ARPM_u;

#pragma endregion
//TG to generate scope signals use WRITE(P2_12,1) / //WRITE(P2_12,0) 
//TG ***** Beware when enabling port pins for scope signals used to debug! Observe the following precautions for unused J8 signals:
//				P1_22	-	VACUUM ENABLE RELAY
//				P1_23 - previously PWM MOTOR DRIVE (not used with new AVR Triac Controller)
//				P2_11 - SPINDLE ENABLE RELAY
//				P2_12 - SPINDLE DIR (not used)
//				P4_28 - (not used)
//   DO NOT USE ANY OF THE ABOVE PINS AS SCOPE SIGNALS IN RELEASE VERSION!!
//TG *******************************************************************************************************************************

//TG ***** Main ISR routine for interrupt-driven I2C mode (this is preferred mode) *****
I2C_ISR()
{
	#if (USEDI2CDEV_M == 1)		 // *************** MASTER Mode ********************
		I2C_MasterHandler(I2CDEV_M);
		if (I2C_MasterTransferComplete(I2CDEV_M)){
			complete_M = SET;
		}
	#endif

	#if (USEDI2CDEV_S == 1)    // **************** SLAVE Mode ********************

	  #ifdef FULL_INTERRUPT		 // Fully Interrupt Driven method, fastest and most reliable method
	    uint8_t curStat;
			i2cStat HandlerStatus;

			//Optional determine whether RECEIVE or TRANSMIT mode, not currently used and may not really be necessary
			#pragma region Optional I2C direction detect
					/*	curStat = (I2CDEV_S->I2STAT >> 5) & 0x7;
			  		switch(curStat)
			  		{
			  			case 3:
			  			case 4:
			  			case 7:
			  				I2C_dir = RECEIVE;
			  			  break;
			  			case 5:
			  			case 6:
			  				I2C_dir = TRANSMIT;
			  				break;
							default:
								I2C_dir = RECEIVE;
							  break;
			  		}
						*/
			#pragma endregion
			
			// call mySlave Handler, if transaction still processing returns I2C_OK, I2C_BYTE_RECV, I2C_BYTE_SENT
			// if transaction complete returns I2C_ERR, I2C_NAK_RECV, I2C_SEND_END, I2C_RECV_END, I2C_STA_STO_RECV  
	    HandlerStatus = mySlaveHandler();
			
			if(HandlerStatus == I2C_OK || HandlerStatus == I2C_BYTE_RECV || HandlerStatus == I2C_BYTE_SENT)
			{
				processI2Cpacket();		// transaction still in process, hook to application code
			}
			else										// I2C transaction is complete
			{ 
				if (HandlerStatus == I2C_SEND_END)					// send transaction completed, check all bytes were sent
					{		
						if(tx_int_buf.head != tx_int_buf.tail)	// if master didn't take all bytes from buffer, flush the buffer
						{																				// to prevent buffer getting stuck when end of buffer is reached
							tx_int_buf.head = tx_int_buf.tail;
							tx_int_buf.count = 0;
						}
					}
				if (HandlerStatus == I2C_RECV_END)    			// should only get this if Rx buffer is full!
					{
						NOP
					}

				processI2Cpacket();																					// hook to handle I2C events
				I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;												// Keep Assert Ack going
				I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC;	// clear I2C Int flag, clear Start flag
				I2C_IntCmd(I2CDEV_S, TRUE);		// Re-Enable I2C Interrupts that were disabled after SlaveHandler completed
			}
	
	  #endif  // Full Interrupt Driven method


		// API method Blocking Interrupt or Polled, these are not preferred nor currently used, and there
		// is no handling in processI2Cpacket() for these modes
    #if ANY(BLOCKING_INTERRUPT, FULL_POLLING )
				I2C_SlaveHandler(I2CDEV_S);
				if(I2C_SlaveTransferComplete(I2CDEV_S))
					complete_S = SET;
    #endif //API method Int or Polled
  
	#endif // USEDI2CDEV_S == 1
}

/******  MAIN ROUTINE to process I2C Events  *****/
void processI2Cpacket(void)
{
uint8_t i;
#pragma region Interrupt driven method
#ifdef FULL_INTERRUPT
	if(rx_int_buf.count > 0)		// only if buffer has some data
	{

		 COMMAND = I2C_RX_peek();  // peek at byte from buffer to see if it's a command, if not, 
																					// we will continue coming back here until we get a command
  	switch(COMMAND)
		{
			case READ_TARGET_RPM:					// Master is requesting Target RPM from Slave
				I2C_RX_dequeue();						// remove command byte from buffer, no other Rx data bytes for this command
																		// Immediately fill transmit buffer with Target RPM to send back to Master
				writeTXbuf(cutter.TARGET_RPM);			// put byte, integer, or float data (auto detects) from union array into I2CTXDAT
				break;

			case WRITE_ACTUAL_RPM:				// Master wants to write Actual RPM to Slave
				if(rx_int_buf.count > sizeof(cutter.ACTUAL_RPM))
				{
					//WRITE(P2_11,1);
				  I2C_RX_dequeue();		  		// remove command byte from buffer, integer(2 data bytes) follows
				  readRXbuf(&cutter.ACTUAL_RPM);		// get bytes of integer or float data (auto detects) from RX buffer
			  	//WRITE(P2_11,0);
				}
				break;

			case SEND_AVR_INFO_BLOCK:			// Master is requesting Marlin's AVR infoBlock/status(including floats), send it
				I2C_RX_dequeue();						// remove command byte from buffer, no other Rx data bytes for this command
				AVRInfoBlock.PIDFLAG = cutter.spindle_use_pid;
				writeTXbufByteAry(&AVRInfoBlock.PIDFLAG, sizeof(AVRInfoBlock));		// send struct of bytes, pass first element as pointer
				if(AVRInfoBlock.Reset_Flag)  AVRInfoBlock.Reset_Flag = 0;					// always clear reset flag after sending
				if(AVRInfoBlock.Update_EEPROM) AVRInfoBlock.Update_EEPROM = 0;		// always clear EEPROM Update flag after sending
				sendAVRBlockComplete = 1;																					// flag completion
				break;				

			case RCV_AVR_INFO_BLOCK:			// Master is sending it's copy of AVR infoBlock/status(including floats), receive it
				if(rx_int_buf.count > sizeof(AVRInfoBlock))
				{
					I2C_RX_dequeue();					// remove command byte from buffer, no other Rx data bytes for this command
					readRXbufByteAry(&AVRInfoBlock.PIDFLAG,sizeof(AVRInfoBlock));		// recv struct of bytes, pass first element as pointer
					cutter.spindle_use_pid = AVRInfoBlock.PIDFLAG;  								// fix this double re-assigned variable later
				}
				RecvAVRBlockComplete = 1;																					// flag completion
				break;				
			
			
			case POST_PID_INFO:						// Master is sending an array of 4 floats for Marlin to echo to Serial/USB
				float tau[4];								// This command is ONLY for data logging to a terminal to test PID performance
				if(rx_int_buf.count > sizeof(tau))
				{
					I2C_RX_dequeue();					// remove command byte from buffer, multiple data bytes remain for this command
					readRXbuf(&tau[0]);
					readRXbuf(&tau[1]);
					readRXbuf(&tau[2]);
					readRXbuf(&tau[3]);	
					sprintf(logMsg," T:%1.3f :KP:%5.2f :KI:%5.2f :KD:%5.2f\n", tau[0], tau[1], tau[2], tau[3] );   // to post PID setup info        
	 				SERIAL_ECHO(logMsg);
				}
				break;

			case POST_PID_DATA:						// Master is sending an array of 6 floats for Marlin to echo to Serial/USB
																		// This command is ONLY for data logging to a terminal to test PID performance
				float pidErr, pidProp, pidInt, pidDer, pidOut, cur_adc;
				if(rx_int_buf.count > 24)
				{
					I2C_RX_dequeue();					// remove command byte from buffer, multiple data bytes remain for this command
					readRXbuf(&pidErr);
					readRXbuf(&pidProp);
					readRXbuf(&pidInt);
					readRXbuf(&pidDer);
					readRXbuf(&pidOut);
					readRXbuf(&cur_adc);
					sprintf(logMsg," PV:%5u :SP:%5u :ER:%5.2f :P:%5.2f :I:%5.2f :D:%5.2f :PO:%5.2f :ADC:%5.2f\n",
					        cutter.ACTUAL_RPM, cutter.TARGET_RPM, pidErr, pidProp, pidInt, pidDer, pidOut, cur_adc );           
	 				SERIAL_ECHO(logMsg);			// to post current data to TFT LCD
				}
				break;

			default:
			  break;
		}
	}
	//GPIO_SetValue(4,0x10000000);		//TG- P4_28 pin
	//GPIO_ClearValue(4,0x10000000);	//TG- P4_28 pin
	return;
#endif
#pragma endregion
	
}

// overloaded function to accept float or integer val and fill TRPM_u array
void writeTXbuf(uint8_t len)
{
	uint8_t i=0;
	for(i = 0; i < len; i++)				// send 4 bytes of float data from array
	{
		I2C_TX_enqueue((uint8_t)TRPM_u.a[i]);
	}
}
void writeTXbuf(volatile float val)
{
	TRPM_u.f = val;	// set float in union from actual_rpm	
	writeTXbuf((uint8_t)sizeof(val));
}
void writeTXbuf(volatile uint16_t val)
{
	TRPM_u.i = val;	// set integer in union from actual_rpm	
	writeTXbuf((uint8_t)sizeof(val));
}
void writeTXbufByteAry(volatile uint8_t * src, uint8_t len)
{
	uint8_t i=0;
	for(i = 0; i < len; i++)	
	{
			I2C_TX_enqueue(src[i]);
	}
}

void readRXbuf(volatile float * val)
{
	uint8_t i=0;
	for(i = 0; i < sizeof(float); i++)			// grab 4 bytes of float data into array
	{
		ARPM_u.a[i] = I2C_RX_dequeue();
	}
  *val = ARPM_u.f;												// get float from union float into actual_rpm		
}
void readRXbuf(volatile uint16_t * val)
{
	uint8_t i=0;
	for(i = 0; i < sizeof(uint16_t); i++)		// grab 2 bytes of int data into array
	{
		ARPM_u.a[i] = I2C_RX_dequeue();
	}
	*val = (uint16_t)ARPM_u.i;							// get integer from union	
}
void readRXbufByteAry(volatile uint8_t * dest, uint8_t len)
{
	uint8_t i=0;
	for(i = 0; i < len; i++)	
	{
		dest[i] = I2C_RX_dequeue();
	}
}

//TG Adapted from lpc17xx_i2c code, customized for use here and renamed
i2cStat mySlaveHandler()
{
uint8_t returnCode;
uint32_t timeout;
RetStatus = I2C_OK;
//GPIO_SetValue(4,0x10000000);		//TG- P4_28 pin
//GPIO_ClearValue(4,0x10000000);	//TG- P4_28 pin

handle_state:
	returnCode = (I2CDEV_S->I2STAT & I2C_STAT_CODE_BITMASK);			// get current I2C status reg byte
  // StateHandler returns I2C_OK, I2C_BYTE_RECV, I2C_BYTE_SENT, I2C_SEND_END, I2C_RECV_END, or I2C_STA_STO_R
	RetStatus = myStateHandler(returnCode);		// call Slave State Machine handler with I2CDEV_S->I2STAT
	
	if(I2C_CheckError(RetStatus))							//##### if I2C_ERR or I2C_NAK_RECV	bit 7 = 1 ##### 
		goto s_int_end;					
	else if (RetStatus & I2C_STA_STO_RECV)		//##### STOP condition or repeated START ##### 
	{
		I2C_IntCmd(I2CDEV_S, FALSE);						// Temporally lock the interrupt for timeout condition
		timeout = I2C_SLAVE_TIME_OUT;						// enable time out
		while(1)
		{
			if (I2CDEV_S->I2CONSET & I2C_I2CONSET_SI)	// ##### got a new Interrupt during timeout? #####
			{																					// could be a repeat Start
				I2C_IntCmd(I2CDEV_S, TRUE);							// re-Enable interrupt
				goto handle_state;											// handle new state
			}
			else																	// countdown the timeout
			{
				timeout--;
				if (timeout == 0)										// ##### timed out, it's really a stop condition #####
					goto s_int_end;
			}
		}
	}
	else if(RetStatus & I2C_SEND_END)					// ##### last byte or NACK, transmission ended #####
		goto s_int_end;
	else if(RetStatus & I2C_RECV_END)					// ##### Rx buffer Full, transmission ended #####
	  goto s_int_end;
	else																			// ##### I2C_OK, I2C_BYTE_RECV, I2C_BYTE_SENT ##### 
		return RetStatus;												// still busy ONLY for I2C_OK, I2C_BYTE_RECV, I2C_BYTE_SENT

s_int_end:		// completed, ONLY for I2C_SEND_END, I2C_RECV_END, I2C_STA_STO_R, I2C_ERR, or I2C_NAK_RECV
	I2C_IntCmd(I2CDEV_S, FALSE);							// Disable interrupt
	I2CDEV_S->I2CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC;
	return RetStatus;	
}

//TG Adapted from lpc17xx_i2c code, customized for use here and renamed, process according to I2CDEV_S->I2STAT
i2cStat myStateHandler(uint32_t CodeStatus)
{
i2cStat Ret = I2C_OK;

switch (CodeStatus)
{ // note: if you need pin toggling for scope output use OUT_WRITE(P1_22,1); / OUT_WRITE(P1_22,0);
	
	/******************************************************************************************************/
	/* Slave Reading phase, a Master wants to write to this slave ------------------------------ */
	/******************************************************************************************************/
			
	case I2C_I2STAT_S_RX_SLAW_ACK:	// 0x60	Own SLA+W has been received, ACK has been returned, data byte will be received
	case I2C_I2STAT_S_RX_GENCALL_ACK:  // 0x70 General call address has been received, ACK has been returned, data byte will be received
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;			// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;		// clear the Interrupt flag
		break;
	case I2C_I2STAT_S_RX_ARB_LOST_M_GENCALL:		// 0x78	General Call has been received and ACK has been returned.
	case I2C_I2STAT_S_RX_ARB_LOST_M_SLA:				// 0x68	Arbitration has been lost in Slave Address + R/W bit as bus Master. 
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA|I2C_I2CONSET_STA;	// STA set to restart Master mode
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;		// clear the Interrupt flag
		break;
	/* Receive Data bytes phase, Master sending data to slave  --------------------------------- */
	case I2C_I2STAT_S_RX_PRE_SLA_DAT_ACK:		// 0x80	Previously addressed with own SLA; DATA byte has been received; ACK has been returned
		if(!I2C_RX_buffer_full())	// buffer has room,  data bytes that over-flow the buffer, just ignore them.
		{
			I2C_RX_enqueue((uint8_t)I2CDEV_S->I2DAT);  // read data from I2DAT, inc buffer ptr
			Ret = I2C_BYTE_RECV;
		}
		// typically this would check for last data byte OR buffer full to NAK the master, but in this custom
		// version we don't know how many bytes will be received, we only know when our RX buffer is full!
		if(I2C_RX_buffer_full())	 								// buffer full
		{
			I2CDEV_S->I2CONCLR = I2C_I2CONCLR_AAC|I2C_I2CONCLR_SIC;	 // prepare to NAK master
			Ret = I2C_BYTE_RECV;
		}
		else {																		// not full or the last data byte yet
			I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;		// keep the assert ACK going
			I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;	// clear the Interrupt flag
		}
		break;
	
  /* In this custom version we only get here if our Rx buffer gets full, not when last byte received! */
	/* Received data will not be saved. Not addressed Slave mode is entered */
	case I2C_I2STAT_S_RX_PRE_SLA_DAT_NACK:	// 0x88 Previously addressed with own SLA; DATA byte has been received; NOT ACK has been returned 
	case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_NACK:	// 0x98 Previously addressed with General Call. Data has been received, NOT ACK has been returned.
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;			// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;		// clear the Interrupt flag
		Ret = I2C_RECV_END;
	  break;
	case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_ACK:	// 0x90 Previous General Call, DATA has been received, Only first data byte will be received with ACK. 
		if(!I2C_RX_buffer_full())								// buffer has room
		{
			I2C_RX_enqueue((uint8_t)I2CDEV_S->I2DAT);  // read data from I2DAT, inc ptr
			Ret = I2C_BYTE_RECV;
		}
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_AAC|I2C_I2CONCLR_SIC; 	// No ACK, clear Interrupt, Additional data will be received with NOT ACK.
		break;
	
	
	/******************************************************************************************************/
	/* Slave Writing phase, Master requested data from slave -------------------------------------------- */
	/******************************************************************************************************/
	
	case I2C_I2STAT_S_TX_SLAR_ACK:	// 0xA8 Own SLA+R has been received, ACK has been returned, DATA will be transmitted, ACK will be received
	case I2C_I2STAT_S_TX_DAT_ACK:		// 0xB8 Data has been transmitted, ACK has been received, DATA will be transmitted, ACK will be received
		if(!I2C_TX_buffer_empty())		// buffer has data - send data bytes up to specified transmit data length.
		{	
			I2C_IntCmd(I2CDEV_S, FALSE);
			I2CDEV_S->I2DAT = I2C_TX_dequeue();;
			I2C_IntCmd(I2CDEV_S, TRUE);
			Ret = I2C_BYTE_SENT;
		}
		else																		// buffer is empty
		{
			I2CDEV_S->I2DAT = 0x99;								// send a bogus byte
		}
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;		// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;	// clear the Interrupt flag
		break;
	/* Arbitration lost in Slave Address and R/W bit as bus Master. Own Slave Address + Read
	   has been received, ACK has been returned. Data will be transmitted, ACK bit will be 
	   received. STA is set to restart Master mode after the bus is free again.*/
	case I2C_I2STAT_S_TX_ARB_LOST_M_SLA:	//	0xB0
		if(!I2C_TX_buffer_empty())			// buffer has data - send data bytes
		{	
			I2CDEV_S->I2DAT = I2C_TX_dequeue();
			Ret = I2C_BYTE_SENT;
		}
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA|I2C_I2CONSET_STA;	// STA set to restart Master mode
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;									// clear the Interrupt flag
		break;
	
	/*
	 * Note: Don't wait for stop event in slave transmit mode, since there no proof to inform
	 * us when a stop signal has been received on slave side.
	 */
	
	/* Last byte has been transmitted, NACK has been received, enter Not addressed Slave mode */
	case I2C_I2STAT_S_TX_LAST_DAT_ACK:	//	0xC8
	/* Data has been transmitted, NOT ACK has been received. Not addressed Slave mode is entered */
	case I2C_I2STAT_S_TX_DAT_NACK:			//	0xC0
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;					// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;					// clear the Interrupt flag
		Ret = I2C_SEND_END;
		break;
	/*
	 * Note that: Return code only let us know a stop condition mixed with a repeat start condition in the
	 * same code value. So we should provide a time-out. In case this is really a stop condition, this will
	 * return back after time out condition. Otherwise, next session that is slave receive data will be completed.
	 */
	/* A STOP condition or repeated START has been received, while still addressed as a 
	   Slave. Data will not be saved. Not addressed Slave mode is entered. */
	case I2C_I2STAT_S_RX_STA_STO_SLVREC_SLVTRX:	// 0xA0		## 1.4us to end of ISR
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;					// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;				// clear the Interrupt flag
		Ret = I2C_STA_STO_RECV;
		break;
	case I2C_I2STAT_NO_INF:	//	0xF8 No status information, Other status must be captured
	default:
		I2CDEV_S->I2CONSET = I2C_I2CONSET_AA;					// keep the assert ACK going
		I2CDEV_S->I2CONCLR = I2C_I2CONCLR_SIC;				// clear the Interrupt flag
		break;
	}

return Ret;
}

//TG custom version adapted from lpc17xx_i2c and renamed, customized for use here 
#pragma region I2C Initialization
void I2C_begin(en_I2C_Mode mode, uint8_t address, uint32_t speed, bool StartInterrupts){
	/* Init I2C pin connections according to which peripheral I2C0, 1, or 2 */
	PINSEL_CFG_Type PinCfg;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;		//TG 8/9/22 changed to 1, was 0
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	#if USEDI2CDEV_M == 0 || USEDI2CDEV_S == 0
	  PinCfg.Funcnum = 1;
	  PinCfg.Pinnum = 27;
	  PinCfg.Portnum = 0;
	  PINSEL_ConfigPin(&PinCfg); // SDA0 / D57  AUX-1
	  PinCfg.Pinnum = 28;
	  PINSEL_ConfigPin(&PinCfg); // SCL0 / D58  AUX-1
	#endif
	#if USEDI2CDEV_M == 1 || USEDI2CDEV_S == 1
		PinCfg.Funcnum = 3;														// alt function 3 = SDA1, SCL1 
	  PinCfg.Pinnum = 0;														// P0.0 = SDA1
	  PinCfg.Portnum = 0;														// Port 0
	  PINSEL_ConfigPin(&PinCfg);  									// SDA1 / D20 SCA
	  PinCfg.Pinnum = 1;														// P0.1 = SCL1
	  PINSEL_ConfigPin(&PinCfg);  									// SCL1 / D21 SCL
	#endif
	#if USEDI2CDEV_M == 2 || USEDI2CDEV_S == 2
	  PinCfg.Funcnum = 2;
	  PinCfg.Pinnum = 10;
	  PinCfg.Portnum = 0;
	  PINSEL_ConfigPin(&PinCfg); // SDA2 / D38  X_ENABLE_PIN
	  PinCfg.Pinnum = 11;
	  PINSEL_ConfigPin(&PinCfg); // SCL2 / D55  X_DIR_PIN
	#endif

	if(mode == I2C_MASTER_MODE){                		// set up Master mode config
	  // Initialize I2C peripheral power and clock setup
		I2C_Init(I2CDEV_M, speed);
		// Enable Master I2C operation
	  I2C_Cmd(I2CDEV_M, I2C_MASTER_MODE, ENABLE);
	}
	
	if(mode == I2C_SLAVE_MODE){                 		// set up Slave mode config structure
	  // Initialize I2C peripheral power and clock setup
		I2C_Init(I2CDEV_S, speed);

		// Configure the Slave Own Address type
		I2C_OWNSLAVEADDR_CFG_Type mySLAVEADDR;
	  mySLAVEADDR.SlaveAddr_7bit = address;     		// the slave device's own address
	  mySLAVEADDR.SlaveAddrMaskValue = 0x00; 
	  mySLAVEADDR.SlaveAddrChannel = 0;         		// can be up to four different addresses
	  mySLAVEADDR.GeneralCallState = ENABLE;
	  I2C_SetOwnSlaveAddr(I2CDEV_S, &mySLAVEADDR);	// Set slave address
	  I2C_Cmd(I2CDEV_S, I2C_SLAVE_MODE, ENABLE); 		// Enable Slave I2C operation
	}
	
	#ifdef FULL_INTERRUPT
	// set interrupt priority 4 for I2C, must not be higher than TIMERS or UART to avoid stuttering motion
	// Marlin IRQ priority defaults to WDTIMER=0, TIMER0=1, TIMER1=2, UART = 3, and SERVO=5
	if(StartInterrupts){
		if(I2CDEV_S == LPC_I2C0)								
		{																				
			NVIC_SetPriority(I2C0_IRQn, 4);
		}
		else if (I2CDEV_S == LPC_I2C1)
		{
			NVIC_SetPriority(I2C1_IRQn, 4);
		}
		else if (I2CDEV_S == LPC_I2C2)
		{
			NVIC_SetPriority(I2C2_IRQn, 4);
		}
		
		I2C_IntCmd(I2CDEV_S, TRUE);								// start I2C interrupts
	}
  #endif
}
#pragma endregion
						
#pragma region Circular Buffer Routines

uint8_t I2C_RX_peek(){
		if(I2C_RX_buffer_empty()==false)						// if buffer not empty
			return rx_int_buf.data[rx_int_buf.head];	// get from queue
		else
			return 0;
}
void I2C_RX_enqueue(uint8_t byte){
		if(I2C_RX_buffer_full()==false)							// if buffer not full
		{
			rx_int_buf.data[rx_int_buf.tail] = byte;	// put in queue at tail
			rx_int_buf.tail = (rx_int_buf.tail + 1) % I2C_BUF_SIZE;	// bump tail
			rx_int_buf.count = (rx_int_buf.count + 1) % I2C_BUF_SIZE;
		}
}
uint8_t I2C_RX_dequeue(){
		uint8_t retbyte;
		if(I2C_RX_buffer_empty()==false)						// if buffer not empty
		{
			retbyte = rx_int_buf.data[rx_int_buf.head];			// get from queue
			rx_int_buf.data[rx_int_buf.head] = 0;						// clear it
			rx_int_buf.head = (rx_int_buf.head + 1) % I2C_BUF_SIZE;	// bump head
			rx_int_buf.count = (rx_int_buf.count - 1) % I2C_BUF_SIZE;
			return retbyte;
		}
		else
			return 0;
}

void I2C_TX_enqueue(uint8_t byte){
		if(I2C_TX_buffer_full()==false)						// if buffer not full
		{
			tx_int_buf.data[tx_int_buf.tail] = byte;			// put in queue at tail
			tx_int_buf.tail = (tx_int_buf.tail + 1) % I2C_BUF_SIZE;	// bump tail
			tx_int_buf.count = (tx_int_buf.count + 1) % I2C_BUF_SIZE;
		}
}
uint8_t I2C_TX_dequeue(){
		uint8_t retbyte;
		if(I2C_TX_buffer_empty()==false)						// if buffer not empty
		{
			retbyte = tx_int_buf.data[tx_int_buf.head];		// get from queue at head
			tx_int_buf.data[tx_int_buf.head] = 0;					// clear from queue
			tx_int_buf.head = (tx_int_buf.head + 1) % I2C_BUF_SIZE;	  // bump head
			tx_int_buf.count = (tx_int_buf.count - 1) % I2C_BUF_SIZE; // dec count
			return retbyte;
		}
		else
			return 0;
}

bool I2C_RX_buffer_full(){
	if(rx_int_buf.head ==	(rx_int_buf.tail + 1) % I2C_BUF_SIZE){	// full if tail + 1 will hit head
		rx_int_buf.count = I2C_BUF_SIZE;

//		GPIO_SetValue(4,0x10000000);		//TG- P4_28 pin
//		DELAY_US(5);
//		GPIO_ClearValue(4,0x10000000);	//TG- P4_28 pin		

		return true;
	}
	else
		return false;
}
bool I2C_TX_buffer_full(){		// full if tail + 1 will hit head
	if(tx_int_buf.head ==	(tx_int_buf.tail + 1) % I2C_BUF_SIZE){	
		tx_int_buf.count = I2C_BUF_SIZE;
		return true;
	}
	else
		return false; 
} 
bool I2C_RX_buffer_empty(){		// empty if head = tail
	if(rx_int_buf.head == rx_int_buf.tail){				
		rx_int_buf.count = 0;
		return true; 
	}
	else
		return false;
}
bool I2C_TX_buffer_empty(){		// empty if head = tail
	if(tx_int_buf.head == tx_int_buf.tail){				
		tx_int_buf.count = 0;
		
//		GPIO_SetValue(2,0x00001000);		//TG- P2_12 pin 
//		DELAY_US(5);
//		GPIO_ClearValue(2,0x00001000);	//TG- P2_12 pin 
		
		return true; 
	}
	else
		return false;
}

#pragma endregion

#endif	// #ifdef TG_I2C_SUPPORT
