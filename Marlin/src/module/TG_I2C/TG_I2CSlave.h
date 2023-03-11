/** //TG MODIFIED BY T.GIOIOSA
 * Based on LPC17xx_I2C.c and adapted for Marlin using I2C1 in Slave Mode to send/receive rpm data
 * T. Gioiosa 5/12/22
 * 
 */
#ifndef _TG_I2CSLAVE_H_
#define _TG_I2CSLAVE_H_
#include <stdint.h>
#include <lpc17xx_i2c.h>

//TG 8/16/22 max about 550000 where 2004A LCD display starts to fail
//depends on lead lenths of jumper wires, 400Khz gives a little margin
#define I2C_CLOCK  400000		

#define READ_TARGET_RPM			0x30	// Master commands over I2C
#define WRITE_ACTUAL_RPM 		0x50
#define POST_PID_INFO 			0x41
#define POST_PID_DATA 			0x42
#define SEND_PID_CONSTANTS 	0x51
#define RCV_PID_CONSTANTS 	0x52
#define SEND_AVR_INFO_BLOCK	0x60
#define RCV_AVR_INFO_BLOCK	0x61

#define I2C_BUF_SIZE 				64

#define FULL_INTERRUPT
//#define BLOCKING_INTERRUPT
//#define FULL_POLLING

extern volatile uint16_t TARGET_RPM;
extern volatile uint16_t ACTUAL_RPM;

volatile typedef struct {
		__IO uint8_t data[I2C_BUF_SIZE];			// the buffer
		__IO int8_t head;											// write pointer			
		__IO int8_t tail;											// read pointer
		__IO uint8_t count;
}I2C_RX_buffer;

volatile typedef struct {
		__IO uint8_t data[I2C_BUF_SIZE];			// the buffer
		__IO int8_t head;											// write pointer	
		__IO int8_t tail;											// read pointer
		__IO uint8_t count;
}I2C_TX_buffer;

#define RXBUF_EMPTY (rx_int_buf.count==0)
#define RXBUF_FULL (rx_int_buf.count==I2C_BUF_SIZE)
#define TXBUF_EMPTY (tx_int_buf.count==0)
#define TXBUF_FULL (tx_int_buf.count==I2C_BUF_SIZE)

void processI2Cpacket(void);
void I2C_begin(en_I2C_Mode mode, uint8_t address, uint32_t speed, bool StartInterrupts);
uint8_t I2C_RX_peek();
uint8_t I2C_RX_dequeue();
void I2C_TX_enqueue(uint8_t byte);
uint8_t I2C_TX_dequeue();
bool I2C_RX_buffer_full();
bool I2C_RX_buffer_empty();
void I2C_RX_enqueue(uint8_t byte);
bool I2C_TX_buffer_full();
bool I2C_TX_buffer_empty();

void writeTXbuf(uint8_t len);
void writeTXbuf(volatile float val);
void writeTXbuf(volatile uint16_t val);
void writeTXbufByteAry(volatile uint8_t * src, uint8_t len);

void readRXbuf(volatile float * val);
void readRXbuf(volatile uint16_t * val);
void readRXbufByteAry(volatile uint8_t * dest, uint8_t len);


i2cStat myStateHandler(uint32_t CodeStatus);
i2cStat mySlaveHandler();

typedef enum {
	TRANSMIT,
	RECEIVE,
} I2C_dir_type;





#endif