#ifndef __communicate__H
#define __communicate__H

#include "gpio.h"
#include "tim.h"
#include "crc.h"
#include "usart.h"
#include "head.h"

extern struct ROBOT robot;

#define SPIBUFFERSIZE 6
#define UARTBUFFERSIZE 10
#define BoardID 0x31


union uart_data_buffer{
	volatile char starter_buf;
	volatile char buf8_t[100];
	volatile uint32_t buf32_t[15];
};

struct UART
{
	UART_HandleTypeDef 	*huart;
	uint8_t				tx_length, rx_length, start;			// count as 32-bits, without crc
	uint32_t			trans_count, recev_count, error_count;
	volatile int32_t	tx[15];
//	volatile int32_t	rx[15];
//	volatile char	    buf[100];
	union uart_data_buffer     data_buffer;
	volatile char		starter_signal[4];
	volatile int   		starter_number;
//	volatile int32_t 	reset_check[15];
//	volatile int32_t	checked_rx[15];
//	volatile int32_t    rx_single[2];
//	int flag;
//	int single_count;
	bool dataValid;
	double UartFreq;
//	int one_or_block;
	bool healthy_communication;
	int byteCount;
	uint32_t crc_value;
	int getStarter;
};


struct SPI{
	uint32_t spiTx[SPIBUFFERSIZE];
	uint32_t spiRx[SPIBUFFERSIZE];
	double spiGet;
	double spiWrong;
	double spiTransmitCount;
	double spiCrc;
};

struct Communicate{

	int FlagCmdSend;
	// variables for spi
	struct SPI spi;
	struct UART uart;

};


void Uart_Transmit(struct ROBOT * robot);
void Uart_Rate_Count(struct ROBOT* robot);
short Uart_Crc_Check(struct ROBOT* robot);
void Uart_RxCplt(struct ROBOT* robot);
void Tx_Data_Update(struct ROBOT * robot);
void getData(struct ROBOT * robot);

#endif
