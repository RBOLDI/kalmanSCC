/*
 * beunBoard_controlSerial.h
 *
 * Created: 9/16/2020 12:47:10 PM
 *  Author: Rowan
 */ 


#ifndef BEUNBOARD_CONTROLSERIAL_H_
#define BEUNBOARD_CONTROLSERIAL_H_

#define RXBUF_DEPTH_CTRL			250
#define MAXCHARACTERSSENTENCE		83
#define UART_115K2_BSEL_VALUE		1047
#define UART_115K2_BSCALE_VALUE		0x0A
#define PD_UART_E0_RXD_CTRL			PIN2_bm
#define PD_UART_E0_TXD_CTRL			PIN3_bm

void initSerial(void);
uint8_t CanRead_Ctrl(void);
uint8_t ReadByte_Ctrl(void);

#endif /* BEUNBOARD_CONTROLSERIAL_H_ */