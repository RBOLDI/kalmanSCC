/*
 * beunBoard_controlSerial.c
 *
 * Created: 9/16/2020 12:46:57 PM
 *  Author: Rowan
 */ 
#define F_CPU     2000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <util/delay.h>
#include "beunBoard_controlSerial.h"

static volatile uint8_t rx_ctrl_wridx, rx_ctrl_rdidx, rx_ctrl_buf[RXBUF_DEPTH_CTRL];

void set_usartctrl (USART_t *usart, uint8_t bscale, uint16_t  bsel);

void initSerial(void);
uint8_t CanRead_Ctrl(void);
uint8_t ReadByte_Ctrl(void);

void set_usartctrl (USART_t *usart, uint8_t bscale, uint16_t  bsel)
{
	usart->BAUDCTRLA = (bsel & USART_BSEL_gm);
	usart->BAUDCTRLB = ((bscale << USART_BSCALE0_bp) & USART_BSCALE_gm) |
	((bsel >> 8) & ~USART_BSCALE_gm);
}

void uart_bscale_bsel(USART_t *usart, int8_t bscale, int16_t bsel)
{
	usart->CTRLA =	USART_RXCINTLVL_LO_gc;         // RXD interrupt
	usart->CTRLB =	USART_TXEN_bm				| USART_RXEN_bm;
	usart->CTRLC =	USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	set_usartctrl(usart, bscale, bsel);
}

void initSerial(void){
	printf("init serial\n");
	PORTE.DIRSET = PIN3_bm;                     // TXD
	PORTE.DIRCLR = PIN2_bm;                     // RXD
	uart_bscale_bsel(&USARTE0, -7, 11);			// Baudrate 115200`
}

uint8_t CanRead_Ctrl(void) {
	uint8_t wridx = rx_ctrl_wridx, rdidx = rx_ctrl_rdidx;
	
	if(wridx >= rdidx)
	return wridx - rdidx;
	else
	return wridx - rdidx + RXBUF_DEPTH_CTRL;
	
} /* CanRead_Ctrl */

uint8_t ReadByte_Ctrl(void) {
	uint8_t res, curSlot, nextSlot;
		
	curSlot = rx_ctrl_rdidx;
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanRead_xxx() first */
	while(!CanRead_Ctrl()) ;
		
	res = rx_ctrl_buf[curSlot];

	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_CTRL)
	nextSlot = 0;
	rx_ctrl_rdidx = nextSlot;
		
	return res;
} /* ReadByte_Ctrl */

ISR(USARTE0_RXC_vect) {
	uint8_t curSlot, nextSlot;
	
	curSlot = rx_ctrl_wridx;
	rx_ctrl_buf[curSlot] = USARTE0.DATA;
	//printf("%c", rx_ctrl_buf[curSlot]);
	
	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_CTRL)
	nextSlot = 0;
	
	if(nextSlot != rx_ctrl_rdidx)
	rx_ctrl_wridx = nextSlot;
}