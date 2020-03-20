/*
 * spi.c
 *
 * Created: 2/13/2020 4:14:29 PM
 *  Author: Rowan
 */ 
#include <avr/io.h>
#include "spi.h"

void spi_init(void)
{
	PORTD.DIRSET  =  SPI_SCK_bm|SPI_MOSI_bm|SPI_SS_bm;
	PORTD.DIRCLR  =  SPI_MISO_bm;
	PORTD.PIN6CTRL	= PORT_OPC_PULLDOWN_gc;
	PORTD.OUTSET  =  SPI_SS_bm;
	SPID.CTRL     =  SPI_ENABLE_bm |         // enable SPI
	SPI_MASTER_bm |         // master mode
	// SPI_CLK2X_bm  |         // no double clock speed
	//SPI_DORD_bm   |         // MSB first
	SPI_MODE_3_gc |         // SPI mode 3
	SPI_PRESCALER_DIV4_gc;  // prescaling 4
}

uint8_t spi_transfer(uint8_t data)
{
	SPID.DATA = data;
	while(!(SPID.STATUS & (SPI_IF_bm)));

	return SPID.DATA;
}

void spi_write(uint8_t data)
{
	spi_transfer(data);
}

uint8_t spi_read(void)
{
	uint8_t data;
	data = spi_transfer(FOO);
	return data;
}
