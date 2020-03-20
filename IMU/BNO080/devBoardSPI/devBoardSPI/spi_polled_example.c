/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA SPI polled driver example source.
 *
 *      This file contains an example application that demonstrates the polled
 *      SPI drivers.
 *
 * \par Application note:
 *      AVR1309: Using the XMEGA SPI
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 5407 $
 * $Date: 2011-10-12 14:53:14 +0200 (on, 12 okt 2011) $  \n
 *
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.

 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.

 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "avr_compiler.h"
#include "spi_driver.h"


/*! \brief The number of test data bytes. */
#define NUM_BYTES			4
#define ID_REQUEST_BYTES	2

/* Global variables */

/*! \brief SPI master module on PORT D. */
SPI_Master_t spiMasterD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0x11, 0x22, 0x33, 0x44};
	
/*! \BNO080 product ID Reqquest. */
uint8_t BNO080_IDrequest[ID_REQUEST_BYTES] = {0xF9, 0x00};

/*! \brief Data received from slave. */
uint8_t masterReceivedData[32];

/*! \brief Result of the test. */
bool success = true;



/*! \brief Test function.
 *
 *  This function tests the SPI master drivers in polled operation,
 *  with a master (on port D).
 *
 *  Hardware setup:
 *
 *    - Connect PD4  (SS)
 *    - Connect PD5  (MOSI)
 *    - Connect PD6  (MISO)
 *    - Connect PD7  (SCK)
 *
 *  The drivers are tested in two phases:
 *
 *  1: Data is transmitted on byte at a time from the master to the slave.
 *     The slave increments the received data and sends it back. The master reads
 *     the data from the slave and verifies that it equals the data sent + 1.
 *
 *  2: Data is transmitted 4 bytes at a time to the slave. As the master sends
 *     a byte to the slave, the preceding byte is sent back to the master.
 *     When all bytes have been sent, it is verified that the last 3 bytes
 *     received at the master, equal the first 3 bytes sent.
 *
 *  The variable, 'success', will be non-zero when the function reaches the
 *  infinite for-loop if the test was successful.
 */
int main(void)
{
	/* Init SS pin as output with wired AND and pull-up. */
	PORTD.DIRSET = PIN4_bm;
	PORTD.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTD.OUTSET = PIN4_bm;

	/* Instantiate pointer to ssPort. */
	PORT_t *ssPort = &PORTD;

	/* Initialize SPI master on port D. */
	SPI_MasterInit(&spiMasterD,
	               &SPID,
	               &PORTD,
	               false,
	               SPI_MODE_0_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV4_gc);
	
	
	/* MASTER: Release SS to slave. */
	SPI_MasterSSHigh(ssPort, PIN4_bm);

	/* PHASE 1: Transceive data packet. */

	/* Create data packet (SS to slave by PD4). */
	SPI_MasterCreateDataPacket(&dataPacket,
	                           BNO080_IDrequest,
	                           masterReceivedData,
	                           NUM_BYTES,
	                           &PORTD,
	                           PIN4_bm);

	/* Transceive packet. */
	SPI_MasterTransceivePacket(&spiMasterD, &dataPacket);

	/* Check that correct data was received. Assume success at first. */
	if (masterReceivedData[0] == 0xF8) {
		PORTC.DIRSET = PIN0_bm;
		PORTC.OUTSET = PIN0_bm;
	}
	while(true) {
		nop();
	}
}
