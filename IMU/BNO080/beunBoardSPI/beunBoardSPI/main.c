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

#include "BNO080.h"

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
	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t linAccuracy = 0;
	/* Instantiate pointer to ssPort. */
	init_stream(F_CPU);
	sei();
	if(!initBNO080()) return 0;
	
	if(BNO080mode == CALIBRATION_MODE){
		//Calibrate all
		BNO080sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
		BNO080enableAccelerometer(100);
		BNO080enableGyro(100);
		BNO080enableMagnetometer(100);
	}
	else if(BNO080mode == OPERATION_MODE){
		BNO080enableLinearAccelerometer(1000);
	}
	
	while(1) {
		if (BNO080dataAvailable())
		{
			if (BNO080mode == OPERATION_MODE) {
				if (newDataReport == SENSOR_REPORTID_LINEAR_ACCELERATION) {
					x = BNO080getLinAccelX();
					y = BNO080getLinAccelY();
					z = BNO080getLinAccelZ();
					linAccuracy = BNO080getLinAccelAccuracy();
					printf("x: %0.2f, y: %0.2f, z: %0.2f, accuratie: %d \n", x, y, z, linAccuracy);
				}
			}
			else if (BNO080mode == CALIBRATION_MODE) {
			}
		}	
	} 
}
