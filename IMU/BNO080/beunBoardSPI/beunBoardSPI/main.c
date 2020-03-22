/*
 * BNO080.c
 *
 * Created: 2/6/2020 3:58:45 PM
 *  Author: Rowan
 
 SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
*/

#include "BNO080.h"

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
	//Process when in Operation mode
	while(BNO080mode == OPERATION_MODE) {
		if (BNO080dataAvailable() && (newDataReport == SENSOR_REPORTID_LINEAR_ACCELERATION)) {
			x = BNO080getLinAccelX();
			y = BNO080getLinAccelY();
			z = BNO080getLinAccelZ();
			linAccuracy = BNO080getLinAccelAccuracy();
			printf("x: %0.2f, y: %0.2f, z: %0.2f, accuratie: %d \n", x, y, z, linAccuracy);
		}
	}
	//Process when in Calibration mode
	while(BNO080mode == CALIBRATION_MODE) {
		if (BNO080dataAvailable())
		{
		}	
	} 
}
