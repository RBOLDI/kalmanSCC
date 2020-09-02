/*
 * BNO080.c
 *
 * Created: 2/6/2020 3:58:45 PM
 *  Author: Rowan
 
 SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
*/
#include <stdio.h>
#include "BNO080.h"
#include "rotation.h"

int main(void)
{
	PORTD.DIRSET = PIN2_bm;
	PORTD.OUTSET = PIN2_bm;
	
	float x = 0;
	float y = 0;
	float z = 0;
	
	float rotationQuatX = 0;
	float rotationQuatY = 0;
	float rotationQuatZ = 0;
	float rotationQuatW = 0;
	
	uint8_t linAccuracy = 0;
	float rotationVectorAccuracy = 0;
	/* Instantiate pointer to ssPort. */
	init_stream(F_CPU);
	sei();
	//Test of inverse matrix function
		float matrix[3][3] = {	{-1, -2, 2},
		{2, 1, 1},
		{3, 4, 5}};
		float inverse[3][3];
		inverseMatrix(matrix, inverse);
	//end of test	
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
		BNO080enableRotationVector(1000);
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
		
		if (BNO080dataAvailable() && (newDataReport == SENSOR_REPORTID_ROTATION_VECTOR)) {
			rotationQuatX = BNO080getQuatI_X();
			rotationQuatY = BNO080getQuatJ_Y();
			rotationQuatZ = BNO080getQuatK_Z();
			rotationQuatW = BNO080getQuatReal_W();
			rotationVectorAccuracy = BNO080getQuatRadianAccuracy();
			printf("x: %0.2f, y: %0.2f, z: %0.2f, w: %0.2f accuratie: %0.2f \n", rotationQuatX, rotationQuatY, rotationQuatZ, rotationQuatW, rotationVectorAccuracy);
		}
	}
	//Process when in Calibration mode
	while(BNO080mode == CALIBRATION_MODE) {
		if (BNO080dataAvailable())
		{
			BNO080calibrationRoutine();
		}	
	} 
}
