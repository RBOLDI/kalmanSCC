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
	
	float accel[3];					//is expressed as a vector of acceleration referenced to the sensor's frame
	float accelNorth[3];				//is expressed as a vector of acceleration referenced to the geographic frame (referenced to magnetic north)
	float rotationQuat[4];			//is expressed as a quaternion referenced to magnetic north and gravity
	uint8_t linAccuracy = 0;
	float rotationVectorAccuracy = 0;
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
		BNO080enableLinearAccelerometer(100);
		BNO080enableRotationVector(50);
	}
	//Process when in Operation mode
	while(BNO080mode == OPERATION_MODE) {
		if (BNO080dataAvailable() && (newDataReport == SENSOR_REPORTID_LINEAR_ACCELERATION)) {
			accel[_X_] = BNO080getLinAccelX();
			accel[_Y_] = BNO080getLinAccelY();
			accel[_Z_] = BNO080getLinAccelZ();
			linAccuracy = BNO080getLinAccelAccuracy();
			printf("x: %0.2f, y: %0.2f, z: %0.2f, accuratie: %d \n", accel[_X_], accel[_Y_], accel[_Z_], linAccuracy);
			update_AccelNorth(accel, accelNorth);
			printf("N: %0.2f, E: %0.2f, D: %0.2f\n", accelNorth[_X_], accelNorth[_Y_], accelNorth[_Z_]);
		}
		
		if (BNO080dataAvailable() && (newDataReport == SENSOR_REPORTID_ROTATION_VECTOR)) {
			rotationQuat[Q_X] = BNO080getQuatI_X();
			rotationQuat[Q_Y] = BNO080getQuatJ_Y();
			rotationQuat[Q_Z] = BNO080getQuatK_Z();
			rotationQuat[Q_W] = BNO080getQuatReal_W();
			rotationVectorAccuracy = BNO080getQuatRadianAccuracy();
			printf("x: %0.2f, y: %0.2f, z: %0.2f, w: %0.2f accuratie: %0.2f \n", rotationQuat[Q_X], rotationQuat[Q_Y], rotationQuat[Q_Z], rotationQuat[Q_W], rotationVectorAccuracy);
			updateRotationMatrix(rotationQuat);
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
