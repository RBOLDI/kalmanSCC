/*
 * rotation.c
 *
 * Created: 9/2/2020 7:22:20 PM
 *  Author: Rowan
 */ 
#include "rotation.h"
#include <avr/io.h>
#include <math.h>

#define Q_X	0
#define Q_Y	1
#define Q_Z 2
#define Q_W 3

float rotationVector[4];		//is expressed as a quaternion referenced to magnetic north and gravity
float rotationMatrix[3][3];		//is expressed as a 3x3 matrix referenced to magnetic north and gravity

void fillRotationVector(float quatX, float quatY, float quatZ, float quatW){
	rotationVector[Q_X] = quatX;
	rotationVector[Q_Y] = quatY;
	rotationVector[Q_Z] = quatZ;
	rotationVector[Q_W] = quatW;
}

void fillRotationMatrix(float *quaternion, float **rotMatrix){
	https://fabiensanglard.net/doom3_documentation/37726-293748.pdf
	//first column:
	rotMatrix[0][0] = 1 - (2*pow(quaternion[Q_Y], 2)) - (2*pow(quaternion[Q_Z],2));					//1 - 2y² - 2z² 
	rotMatrix[0][1] = (2*quaternion[Q_X]*quaternion[Q_Y]) - (2*quaternion[Q_W]*quaternion[Q_Z]);	//2xy - 2wz
	rotMatrix[0][2] = (2*quaternion[Q_X]*quaternion[Q_Z]) + (2*quaternion[Q_W]*quaternion[Q_Y]);	//2xz + 2wy
	
	//second column:
	rotMatrix[1][0] = (2*quaternion[Q_X]*quaternion[Q_Y]) + (2*quaternion[Q_W]*quaternion[Q_Z]);	//2xy + 2wz
	rotMatrix[1][1] = 1 - (2*pow(quaternion[Q_X], 2)) - (2*pow(quaternion[Q_Z],2));					//1 - 2x² - 2z²
	rotMatrix[1][2] = (2*quaternion[Q_Y]*quaternion[Q_Z]) - (2*quaternion[Q_W]*quaternion[Q_X]);	//2yz - 2wx
	
	//third column:
	rotMatrix[2][0] = (2*quaternion[Q_X]*quaternion[Q_Z]) - (2*quaternion[Q_W]*quaternion[Q_Y]);	//2xz - 2wy
	rotMatrix[0][2] = (2*quaternion[Q_Y]*quaternion[Q_Z]) + (2*quaternion[Q_W]*quaternion[Q_X]);	//2yz + 2wx
	rotMatrix[1][1] = 1 - (2*pow(quaternion[Q_X], 2)) - (2*pow(quaternion[Q_Y],2));					//1 - 2x² - 2y²
}

