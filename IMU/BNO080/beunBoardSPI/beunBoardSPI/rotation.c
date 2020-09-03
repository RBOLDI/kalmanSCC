/*
 * rotation.c
 *
 * Created: 9/2/2020 7:22:20 PM
 *  Author: Rowan
 */ 
#include "rotation.h"
#include <avr/io.h>
#include <math.h>
#include <stdio.h>

float rotationVector[4];			//is expressed as a quaternion referenced to magnetic north and gravity
float rotationMatrix[3][3];			//is expressed as a 3x3 matrix referenced to magnetic north and gravity
float inverseRotationMatrix[3][3];	//is expressed as a 3x3 matrix, needed to rotate the acceleration vector

float accelNorth = 0;
float accelEast = 0;
float acceldown = 0;

void updateRotationMatrix(float *quaternion);
void update_AccelNorth(float *accelVector, float *accelVector_refNorth);
void matrixOfMinors(float matrix[3][3], float minorMatrix[3][3]);
void cofactorMatrix(float minorMatrix[3][3]);
void inverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant);
void rotateAccel(float *accel, float inverseRotMatrix[3][3], float *accelRotated);

void updateRotationMatrix(float *quaternion){
	//https://fabiensanglard.net/doom3_documentation/37726-293748.pdf
	//first column:
	rotationMatrix[0][0] = 1 - (2*pow(quaternion[Q_Y], 2)) - (2*pow(quaternion[Q_Z],2));				//1 - 2y² - 2z² 
	rotationMatrix[0][1] = (2*quaternion[Q_X]*quaternion[Q_Y]) + (2*quaternion[Q_W]*quaternion[Q_Z]);	//2xy + 2wz
	rotationMatrix[0][2] = (2*quaternion[Q_X]*quaternion[Q_Z]) - (2*quaternion[Q_W]*quaternion[Q_Y]);	//2xz - 2wy
	
	//second column:
	rotationMatrix[1][0] = (2*quaternion[Q_X]*quaternion[Q_Y]) - (2*quaternion[Q_W]*quaternion[Q_Z]);	//2xy - 2wz
	rotationMatrix[1][1] = 1 - (2*pow(quaternion[Q_X], 2)) - (2*pow(quaternion[Q_Z],2));				//1 - 2x² - 2z²
	rotationMatrix[1][2] = (2*quaternion[Q_Y]*quaternion[Q_Z]) + (2*quaternion[Q_W]*quaternion[Q_X]);	//2yz + 2wx
	
	//third column:
	rotationMatrix[2][0] = (2*quaternion[Q_X]*quaternion[Q_Z]) + (2*quaternion[Q_W]*quaternion[Q_Y]);	//2xz + 2wy
	rotationMatrix[2][1] = (2*quaternion[Q_Y]*quaternion[Q_Z]) - (2*quaternion[Q_W]*quaternion[Q_X]);	//2yz - 2wx
	rotationMatrix[2][2] = 1 - (2*pow(quaternion[Q_X], 2)) - (2*pow(quaternion[Q_Y],2));				//1 - 2x² - 2y²
}

void update_AccelNorth(float *accelVector, float *accelVector_refNorth){
	//Inverse matrix = (1/determinat)* adjugate matrix
	float determinant = 0;
	float inverse_Matrix[3][3];
	float copyMatrix[3][3];
	matrixOfMinors(rotationMatrix, copyMatrix);						//determine matrix of minors
	cofactorMatrix(copyMatrix);										//determine cofactor matrix	
	determinant = determinantCalc(rotationMatrix);					//calculate the determinant
	adjugateMatrix(copyMatrix);										//determine adjugate matrix
	inverseMatrix(copyMatrix, inverse_Matrix, determinant);			//fill inverseMatrix
	rotateAccel(accelVector, inverse_Matrix, accelVector_refNorth);
}

void matrixOfMinors(float matrix[3][3], float minorMatrix[3][3]){
	int index = 0;
	float miniMatrix[4];	
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			//search within the matrix to the values needed to determine the minor
			for (int ii = 0; ii < 3; ii++){
				for(int jj = 0; jj < 3; jj++){
					if(ii != i && jj != j){
						miniMatrix[index++] = matrix[ii][jj];
					}
				}
			}
			index = 0;
			minorMatrix[i][j] = miniMatrix[0]*miniMatrix[3] - miniMatrix[1]*miniMatrix[2];
		}
	}
}

void cofactorMatrix(float minorMatrix[3][3]){
	int cofactor = 1;
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			minorMatrix[i][j] = minorMatrix[i][j]*cofactor;
			cofactor = cofactor*-1;
		}
	}
}

float determinantCalc(float matrix[3][3]){
	  return (matrix[0][0] * ((matrix[1][1]*matrix[2][2]) - (matrix[2][1]*matrix[1][2])) -matrix[0][1] * (matrix[1][0]
		    * matrix[2][2] - matrix[2][0] * matrix[1][2]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1]));
}

void adjugateMatrix(float cofactor_Matrix[3][3]){
	float adjugate_Matrix[3][3];
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			adjugate_Matrix[i][j] = cofactor_Matrix[j][i];
		}
	}
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			cofactor_Matrix[i][j] = adjugate_Matrix[i][j];
		}
	}
}

void inverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant){
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			inverse_Matrix[i][j] = (1/determinant)*adjugate_Matrix[i][j];
		}
	}
}

void rotateAccel(float *accel, float inverseRotMatrix[3][3], float *accelRotated){
	for (int i = 0; i < 3; i++) {
		accelRotated[i] = 0;
	}
	//multiply accelVector by the inverse rotation matrix.
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			accelRotated[i] = accelRotated[i]+(inverseRotMatrix[i][j]*accel[j]);
		}
	}
}
