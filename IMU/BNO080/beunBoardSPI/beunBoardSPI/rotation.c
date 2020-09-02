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

#define Q_X	0
#define Q_Y	1
#define Q_Z 2
#define Q_W 3

float rotationVector[4];			//is expressed as a quaternion referenced to magnetic north and gravity
float rotationMatrix[3][3];			//is expressed as a 3x3 matrix referenced to magnetic north and gravity
float inverseRotationMatrix[3][3];	//is expressed as a 3x3 matrix, needed to rotate the acceleration vector

float accelNorth = 0;
float accelEast = 0;
float acceldown = 0;

void fillRotationVector(float quatX, float quatY, float quatZ, float quatW);
void fillRotationMatrix(float *quaternion, float **rotMatrix);
void inverseMatrix(float matrix[3][3], float inverse[3][3]);
void matrixOfMinors(float matrix[3][3], float minorMatrix[3][3]);
void cofactorMatrix(float minorMatrix[3][3]);
void fillInverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant);

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

void inverseMatrix(float matrix[3][3], float inverse[3][3]){
	//Inverse matrix = (1/determinat)* adjugate matrix
	float determinant = 0;
	float copyMatrix[3][3];
	
	matrixOfMinors(matrix, copyMatrix);						//determine matrix of minors
	cofactorMatrix(copyMatrix);								//determine cofactor matrix	
	determinant = determinantCalc(matrix);					//calculate the determinant
	adjugateMatrix(copyMatrix);								//determine adjugate matrix
	fillInverseMatrix(copyMatrix, inverse, determinant);	//fill inverseMatrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++){
			printf("[%d][%d] = %0.2f\n", i, j, inverse[i][j]);
		}
	}
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

void fillInverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant){
	for (int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			inverse_Matrix[i][j] = (1/determinant)*adjugate_Matrix[i][j];
		}
	}
}

