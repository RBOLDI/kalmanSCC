/*
 * rotation.h
 *
 * Created: 9/2/2020 7:22:36 PM
 *  Author: Rowan
 */ 


#ifndef ROTATION_H_
#define ROTATION_H_

#define Q_X	0
#define Q_Y	1
#define Q_Z 2
#define Q_W 3

#define _X_ 0
#define _Y_ 1
#define _Z_ 2

void updateRotationMatrix(float *quaternion);
void update_AccelNorth(float *accelVector, float *accelVector_refNorth);
void matrixOfMinors(float matrix[3][3], float minorMatrix[3][3]);
void cofactorMatrix(float minorMatrix[3][3]);
float determinantCalc(float matrix[3][3]);
void adjugateMatrix(float cofactor_Matrix[3][3]);
void inverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant);
void rotateAccel(float *accel, float inverseRotMatrix[3][3], float *accelRotated);


#endif /* ROTATION_H_ */