/*
 * rotation.h
 *
 * Created: 9/2/2020 7:22:36 PM
 *  Author: Rowan
 */ 


#ifndef ROTATION_H_
#define ROTATION_H_

void inverseMatrix(float matrix[3][3], float inverse[3][3]);
void matrixOfMinors(float matrix[3][3], float minorMatrix[3][3]);
void cofactorMatrix(float minorMatrix[3][3]);
float determinantCalc(float matrix[3][3]);
void adjugateMatrix(float cofactor_Matrix[3][3]);
void fillInverseMatrix(float adjugate_Matrix[3][3], float inverse_Matrix[3][3], float determinant);


#endif /* ROTATION_H_ */