#include <stdio.h>
#include <string.h> //memcpy()
#include <math.h>   //sin(), cos(), pow()

#define LEN(arr) ((int) (sizeof (arr) / sizeof (arr)[0])) // LEN(arr) = rows, LEN(arr[0]) = columns
#define PI 3.14159265

// https://stackoverflow.com/questions/8380348/assigning-an-entire-array-with-a-single-statement
// https://en.wikipedia.org/wiki/Matrix_representation

void MatrixMultiplication(int rowsA, int colsA, float matA[][colsA], int colsB, float matB[][colsB], float returnSet[][colsB])
{
    int i, j, k;
    for (i = 0; i < rowsA; i++) {
        for(j = 0; j < colsB; j++) {
            for(k = 0; k < colsA; k++) {
                returnSet[i][j] += matA[i][k] * matB[k][j];
            }
            printf("%f\n", returnSet[i][j]);
        }
    }
}

// Multiply by number
void MatrixDotVector(int rows, int cols, float matA[][cols], float vector, float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0 ; j < cols; j++) {
            returnSet[i][j] = matA[i][j] * vector;
        }
    }
}

void MatrixAddition(int rows, int cols, float matA[][cols], float matB[][cols], float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0 ; j < cols; j++) {
            returnSet[i][j] = matA[i][j] + matB[i][j];
            printf("%.6f\n", returnSet[i][j]);
        }
    }
}

struct KalmanSet {
    float matA[3][3];
    float matB[3][1];
    float matC[3][1];
    float X_k1[2][1];
    float X_kp[2][1];
};

/*
 * Inputs:
 *   t = time interval between itterations
 *   u_k = acceleration
 *   theta = Angle apposed to true North
 */
void prediction(float u_k, int theta, struct KalmanSet *set)
{
    // 1: X_kp = AX_k-1 + Bu_k + w_k
    float A[2][2] = {1,0,0,1};
    float B[2][1] = {cos(theta * PI/180), sin(theta * PI/180) / cos(set->X_k1[0][0] * PI/180)};
    int w_k = 0;

    // Convert meters to delta latitude, radius of the earth is 6378137m
    u_k = (u_k / 3.6) / ((2 * PI/360) * 6378137);

    //self.X_kp = np.dot(A, self.X_k1) + np.dot(B, u_k) + w_k
    float temp[2][1];
    float temp2[2][1];

    MatrixMultiplication(LEN(A), LEN(A[0]), A, LEN(set->X_k1[0]), set->X_k1, temp);
    MatrixDotVector(LEN(B), LEN(B[0]), B, u_k, temp2);
    MatrixAddition(2, 1, temp, temp2, set->X_kp);

    // 2: P_kp = AP_k-1A^T + Qk
    float Q_k = pow(0.1, 2);
}

/*
 * Inputs:
 *   Oe = Observational errors, matrix of error in measurement
 *   MEA = Location and velocity measurement

void update(Oe)
{

}
 */

int main() {
    struct KalmanSet package = {.matA = {5,8,-4,6,9,-5,4,7,-2}, .matB = {2,-3,1}, .matC = {0}, .X_k1 = {51.540002638862795,-0.009483482286004047}};


    prediction(25.24, 335.7, &package);

    //MatrixMultiplication(LEN(package.matA), LEN(package.matA[0]), package.matA, LEN(package.matB[0]), package.matB, package.matC);
}
