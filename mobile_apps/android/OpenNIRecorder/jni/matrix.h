#ifndef MATRIX_H
#define MATRIX_H

/**
 * Thanks: http://code.google.com/p/spinningcube/ for sharing this source code
 * It is easy to use at least
 *
 * -ray (Oct 17, 2011)
 */
#include <math.h>

void rotate_matrix(double angle, double x, double y, double z, float *R);
void rotate_then_translate_matrix(double angle, double x, double y, double z, double xt, double yt, double zt, float *R);
void translate_matrix(double xt, double yt, double zt, float *T);
void perspective_matrix(double fovy, double aspect, double znear, double zfar, float *P);
void multiply_matrix(float *A, float *B, float *C);

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif /* M_PI */

#endif
