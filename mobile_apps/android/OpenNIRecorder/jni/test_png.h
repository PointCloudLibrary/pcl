#ifndef __TEST_PNG_H__
#define __TEST_PNG_H__

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <png.h>
#include "quicklz/quicklz.h"


// Creates a test image for saving. Creates a Mandelbrot Set fractal of size width x height
float *createMandelbrotImage(int width, int height, float xS, float yS, float rad, int maxIteration);

// This takes the float value 'val', converts it to red, green & blue values, then
// sets those values into the image memory buffer location pointed to by 'ptr'
inline void setRGB(png_byte *ptr, float val);

// This function actually writes out the PNG image file. The string 'title' is
// also written into the image file
int writeImage(char* filename, int width, int height, float *buffer, char* title);
int writeImageRGB(char* filename, int width, int height, unsigned char *buffer, char* title);
int writeImageRAW_RGB(char* filename, int width, int height, int num_channels, unsigned char *buffer, int compress);
int writeImageDepth(char* filename, int width, int height, unsigned short *buffer, char* title);

//actual test
int main_run();

#endif
