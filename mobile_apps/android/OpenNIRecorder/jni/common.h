#ifndef __COMMON_H__
#define __COMMON_H__

//all the common libraries and variables we have to predefined
static int screenWidth = 640;
static int screenHeight = 480;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const int COLOR_CHANNEL = 4;
const int NUM_VERTICES = IMAGE_WIDTH*IMAGE_HEIGHT;
const int DEPTH_VERTEX_SIZE = IMAGE_WIDTH*IMAGE_HEIGHT*3*sizeof(float);
#include <android/log.h>
#include <time.h>

#endif
