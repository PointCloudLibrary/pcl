#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include "common.h"
#include <queue>

//using namespace cv; !!BUG for conflicting the PCL AND OPENCV LIBRARY!! NO NAMESPACE EVERYWHERE

#ifndef __INCLUDED_FINGER_TRACKER
#define __INCLUDED_FINGER_TRACKER


class FingerTracker {
public:
	FingerTracker(int w, int h);
    ~FingerTracker();
    int isGrasp(); //return 1 if we detected two hands
    void getPosition(float *x, float *y); //return the distance between two hands
    void runTracking(unsigned short *raw_depth, unsigned char *rgba_buffer, int min_range, int max_range);
    void getBoundingBox(int *box);

private:
    int width;
    int height;
    int boundingbox[4];

    IplImage *depth_map;
    IplImage *rgb_image;
    IplImage *depth_map8;

    CvMemStorage *g_storage;
    CvMemStorage *hand_storage;
    CvMemStorage* hullStorage;
    float x;
    float y; //for now we only track 1 hand
    int grasp;
    void myThreshold(IplImage *depth_raw, IplImage *depth_binary, int min, int max);
};

#endif
