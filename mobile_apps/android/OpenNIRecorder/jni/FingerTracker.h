#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>
#include "common.h"
#include <queue>

//using namespace cv; !!BUG for conflicting the PCL AND OPENCV LIBRARY!! NO NAMESPACE EVERYWHERE

#ifndef __INCLUDED_FINGER_TRACKER
#define __INCLUDED_FINGER_TRACKER

//should match with the FPS. Assuming it is 30fps for 1 second history
#define HISTORY_TRACK_SIZE 30
#define NUM_HIS_FRAME 8

//this is the fastest speed we accept for the hand tracking, anything faster than that we
//will ignore in the prediction stage
#define SPEED_LIMIT 100

//reduce the size of the frame to improve performance
//this will also reduce the accuracy of the tracking.
//must use a multiple of 2 for best performance and must be
//divisible by 640x480
#define DOWN_SAMPLE_SIZE 2
//this is the threshold we use to classify how fast we have to push
const int SLOPE_THRESHOLD = 400;


class FingerTracker {
public:
int main_fail;
int main_found;
	FingerTracker();
	FingerTracker(int w, int h);
    ~FingerTracker();
    int isGrasp(); //return 1 if we detected two hands
    void getPosition(int *x, int  *y, int *z); //return the distance between two hands
    void getPositionXY(float *x, float *y);
    void runTracking(unsigned short *raw_depth, unsigned char *rgba_buffer, int min_range, int max_range);
    void getBoundingBox(int *box);
    void blobTracking(unsigned short *raw_depth);
    void showHistory();
    void resetSeed();
    void HandLost();
    void HandFound();
    void HandUpdate ();

private:
    int width;
    int height;
    int boundingbox[4];
    //3D tracking
    int x_track_his[HISTORY_TRACK_SIZE];
    int y_track_his[HISTORY_TRACK_SIZE];
    int z_track_his[HISTORY_TRACK_SIZE];
    int predictHand[3];
    int hasSeeds;

    //kalman filter
    cv::KalmanFilter KF;
    cv::Mat state;
    cv::Mat processNoise;
    cv::Mat measurement;

    IplImage **his_frame_buf; //a list of previous frame
    IplImage *depth_map;
    IplImage *rgb_image;
    IplImage *depth_map8;
    IplImage *pushmask, *tmpmask, *handmask, *handcheckmask, *tmp_depth;
    IplImage *box_tmp;

    CvMemStorage *g_storage;
    CvMemStorage *hand_storage;
    CvMemStorage* hullStorage;
    float x;
    float y; //for now we only track 1 hand
    int grasp;
    void myThreshold(IplImage *depth_raw, IplImage *depth_binary, int min, int max);
    //helper functions
    void pushMask(int threshold);
    int getSeeds();
    int growHand(int delta, int depth_bound, int box_size);
    int connect(int x, int y, int z, int direction, int delta);
    int **spiralLookup(int box_size);
    void predictHistoryTrack(int *results);
    void updateHistory(int x, int y, int z);
    void locateTrackPoint(int *result, int box_size);

};

#endif
