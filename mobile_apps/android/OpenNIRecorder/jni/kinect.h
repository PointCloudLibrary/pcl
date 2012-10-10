#ifndef __KINECT_H__
#define __KINECT_H__

int try_xtion();
int try_kinect();

void change_angle(double angle);
void getDepthData(unsigned char *depth_rgb);
void getDepthDataShort(uint16_t *depth_float);
void getRGBData(unsigned char *depth_rgb);
void getRGBDataCalibrated(unsigned char *depth_rgb);
int new_frame();
void stop_kinect();
void get_max_xy(int *x, int *y);
float rawDepthToMeters(int depthValue);

#endif
