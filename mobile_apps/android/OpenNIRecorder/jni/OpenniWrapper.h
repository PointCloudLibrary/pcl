#ifndef __INCLUDED_OPENNI_WRAPPER_H
#define __INCLUDED_OPENNI_WRAPPER_H
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
#include <XnOS.h>
#include <math.h>
//android logging only
#include <android/log.h>

#include "test_png.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "/data/ni/SamplesConfig.xml"
#define DEBUG_OPENNI


using namespace xn;
using namespace std;

class OpenniWrapper {
public:
	OpenniWrapper();
	~OpenniWrapper();
	//initialize the devices, and the generators
	int start();
	//release the resources
	void release();
	//check and update the frame (blocking call).
	bool WaitAndUpdate();
	//the raw rgb
	void getRGB(unsigned char *rgb_buffer);
	//the raw depth image
	void getDepth(unsigned short *depth_buffer);
	//the rgbd (D = depth value and it is compressed and shifted) see: documentation
	void getRGBD(unsigned char *rgbd_buffer);

	int getWidth();
	int getHeight();

private:
	//memory for the OpenNI data
	Context g_context;
	ScriptNode g_scriptNode;
	//the depth map and color image
	DepthGenerator g_depth;
	ImageGenerator g_image;
	DepthMetaData g_depthMD;
	ImageMetaData g_imageMD;
	int width, height;
	//local storage for the depth and rgb
	unsigned char *rgbImage;
	unsigned short *pDepth;

	bool hasRGB;
	bool hasDepth;
	char buf[1024];

	//private/helper functions
	XnBool fileExists(const char *fn);
};
#endif

