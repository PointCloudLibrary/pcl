#ifndef __RENDERER_H__
#define __RENDERER_H__
//GPU, OpenGL ES2 and other NVIDIA APIs for loading vertex and fragment programs
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

//#include <nv_time/nv_time.h>
#include <nv_shader/nv_shader.h>
#include <nv_thread/nv_thread.h>
#include <nv_file/nv_file.h>
#include <nv_and_util/nv_native_app_glue.h>

#include <nv_math/nv_math.h>
#include <nv_math/nv_matrix.h>
#include <sys/time.h>

#include "FrameBuffer.h"
#include "common.h"

//bad
#include "kinect.h"
#include "matrix.h"
#include "vec4.h"

//#define USE_UI_HACK

#ifdef USE_UI_HACK
#include "gpgpu/GpuImageProcessor.hpp"
#include "gpgpu/GpuImageProcessorBuffer.hpp"
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>

#include "FingerTracker.h"



extern "C" {
	//screenbuffer code
	#include "capturescr.h"
	#include "threadpool.h"
}

#include <pthread.h>

class Renderer{
protected:
public:
	Renderer(int width, int height);
	~Renderer();
	bool init();
	void render(float x, float y, float w, float h);
	void render_pointcloud(float x, float y, float w, float h);
	unsigned char *getDepthInfoPtr();
	unsigned short *getDepthShortInfoPtr();
	unsigned char *getRGBInfoPtr();
	void updateMousePointer(float x, float y);
	void setScreenSize(int x, int y);
	void touch();
	void savePNG();
	void pressButton(int x, int y);
	void renderButtons(unsigned char *rgba);
	bool recordDepth();
	bool recordRGB();
	void setMessage(char *msg, int size);
	void setMessage2(char *msg, int size);
	bool showDepth();
	bool showRGB();
	void setXYZ(double x, double y, double z);
	void showBar(float x, float y);

private:
    float my_x, my_y, iAccx, iAccy, iAccz;
	bool buttons[10];
	char **button_names;
	bool record_depth;
	bool record_rgb;
	bool show_depth;
	bool show_rgb;


	char msg[1024];
	char msg2[1024];

	FingerTracker *fingerTracker;
	FrameBuffer *fb;
	int image_width;
	int image_height;
	int screen_width;
	int screen_height;

	GLuint gProgram;
	GLuint gvPositionHandle;
	GLuint gvTexCoordHandle;
	GLuint gvSamplerRGBHandle;
	GLuint gvSamplerDEPTHHandle;
	GLuint gvDepthHandle;

	GLuint gDirectProgram;
	GLuint gvDirectPositionHandle;
	GLuint gvDirectTexCoordHandle;
	GLuint gvDirectSamplerRGBHandle;
	GLuint gvDirectSamplerDEPTHHandle;

	GLuint overlayProgram;
	GLuint overlayvPositionHandle;
	GLuint overlayvTexCoordHandle;
	GLuint overlayvSamplerHandle;
	GLuint overlayaColorHandle;

	GLfloat *myVertices;
	GLfloat *myColor;

	//these shall point to the buffer
	unsigned char *depth_info;
	unsigned short *depth_short_info;
	float *depth_float_info;
	unsigned char *processed_data;
	float *lookupDepth;
	unsigned char *rgb_info;
	//need a floating point data soon.

	//user input coordinate
	float user_input_y;
	float user_input_x;
	bool isTouched;
	bool isSaving;
	bool trackFinger;

	//overlay
	double iXangle, iYangle, iZangle;
	float aRotate[16], aModelView[16], aPerspective[16], aMVP[16];
	float uiWidth; float uiHeight;
	float animParam;

	#ifdef USE_UI_HACK
	GpuImageProcessor *gp;
	GpuImageProcessorBuffer *gb_in;
	GpuImageProcessorBuffer *gb_out;
	#endif
	//private functions for handling the works
	void displayOverlay();
	void displayOverlay2();
	void displayTexture();
	void displayTextureDirect(int offset_x, int offset_y, float scale, float rotx, float roty);
	void copyToHost(unsigned char *data);
	int copyToDevice(unsigned char* data);
	void convertToDepth(float *output, unsigned short *input);
	void changeToBlack(unsigned char *data, int x, int y);
	void processBuffer(unsigned char* rgb_d, float *depth);
	void displayOverlayPointCloud(float *point_cloud_data, const int NUM_POINT_CLOUD_DATA);
};

#endif

