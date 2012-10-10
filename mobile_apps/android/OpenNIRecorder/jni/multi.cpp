////----------------------------------------------------------------------------------
//// File:            apps\multitouch\jni\multi.cpp
//// Samples Version: Android NVIDIA samples 2
//// Email:           tegradev@nvidia.com
//// Forum:           http://developer.nvidia.com/tegra/forums/tegra-forums/android-development
////
//// Copyright 2009-2010 NVIDIA� Corporation
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////   http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
////
////----------------------------------------------------------------------------------
//
//#define MODULE "multi"
//#define DEBUG(a) __android_log_print(ANDROID_LOG_DEBUG, MODULE, a)
//
//#include <unistd.h>
//#include <math.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <time.h>
//#include <jni.h>
//#include <android/log.h>
//
//#include <nv_and_util/nv_native_app_glue.h>
//#include <nv_egl_util/nv_egl_util.h>
//
//
//#include "renderer.h" //our rendering functions
//#include "kinect.h"
//#include "mindmesh.h"
//#include "common.h"
//
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <cv.h>
//#include "OpenniWrapper.h"
////#include <opencv2/features2d/features2d.hpp>
//
//#if 1 // TODO - these should be queried via JNI, as we do in nv_event
//const int ACTION_MASK = 255;
//const int ACTION_DOWN = 0;
//const int ACTION_UP = 1;
//const int ACTION_MOVE = 2;
//const int ACTION_CANCEL = 3;
//const int ACTION_OUTSIDE = 4;
//const int ACTION_POINTER_DOWN = 5;
//const int ACTION_POINTER_1_DOWN = 5;
//const int ACTION_POINTER_2_DOWN = 261;
//const int ACTION_POINTER_3_DOWN = 517;
//const int ACTION_POINTER_UP = 6;
//const int ACTION_POINTER_1_UP = 6;
//const int ACTION_POINTER_2_UP = 262;
//const int ACTION_POINTER_3_UP = 518;
//const int ACTION_POINTER_ID_MASK = 65280;
//const int ACTION_POINTER_ID_SHIFT = 8;
//#endif
//
//#define CURSOR_SIZE 48
////ChrisAR
//#define MULTI_MAX_INPUTS 10
//#define USE_OPENNI
//
//typedef struct {
//    float x, y;
//    int id;
//    float size;
//    float pressure;
//} NvMultiInput;
//
////static NvMultiInput pointerData[5];
//static NvMultiInput pointerData[MULTI_MAX_INPUTS];
//static float s_inputData[8*16]; //(really 5*10, but buffering for future...)
//static int pointerCount = 0;
//static int s_testOffset = 0;
//
//int first_time=1;
//
//Renderer *my_display;
//OpenniWrapper *ni_wrapper;
//
//using namespace std;
//using namespace cv;
//
//jboolean init(JNIEnv*  env, jobject  thiz)
//{
//	//create a display that we can use for the OpenGL textures etc
//	my_display = new Renderer(IMAGE_WIDTH, IMAGE_HEIGHT);
//
//	if(!my_display->init()){
//		return false;
//	}
//
//#ifdef USE_OPENNI
//	ni_wrapper = new OpenniWrapper();
//	if(ni_wrapper->start()){
//		__android_log_write(ANDROID_LOG_INFO, "OPENNI", "Failed to start OpenNI Driver");
//		return false;
//	}
//#else
//	__android_log_write(ANDROID_LOG_INFO, "Kinect", "Start Kinect Init...\n");
//    if(!try_kinect()){
//		__android_log_write(ANDROID_LOG_INFO, "Kinect", "Threaded successfully...\n");
//    }
//#endif
//
//    return JNI_TRUE;
//}
//
//
//
//
//jint render(JNIEnv*  env, jobject  thiz, jint drawWidth, jint drawHeight, jboolean forceRedraw)
//{
//    __android_log_print(ANDROID_LOG_DEBUG, MODULE, "Calling render MAIN %d %d\n", drawWidth, drawHeight);
//    screenWidth = drawWidth;
//	//return JNI_TRUE;
//	my_display->setScreenSize(drawWidth, drawHeight);
//	//usleep(1000);
//	//return 0;
//	//show_mind_mesh();
//	int i;
//
//    if (screenWidth != drawWidth)
//    {
//        __android_log_print(ANDROID_LOG_DEBUG, MODULE, "width changed %d -> %d", screenWidth, drawWidth);
//        screenWidth = drawWidth;
//        //glViewport(0, 0, screenWidth, screenHeight);
//    }
//    if (screenHeight != drawHeight)
//    {
//        __android_log_print(ANDROID_LOG_DEBUG, MODULE, "height changed %d -> %d", screenHeight, drawHeight);
//        screenHeight = drawHeight;
//        //glViewport(0, 0, screenWidth, screenHeight);
//    }
//#ifdef USE_OPENNI
//    //blocking call for now?
//    if(ni_wrapper->WaitAndUpdate()){
//    	//get our RGBD data
//    	ni_wrapper->getRGBD(my_display->getDepthInfoPtr());
//    	//get the raw depth data
//    	ni_wrapper->getDepth(my_display->getDepthShortInfoPtr());
//        my_display->render(0,0,screenWidth, screenHeight);
//    }
//#else
//    //obtain the latest data from the kinect.
//    getRGBData(my_display->getDepthInfoPtr());
//    getDepthDataShort(my_display->getDepthShortInfoPtr());
//    my_display->render(0,0,screenWidth, screenHeight);
//#endif
//    return JNI_TRUE;
//}
//
//
//void cleanup(JNIEnv*  env)
//{
//    __android_log_print(ANDROID_LOG_DEBUG, MODULE, "framework cleanup");
//
//    //stop the kinect nicely so we can resume later
//#ifdef USE_OPENNI
//    ni_wrapper->release();
//#else
//    stop_kinect();
//#endif
//    //this shall call the destructor
//    delete my_display;
//}
//
///**
// * Touch pad events
// */
//jboolean multitouchEvent(JNIEnv* env, jobject thiz, jint action,
//		jint inputCount, jfloatArray inputData, jint dataStride) {
//	int actionCode = action & ACTION_MASK;
//	//    int pointerID = (action & ACTION_POINTER_ID_MASK) >> ACTION_POINTER_ID_SHIFT;
//
//	// copy out the data we need (just the data that's active...)
//	if (inputCount)
//		env->GetFloatArrayRegion(inputData, 0, dataStride * inputCount,
//				s_inputData);
//
//	// this code is limited in tracking of state
//	// so we just stop things at the moment, w/o checks.
//	// !!!!TBD
//	if (actionCode == ACTION_UP || actionCode == ACTION_CANCEL || actionCode
//			== ACTION_OUTSIDE) {
//		pointerCount = 0;
//	} else // ACTION_MOVE, ACTION_POINTER_DOWN, ACTION_POINTER_UP !!!!TBD
//	{
//		pointerCount = inputCount;
//	}
//
//	for (int i = 0; i < inputCount; i++) {
//		const int j = dataStride * i;
//		// x and y at the front, so people who just want them don't have to skip elements.
//		pointerData[i].x = s_inputData[j + 0];
//		pointerData[i].y = s_inputData[j + 1];
//		pointerData[i].id = (int) s_inputData[j + 2];
//		pointerData[i].size = s_inputData[j + 3];
//		pointerData[i].pressure = s_inputData[j + 4];
//		__android_log_print(ANDROID_LOG_DEBUG, MODULE,
//				"@> input %d = [%d] (%0.2f, %0.2f) [%d]", i, pointerData[i].id,
//				pointerData[i].x, pointerData[i].y, pointerData[i].pressure);
//	}
//	if (inputCount >= 1){
//		//change_angle(30-pointerData[0].y/800*60);
//		float my_x = pointerData[0].x;
//		float my_y = pointerData[0].y;
//		my_display->updateMousePointer(-my_y, -my_x);
//	}
//	return JNI_TRUE;
//}
//
//
//jboolean keyEvent(JNIEnv* env, jobject thiz, jint action, jint unicode)
//{
//    static jclass KeyCode_class = env->FindClass("android/view/KeyEvent");
//    static jfieldID ACTION_UP_id = env->GetStaticFieldID(KeyCode_class, "ACTION_UP", "I");
//    static int ACTION_UP = env->GetStaticIntField(KeyCode_class, ACTION_UP_id);
//
//    if (action != ACTION_UP)
//        return JNI_TRUE;
//    __android_log_print(ANDROID_LOG_DEBUG, MODULE, "action: %d, unicode: %c", action, unicode);
//
//    switch (unicode)
//    {
//        case 'p':
//            __android_log_print(ANDROID_LOG_DEBUG, MODULE, "KEYCODE_P");
//            //pause_demo = !pause_demo;
//            break;
//    }
//
//    return JNI_TRUE;
//}
//
//
//jint JNI_OnLoad(JavaVM* vm, void* reserved)
//{
//    JNIEnv *env;
//
//	NVThreadInit(vm);
//
//	DEBUG("JNI_OnLoad called");
//    if (vm->GetEnv((void**) &env, JNI_VERSION_1_4) != JNI_OK)
//    {
//        DEBUG("Failed to get the environment using GetEnv()");
//        return -1;
//    }
//
//    JNINativeMethod methods[] =
//    {
//        {
//            "init",
//            "()Z",
//            (void *) init
//        },
//        {
//            "render",
//            "(IIZ)Z",
//            (void *) render
//        },
//        {
//            "multitouchEvent",
//            "(II[FILandroid/view/MotionEvent;)Z",
//            (void *) multitouchEvent
//        },
//        {
//            "keyEvent",
//            "(IILandroid/view/KeyEvent;)Z",
//            (void *) keyEvent
//        },
//        {
//            "cleanup",
//            "()V",
//            (void *) cleanup
//        }
//    };
//    jclass k;
//    k = (env)->FindClass ("com/nvidia/devtech/multi/Multi");
//    (env)->RegisterNatives(k, methods, 5);
//
//	//NVTimeInit();
//
//    return JNI_VERSION_1_4;
//}
//
