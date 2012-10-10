//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#include "engine.h"

#include <EGL/egl.h>
#include <EGL/eglplatform.h>
#include <GLES2/gl2.h>

#include <android/log.h>
#include <nv_and_util/nv_native_app_glue.h>
#include <nv_egl_util/nv_egl_util.h>
#include <nv_bitfont/nv_bitfont.h>
#include <nv_shader/nv_shader.h>

#define MODULE "multi"
#define DEBUG(a) __android_log_print(ANDROID_LOG_DEBUG, MODULE, a)

#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <jni.h>
#include "renderer.h" //our rendering functions
#include "kinect.h"
#include "common.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include "OpenniWrapper.h"
#include "PCLWrapper.h"
//#include <opencv2/features2d/features2d.hpp>
static int pointerCount = 0;
static int s_testOffset = 0;

int first_time=1;

Renderer *my_display;
OpenniWrapper *ni_wrapper;
#define USE_OPENNI

//for multithreading
static pthread_mutex_t rgb_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
static int frame_count=0;
using namespace std;
using namespace cv;

typedef struct{
    int width;
    int height;
    unsigned char *rgb_data;
    unsigned short *depth_data;
    int frame_count;
}RGBpack;

Engine::Engine(NvEGLUtil& egl, struct android_app* app) :
	mEgl(egl)
{
    mApp = app;

	mResizePending = false;

	mGameplayMode = true;

	mForceRender = 4;

    mTimeVal = 0.0;

    app->userData = this;
	app->onAppCmd = &Engine::handleCmdThunk;
    app->onInputEvent = &Engine::handleInputThunk;

	m_uiInitialized = false;

	nv_shader_init(app->activity->assetManager);

	m_clockText = NULL;
	m_uiText[0] = NULL;
	m_uiText[1] = NULL;

	//acelerometer
    m_sensorManager = ASensorManager_getInstance();
    m_accelerometerSensor = ASensorManager_getDefaultSensor(m_sensorManager,
            ASENSOR_TYPE_ACCELEROMETER);
    m_sensorEventQueue = ASensorManager_createEventQueue(m_sensorManager,
            mApp->looper, LOOPER_ID_USER, NULL, NULL);

    m_rawAccelerometer.x = 0.0f;
    m_rawAccelerometer.y = -9.8f;
    m_rawAccelerometer.z = 0.0f;

    m_worldAccelerometer.x = 0.0f;
    m_worldAccelerometer.y = -9.8f;
    m_worldAccelerometer.z = 0.0f;

	mOrientation = nv_app_get_display_rotation(mApp);

    ASensorEventQueue_enableSensor(m_sensorEventQueue,
            m_accelerometerSensor);
    // We'd like to get 30 events per second (in us).
    ASensorEventQueue_setEventRate(m_sensorEventQueue,
            m_accelerometerSensor, (1000L/30)*1000);
}

Engine::~Engine()
{

    __android_log_print(ANDROID_LOG_DEBUG, MODULE, "framework cleanup");
    //stop the kinect nicely so we can resume later
#ifdef USE_OPENNI
    ni_wrapper->release();
#else
    stop_kinect();
#endif

	NVBFTextFree(m_uiText[1]);
	NVBFTextFree(m_uiText[0]);
	NVBFTextFree(m_clockText);
	NVBFCleanup();

    threadpool_free(pool,1);
}

void fast_task(void *ptr)
{
	RGBpack *file_pack = (RGBpack*)ptr;
	//pthread_mutex_lock(&rgb_mutex);
	//put anything that requires sync here later.

	//frame_count++;
	//char buf[1024];
	//sprintf(buf, "%d", frame_count);
	//__android_log_write(ANDROID_LOG_INFO, "THREAD POOL:", buf);
	//pthread_mutex_unlock(&rgb_mutex);

	char my_path[1024];
	if(file_pack->rgb_data!=NULL){
		sprintf(my_path, "out_rgb_%06d.png", file_pack->frame_count, file_pack->width, file_pack->height);
		int ret=0;
		ret = writeImageRGB(my_path, file_pack->width, file_pack->height, file_pack->rgb_data, my_path); //write RGB image, with compression
		//__android_log_write(ANDROID_LOG_INFO, "PNG DUMP:", my_path);
		if(ret){
			pthread_mutex_lock(&rgb_mutex);
				char buf[512];
				sprintf(buf, "Error Writing to SDCard - PNG RGB");
				my_display->setMessage2(buf, 512);
			pthread_mutex_unlock(&rgb_mutex);
		}else{
			char buf[512];
			sprintf(buf, "");
			my_display->setMessage2(buf, 512);
		}
	}
	if(file_pack->depth_data!=NULL){
		sprintf(my_path, "out_depth_%06d.png", file_pack->frame_count);
		int ret=0;
		ret=writeImageDepth(my_path, file_pack->width, file_pack->height, file_pack->depth_data, my_path);
		//__android_log_write(ANDROID_LOG_INFO, "PNG DUMP:", my_path);
		if(ret){
			pthread_mutex_lock(&rgb_mutex);
				char buf[512];
				sprintf(buf, "Error Writing to SDCard - PNG depth");
				my_display->setMessage2(buf, 512);
			pthread_mutex_unlock(&rgb_mutex);
		}else{
			char buf[512];
			sprintf(buf, "");
			my_display->setMessage2(buf, 512);
		}
	}
    if(file_pack!=NULL){
	    if(file_pack->rgb_data!=NULL)
    		free(file_pack->rgb_data);
    	if(file_pack->depth_data!=NULL)
    		free(file_pack->depth_data);
	    free(file_pack);
    }
}



bool Engine::initUI()
{
	if (m_uiInitialized)
		return true;

	//benchmark
//	sleep(1);
//	__android_log_write(ANDROID_LOG_INFO, "PCL BENCHMARK", "PREPARING");
	PCLWrapper *pcl_tester2 = new PCLWrapper();
//	//	//pcl_tester2->benchmark_png();
//	pcl_tester2->benchmark_raw();
	pcl_tester2->test_registration();
//	sleep(1);
//	__android_log_write(ANDROID_LOG_INFO, "PCL BENCHMARK", "FINISHED");

	#define NUM_FONTS	2
	static NvBool fontsSplit[NUM_FONTS] = {1,1}; /* all are split */
	static const char *fontFiles[NUM_FONTS] = {
	    "courier+lucida_256.dds",
	    "utahcond+bold_1024.dds"
	};
	if (NVBFInitialize(NUM_FONTS, (const char**)fontFiles, fontsSplit, 0))
	{
		LOGW("Could not initialize NvBitFont");
		return false;
	}

	m_clockText = NVBFTextAlloc();
	NVBFTextSetFont(m_clockText, 1); // should look up by font file name.
	NVBFTextSetSize(m_clockText, 32);
    NVBFTextSetColor(m_clockText, NV_PC_PREDEF_WHITE);
	NVBFTextSetString(m_clockText, "000:00.00");

	m_uiText[0] = NVBFTextAlloc();
	NVBFTextSetFont(m_uiText[0], 2); // should look up by font file name.
	NVBFTextSetSize(m_uiText[0], 32);
    NVBFTextSetColor(m_uiText[0], NV_PC_PREDEF_WHITE);
	NVBFTextSetString(m_uiText[0],
			NVBF_COLORSTR_RED NVBF_STYLESTR_BOLD "Auto-pause:\n" NVBF_STYLESTR_NORMAL
			NVBF_COLORSTR_BLUE "Press back to quit\nTap window to resume");

	m_uiText[1] = NVBFTextAlloc();
	NVBFTextSetFont(m_uiText[1], 2); // should look up by font file name.
	NVBFTextSetSize(m_uiText[1], 32);unsigned char *rgb_data;
    NVBFTextSetColor(m_uiText[1], NV_PC_PREDEF_WHITE);
	NVBFTextSetString(m_uiText[1], NVBF_COLORSTR_GREEN "Gameplay mode!");

	//create a display that we can use for the OpenGL textures etc
	my_display = new Renderer(IMAGE_WIDTH, IMAGE_HEIGHT);
	my_display->init();

	//init our openni drivers
#ifdef USE_OPENNI
	ni_wrapper = new OpenniWrapper();
	if(ni_wrapper->start()){
		__android_log_write(ANDROID_LOG_INFO, "OPENNI", "Failed to start OpenNI Driver\n");
		//return false;
	}else{
		__android_log_write(ANDROID_LOG_INFO, "OPENNI", "Started OpenNI Driver\n");
	}
#else
	__android_log_write(ANDROID_LOG_INFO, "Kinect", "Start Kinect Init...\n");
    if(!try_kinect()){
		__android_log_write(ANDROID_LOG_INFO, "Kinect", "Threaded successfully...\n");
    }
#endif

    if ((pool = threadpool_init(3)) == NULL) {
    	__android_log_write(ANDROID_LOG_INFO, "THREAD POOL", "Failed to initialize the threads\n");
    }
	m_uiInitialized = true;

	return true;
}
void Engine::updateAccelHistory(float x, float y, float z){
	//shifting...
	for(int i=0; i<ACCEL_BUFFER_SAMPLE_SIZE; i++){
		int j = 3*i;
		accel_histroy[j]=accel_histroy[j+3];
		accel_histroy[j+1]=accel_histroy[j+4];
		accel_histroy[j+2]=accel_histroy[j+5];
	}
	accel_histroy[ACCEL_BUFFER_SAMPLE_SIZE-1-2]= m_rawAccelerometer.x;
	accel_histroy[ACCEL_BUFFER_SAMPLE_SIZE-1-1]= m_rawAccelerometer.y;
	accel_histroy[ACCEL_BUFFER_SAMPLE_SIZE-1]= m_rawAccelerometer.z;
	//store the data at the end of the list
}

void Engine::looperIteration(int looperIdent)
{
//    __android_log_write(ANDROID_LOG_INFO, "ACCELEROMETER", "Accelerometer: Looper looperIteration");
//    __android_log_write(ANDROID_LOG_INFO, "ACCELEROMETER:", "NOTHING\n");
    if (looperIdent == LOOPER_ID_USER) {
        if (m_accelerometerSensor != NULL) {
            ASensorEvent event;
            while (ASensorEventQueue_getEvents(m_sensorEventQueue,
                    &event, 1) > 0) {
                m_rawAccelerometer = event.acceleration;
				// Transform into "rendering space" from the device-
				// relative cannonical space and cache in the engine object
                canonicalToWorld(mOrientation, m_rawAccelerometer,
                    m_worldAccelerometer);
                updateAccelHistory(m_rawAccelerometer.x, m_rawAccelerometer.y, m_rawAccelerometer.z);

//                LOGI("accelerometer: raw(%f, %f, %f), world(%f, %f, %f)",
//                        m_rawAccelerometer.x, m_rawAccelerometer.y,
//                        m_rawAccelerometer.z,
//                        m_worldAccelerometer.x, m_worldAccelerometer.y,
//                        m_worldAccelerometer.z);

//                __android_log_write(ANDROID_LOG_INFO, "ACCELEROMETER", "NOTHING");
                if(my_display)
                	my_display->setXYZ(m_worldAccelerometer.x, m_worldAccelerometer.y, m_worldAccelerometer.z);
			}
        }

    }
}
void Engine::setGameplayMode(bool running)
{
	if (mGameplayMode != running)
	{
		requestForceRender();

		if (m_accelerometerSensor != NULL)
		{
			// Immediately start/stop the accelerometer when we're
			// entering/leaving "gameplay", or active looping/rendering mode
			// leaving this enabled in pause mode is _bad_ as it spams the
			// event queue and causes the app to fall behind
			if (running)
			{
                ASensorEventQueue_enableSensor(m_sensorEventQueue,
                        m_accelerometerSensor);
                // We'd like to get 60 events per second (in us).
                ASensorEventQueue_setEventRate(m_sensorEventQueue,
                        m_accelerometerSensor, (1000L/60)*1000);
            }
			else
			{
                ASensorEventQueue_disableSensor(m_sensorEventQueue,
                        m_accelerometerSensor);
            }
		}
	}

	mGameplayMode = running;
}

bool Engine::checkWindowResized()
{
	if (mEgl.checkWindowResized())
	{
		mResizePending = true;
		requestForceRender();
		LOGI("Window size change %dx%d", mEgl.getWidth(), mEgl.getHeight()); 
		return true;
	}

	return false;
}unsigned char *rgb_data;

bool Engine::resizeIfNeeded()
{
	if (!mResizePending)
		return false;

	int w = mEgl.getWidth();
	int h = mEgl.getHeight();
	int height = (w > h) ? (h / 16) : (w / 16);

	NVBFSetScreenRes(w, h);
	if (m_clockText)
	{
		NVBFTextSetSize(m_clockText, height);
		NVBFTextCursorAlign(m_clockText, NVBF_ALIGN_LEFT, NVBF_ALIGN_TOP);
		NVBFTextCursorPos(m_clockText, 10, 10);
	}
	if (m_uiText[0])
	{
		NVBFTextSetSize(m_uiText[0], height);
		NVBFTextCursorAlign(m_uiText[0], NVBF_ALIGN_CENTER, NVBF_ALIGN_BOTTOM);
		NVBFTextCursorPos(m_uiText[0], w/2, h/2);
	}
	if (m_uiText[1])
	{
		NVBFTextSetSize(m_uiText[1], height);
		NVBFTextCursorAlign(m_uiText[1], NVBF_ALIGN_CENTER, NVBF_ALIGN_CENTER);
		NVBFTextCursorPos(m_uiText[1], w/2, h/2);
	}
	mResizePending = false;

	return true;
}

bool Engine::renderFrame(bool allocateIfNeeded)
{
    static int counter = 0;

    if (!mEgl.isReadyToRender(allocateIfNeeded))
        return false;

	if (!initUI())
	{
		LOGW("Could not initialize UI - assets may be missing!");
		ANativeActivity_finish(mApp->activity);
		return false;
	}

	resizeIfNeeded();

	my_display->setScreenSize(mEgl.getWidth(), mEgl.getHeight());

    struct timeval start, end;
    double t1,t2;
    static double elapsed_sec=0;
    gettimeofday(&start, NULL);
    int i;
    static int count=0;
    const int MAX_COUNT=10;

//
//	//	// set up viewport
//	glViewport((GLint)0, (GLint)0,
//			(GLsizei)(mEgl.getWidth()), (GLsizei)(mEgl.getHeight()));
//	//
//	//	// clear buffers as necessary, but not this time
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	// start rendering bitfont text overlaid here.

//	NVBFTextRenderPrep();
//	if (m_clockText)
//	{
//		NVBFTextSetMultiplyColor(m_clockText, mGameplayMode ?
//				NV_PACKED_COLOR(255, 255, 255, 255) :
//				NV_PACKED_COLOR(255, 50, 50, 255) );
//		NVBFTextRender(m_clockText);
//
//		// we update the clock text >after< drawing so it will change on pause.
//		int mins = mTimeVal / 60;
//		float secs = (float)mTimeVal - mins*60;
//		char str[32];
//		sprintf(str, "%03d:%05.2f", mins, secs);
//		NVBFTextSetString(m_clockText, str);
//	}
//	NVBFTextRenderDone();

//	void* uiText = m_uiText[mGameplayMode ? 1 : 0];
//	if (uiText)
//		NVBFTextRender(uiText);
////
//	// done rendering overlaid text.




#ifdef USE_OPENNI
    if(ni_wrapper->WaitAndUpdate()){
	    //get our RGBD data, this will do memory copy.
    	if(my_display->showRGB()){
    		ni_wrapper->getRGBD(my_display->getDepthInfoPtr());
    		ni_wrapper->getRGB(my_display->getRGBInfoPtr());
    	}
	    //get the raw depth data
    	if(my_display->showDepth())
    		ni_wrapper->getDepth(my_display->getDepthShortInfoPtr());


	    char my_path[512];
	    RGBpack *filepack = (RGBpack*)(malloc(sizeof(RGBpack)));

	    filepack->width=IMAGE_WIDTH;
	    filepack->height=IMAGE_HEIGHT;
	    filepack->depth_data=NULL;
	    filepack->rgb_data=NULL;

	    if(my_display->recordRGB()||my_display->recordDepth()){
	    	counter++;
	    }

	    filepack->frame_count=counter;
	    if(my_display->recordRGB()){
	    	filepack->rgb_data = (unsigned char*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*3*sizeof(unsigned char));
	    	ni_wrapper->getRGB(filepack->rgb_data);
	    }
	    if(my_display->recordDepth()){
	    	filepack->depth_data = (unsigned short*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(unsigned short));
		    ni_wrapper->getDepth(filepack->depth_data);
	    }
	    //adding each of the frame packages to the thread, blocking call if the threadpool is full.
	    int ret = threadpool_add_task(pool,fast_task,filepack,1); //this will also free the memory

	    if(ret==-1){
		    __android_log_write(ANDROID_LOG_INFO, "THREAD POOL:", "POOL ERROR?\n");
		    if(filepack!=NULL){
			    if(filepack->rgb_data!=NULL)
		    		free(filepack->rgb_data);
		    	if(filepack->depth_data!=NULL)
		    		free(filepack->depth_data);
			    free(filepack);
		    }
	    }
	    if(ret==-2){
		    __android_log_write(ANDROID_LOG_INFO, "THREAD POOL:", "FAILED to add task, pool full?\n");
		    if(filepack!=NULL){
			    if(filepack->rgb_data!=NULL)
		    		free(filepack->rgb_data);
		    	if(filepack->depth_data!=NULL)
		    		free(filepack->depth_data);
			    free(filepack);
		    }
	    }
    }
    my_display->render(0,0,screenWidth, screenHeight);
#else
    //obtain the latest data from the kinect.
    getRGBData(my_display->getDepthInfoPtr());
    getDepthDataShort(my_display->getDepthShortInfoPtr());
    my_display->render(0,0,screenWidth, screenHeight);
#endif
    //

    //    glViewport((GLint)0, (GLint)0,
//    			(GLsizei)(mEgl.getWidth()), (GLsizei)(mEgl.getHeight()));
	if (mForceRender > 0)
		mForceRender--;

    mEgl.swap();

    gettimeofday(&end, NULL);
	t1=start.tv_sec+(start.tv_usec/1000000.0);
	t2=end.tv_sec+(end.tv_usec/1000000.0);
	elapsed_sec += (t2-t1);
	count++;
	if(count>=MAX_COUNT){
		char buf[512];
		sprintf(buf, "Display loop %f (s), %f fps\n", (elapsed_sec)/MAX_COUNT, MAX_COUNT/elapsed_sec);
		__android_log_write(ANDROID_LOG_INFO, "Render Loop:", buf);
	    sprintf(buf, "%d frames, %lf fps", counter, MAX_COUNT/elapsed_sec);
	    my_display->setMessage(buf, 512);

		elapsed_sec=0;
		count=0;
	}

    return true;
}

void Engine::updateFrame(bool interactible, long deltaTime)
{
	if (interactible)
	{
		// Each frame, we check to see if the window has resized.  While the
		// various events we get _should_ cover this, in practice, it appears
		// that the safest move across all platforms and OSes is to check at 
		// the top of each frame
		checkWindowResized();

		// Time stands still when we're auto-paused, and we don't
		// automatically render
		if (mGameplayMode)
		{
			advanceTime(deltaTime);

			// This will try to set up EGL if it isn't set up
			// When we first set up EGL completely, we also load our GLES resources
			// If these are already set up or we succeed at setting them all up now, then
			// we go ahead and render.
			renderFrame(true);
		}
		else if (isForcedRenderPending()) // forced rendering when needed for UI, etc
		{
			renderFrame(true);
		}
	}
	else
	{
		// Even if we are not interactible, we may be visible, so we
		// HAVE to do any forced renderings if we can.  We must also
		// check for resize, since that may have been the point of the
		// forced render request in the first place!
		if (isForcedRenderPending() && mEgl.isReadyToRender(false)) 
		{
			checkWindowResized();
			renderFrame(false);
		}
	}
}

int Engine::handleInput(AInputEvent* event)
{
    //We only handle motion events (touchscreen) and key (button/key) events
	int32_t eventType = AInputEvent_getType(event);
	static char eng_buf[1024];
	if (eventType == AINPUT_EVENT_TYPE_MOTION)
	{
		int32_t action = AMOTION_EVENT_ACTION_MASK &
    					 AMotionEvent_getAction((const AInputEvent*)event);

		int32_t iX = AMotionEvent_getX(event, 0);
		int32_t iY = AMotionEvent_getY(event, 0);

		//get the x-y coordinate
		//sprintf(eng_buf, "x: %d \t y: %d", iX, iY);
		//my_display->updateMousePointer(iX, iY);

		//__android_log_write(ANDROID_LOG_INFO, "KEY EVENT:", eng_buf);

		// A tap on the screen takes us out of autopause into gameplay mode if
		// we were paused.  No other touch processing is done.
		if (action == AMOTION_EVENT_ACTION_DOWN)
		{
			//determine the buttons by regions.
			//setGameplayMode(true);
//			return 0;
			sprintf(eng_buf, "Key Down Event at x: %d \t y: %d", iX, iY);
			__android_log_write(ANDROID_LOG_INFO, "KEY EVENT:", eng_buf);

		}else if (action == AMOTION_EVENT_ACTION_UP){
			sprintf(eng_buf, "Key Up Event at x: %d \t y: %d", iX, iY);
			my_display->pressButton(iX, iY);
			if(iY<200){
				my_display->touch();
			}
			__android_log_write(ANDROID_LOG_INFO, "KEY EVENT:", eng_buf);
		}

		return 1;
	} else if (eventType == AINPUT_EVENT_TYPE_KEY) {
		int32_t code = AKeyEvent_getKeyCode((const AInputEvent*)event);
		// if we are in gameplay mode, we eat the back button and move into
		// pause mode.  If we are already in pause mode, we allow the back
		// button to be handled by the OS, which means we'll be shut down
		if ((code == AKEYCODE_BACK) && mGameplayMode)
		{
			setGameplayMode(false);
			return 1;
		}
	}

    return 0;
}

void Engine::handleCommand(int cmd)
{
    switch (cmd)
    {
		// The window is being shown, get it ready.
		// Note that on ICS, the EGL size will often be correct for the new size here
		// But on HC it will not be.  We need to defer checking the new res until the
		// first render with the new surface!
        case APP_CMD_INIT_WINDOW:
        case APP_CMD_WINDOW_RESIZED:
			mEgl.setWindow(mApp->window);
			requestForceRender();
        	break;

        case APP_CMD_TERM_WINDOW:
            // The window is being hidden or closed, clean it up.
			mEgl.setWindow(NULL);
			break;

        case APP_CMD_GAINED_FOCUS:
			requestForceRender();
			break;

        case APP_CMD_LOST_FOCUS:
		case APP_CMD_PAUSE:
        	// Move out of gameplay mode if we are in it.  But if we are
			// in another dialog mode, leave it as-is
            if (mGameplayMode)
				//setGameplayMode(false);
			requestForceRender();
            break;

		// ICS does not appear to require this, but on GB phones,
		// not having this causes rotation changes to be delayed or
		// ignored when we're in a non-rendering mode like autopause.
		// The forced renders appear to allow GB to process the rotation
		case APP_CMD_CONFIG_CHANGED:
			requestForceRender();
			break;
    }
}

/**
 * Process the next input event.
 */
int32_t Engine::handleInputThunk(struct android_app* app, AInputEvent* event)
{
    Engine* engine = (Engine*)app->userData;
	if (!engine)
		return 0;

	return engine->handleInput(event);
}

/**
 * Process the next main command.
 */
void Engine::handleCmdThunk(struct android_app* app, int32_t cmd)
{
    Engine* engine = (Engine*)app->userData;
	if (!engine)
		return;

	engine->handleCommand(cmd);
}

// Transforms the device-cannonical-space accelerometer vector to
// current-screen-orientation-relative accelerometer.
void Engine::canonicalToWorld(int displayRotation, const ASensorVector& cannonicalVec,
    ASensorVector& worldVec)
{
	// define a table to do the axis negate/swaps,
	struct AxisSwap
	{
		signed char negateX;
		signed char negateY;
		signed char xSrc;
		signed char ySrc;
	};
	static const AxisSwap axisSwap[] = {
		{ 1,  1, 0, 1 },
		{-1,  1, 1, 0 },
		{-1, -1, 0, 1 },
		{ 1, -1, 1, 0 } };

	const AxisSwap& as = axisSwap[displayRotation];
	worldVec.v[0] = (float)as.negateX * cannonicalVec.v[ as.xSrc ];
	worldVec.v[1] = (float)as.negateY * cannonicalVec.v[ as.ySrc ];
	worldVec.v[2] = cannonicalVec.v[2];
}
