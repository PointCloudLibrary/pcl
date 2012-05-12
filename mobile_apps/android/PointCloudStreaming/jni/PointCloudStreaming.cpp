/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.
  Copyright 2012 Willow Garage, Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 ========================================================================*/
/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <jni.h>
#include <errno.h>
#include <fstream>

#include <EGL/egl.h>

#include <android/sensor.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <android/asset_manager.h>

#include <vesCamera.h>
#include <vesKiwiViewerApp.h>
#include <vesShaderProgram.h>
#include <vesKiwiText2DRepresentation.h>
#include <vesKiwiStreamingDataRepresentation.h>
#include <vtksys/SystemTools.hxx>


#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "PointCloudStreaming", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "PointCloudStreaming", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_WARN, "PointCloudStreaming", __VA_ARGS__))


//----------------------------------------------------------------------------
namespace {

class vesKiwiPointCloudApp : public vesKiwiViewerApp {
public:

  vesTypeMacro(vesKiwiPointCloudApp);
  typedef vesKiwiViewerApp Superclass;

  vesKiwiPointCloudApp() : vesKiwiViewerApp()
  {
    mIsConnected = false;
    mText = 0;
    mHost = "";
    mPort = 0;
  }

  ~vesKiwiPointCloudApp()
  {
    this->unloadData();
  }

  void loadSettingsFromFile(const std::string& configFile)
  {
    ifstream f(configFile.c_str());
    if (f.is_open()) {
      f >> mHost >> mPort;
    }
    else {
      LOGI("Failed to read config file: %s", configFile.c_str());
    }
  }

  void initShader(const std::string& vertexSource, const std::string fragmentSource)
  {
    mShader = this->addShaderProgram(vertexSource, fragmentSource);
    this->addModelViewMatrixUniform(mShader);
    this->addProjectionMatrixUniform(mShader);
    this->addVertexPositionAttribute(mShader);
    this->addNormalMatrixUniform(mShader);
    this->addVertexNormalAttribute(mShader);
    this->addVertexColorAttribute(mShader);
  }


  void unloadData()
  {
    if (mDataRep) {
      mDataRep->removeSelfFromRenderer(this->renderer());
      mDataRep.reset();
    }
  }

  void willRender()
  {
    this->Superclass::willRender();

    if (mIsConnected) {
      this->mDataRep->willRender(this->renderer());
    }
    else {
      this->connect(mHost, mPort);
    }
  }

  void showText(const std::string& textStr)
  {
    LOGI("%s", textStr.c_str());

    if (!mText) {
      mText = this->addTextRepresentation(textStr);
      mText->setDisplayPosition(vesVector2f(10, 10));
    }
    else {
      mText->setText(textStr);
    }
  }

  void setHost(const std::string& host)
  {
    mHost = host;
  }

  void setPort(int port)
  {
    mPort = port;
  }

  bool connect(const std::string& host, int port)
  {
    //this->unloadData();
    mIsConnected = false;

    std::stringstream hostPort;
    hostPort << host << ":" << port;
    this->showText("Connecting to " + hostPort.str());

    if (!mDataRep) {
      mDataRep = vesKiwiStreamingDataRepresentation::Ptr(new vesKiwiStreamingDataRepresentation);
    }

    if (!mDataRep->connectToServer(host, port)) {
      this->showText("Connection failed to " + hostPort.str());
      return false;
    }

    this->showText("Connected to " + hostPort.str());
    mIsConnected = true;
    mDataRep->initializeWithShader(mShader);
    mDataRep->addSelfToRenderer(this->renderer());
    this->resetView();
    return true;
  }

  bool mIsConnected;
  int mPort;
  std::string mHost;

  vesShaderProgram::Ptr mShader;
  vesKiwiStreamingDataRepresentation::Ptr mDataRep;
  vesKiwiText2DRepresentation* mText;
};

//----------------------------------------------------------------------------
std::string storageDir;
AAssetManager* assetManager;

//----------------------------------------------------------------------------
std::string documentsDirectory()
{
  assert(storageDir.size());
  return storageDir + "/PointCloudStreaming";
}

//----------------------------------------------------------------------------
std::string copyAssetToExternalStorage(std::string filename)
{
  std::string destDirectory = documentsDirectory();
  std::string destFilename = destDirectory + "/" + filename;

  if (vtksys::SystemTools::FileExists(destFilename.c_str())) {
    return destFilename;
  }

  vtksys::SystemTools::MakeDirectory(destDirectory.c_str());

  LOGI("Reading asset file: %s", filename.c_str());
  AAsset* asset = AAssetManager_open(assetManager, filename.c_str(), AASSET_MODE_UNKNOWN);
  if (asset == NULL) {
      LOGE("Could not open asset: %s", filename.c_str());
      return std::string();
  }

  off_t len = AAsset_getLength(asset);
  const char* input_string = static_cast<const char*>(AAsset_getBuffer(asset));
  LOGI("Asset file is %u bytes", len);

  LOGI("Writing to destination file: %s", destFilename.c_str());
  std::ofstream outfile(destFilename.c_str(), std::ofstream::binary);
  outfile.write(input_string, len);
  outfile.close();
  AAsset_close(asset);

  return destFilename;
}

}


/**
 * Our saved state data.
 */
struct saved_state {
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;

    bool isTwoTouches;
};

/**
 * Shared state for our app.
 */
struct engine {
    struct android_app* app;

    ASensorManager* sensorManager;
    const ASensor* accelerometerSensor;
    ASensorEventQueue* sensorEventQueue;

    int animating;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    int32_t width;
    int32_t height;
    struct saved_state state;

    vesKiwiPointCloudApp::Ptr kiwiApp;
};


/**
 * Initialize an EGL context for the current display.
 */
static int engine_init_display(struct engine* engine) {
    // initialize OpenGL ES and EGL

    /*
     * Here specify the attributes of the desired configuration.
     * Below, we select an EGLConfig with at least 8 bits per color
     * component compatible with on-screen windows
     */
    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_DEPTH_SIZE, 8,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_NONE
    };
    const EGLint context_attribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
    };
    EGLint w, h, dummy, format;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);

    /* Here, the application chooses the configuration it desires. In this
     * sample, we have a very simplified selection process, where we pick
     * the first EGLConfig that matches our criteria */
    eglChooseConfig(display, attribs, &config, 1, &numConfigs);

    /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
     * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
     * As soon as we picked a EGLConfig, we can safely reconfigure the
     * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
    eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);

    ANativeWindow_setBuffersGeometry(engine->app->window, 0, 0, format);

    surface = eglCreateWindowSurface(display, config, engine->app->window, NULL);
    context = eglCreateContext(display, config, NULL, context_attribs);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        LOGW("Unable to eglMakeCurrent");
        return -1;
    }


    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    engine->display = display;
    engine->context = context;
    engine->surface = surface;
    engine->width = w;
    engine->height = h;
    engine->state.x0 = 0;
    engine->state.y0 = 0;
    engine->state.x1 = 0;
    engine->state.y1 = 0;
    engine->state.isTwoTouches = false;


    vesKiwiPointCloudApp::Ptr kiwiApp = vesKiwiPointCloudApp::Ptr(new vesKiwiPointCloudApp);
    kiwiApp->initGL();
    engine->kiwiApp = kiwiApp;

    kiwiApp->resizeView(w, h);
    LOGI("resizeView %d %d\n", w, h);

    storageDir = "/mnt/sdcard";
    assetManager = engine->app->activity->assetManager;


    // initialize shaders
    AAsset* vertex_asset = AAssetManager_open(assetManager, "vesShader_vert.glsl", AASSET_MODE_UNKNOWN);
    AAsset* fragment_asset = AAssetManager_open(assetManager, "vesShader_frag.glsl", AASSET_MODE_UNKNOWN);

    std::string vertex_source = std::string(static_cast<const char*>(AAsset_getBuffer(vertex_asset)), AAsset_getLength(vertex_asset));
    std::string fragment_source = std::string(static_cast<const char*>(AAsset_getBuffer(fragment_asset)), AAsset_getLength(fragment_asset));

    AAsset_close(vertex_asset);
    AAsset_close(fragment_asset);


    std::string configFile = copyAssetToExternalStorage("appConfig.txt");
    kiwiApp->initShader(vertex_source, fragment_source);
    kiwiApp->loadSettingsFromFile(configFile);

    return 0;
}

/**
 * Just the current frame in the display.
 */
static void engine_draw_frame(struct engine* engine) {
    engine->kiwiApp->render();
    eglSwapBuffers(engine->display, engine->surface);
}

/**
 * Tear down the EGL context currently associated with the display.
 */
static void engine_term_display(struct engine* engine) {
    if (engine->display != EGL_NO_DISPLAY) {
        eglMakeCurrent(engine->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (engine->context != EGL_NO_CONTEXT) {
            eglDestroyContext(engine->display, engine->context);
        }
        if (engine->surface != EGL_NO_SURFACE) {
            eglDestroySurface(engine->display, engine->surface);
        }
        eglTerminate(engine->display);
    }
    engine->animating = 0;
    engine->display = EGL_NO_DISPLAY;
    engine->context = EGL_NO_CONTEXT;
    engine->surface = EGL_NO_SURFACE;
}

/**
 * Process the next input event.
 */
static int32_t engine_handle_input(struct android_app* app, AInputEvent* event) {
    struct engine* engine = (struct engine*)app->userData;
    if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {


        if (AMotionEvent_getAction(event) == AMOTION_EVENT_ACTION_DOWN) {
            engine->state.x0 = AMotionEvent_getX(event, 0);
            engine->state.y0 = AMotionEvent_getY(event, 0);
            return 1;
        }

        if (AMotionEvent_getAction(event) == AMOTION_EVENT_ACTION_UP
            && (AMotionEvent_getPointerCount(event) == 1)) {
            engine->kiwiApp->handleSingleTouchUp();
            return 1;
        }

        // When transitioning from one touch to two touches, we record the position
        // of of the new touch and return.  When transitioning from two touches to
        // one touch, we record the position of the single remaining touch and return.
        if (AMotionEvent_getPointerCount(event) > 1) {
            if (!engine->state.isTwoTouches) {
              engine->state.x1 = AMotionEvent_getX(event, 1);
              engine->state.y1 = AMotionEvent_getY(event, 1);
              engine->state.isTwoTouches = true;
              return 1;
            }
        }
        else if (engine->state.isTwoTouches) {
          engine->state.isTwoTouches = false;
          engine->state.x0 = AMotionEvent_getX(event, 0);
          engine->state.y0 = AMotionEvent_getY(event, 0);
          return 1;
        }

        if (AMotionEvent_getPointerCount(event) > 1 && !engine->state.isTwoTouches) {
            return 1;
        }

        float px0 = engine->state.x0;
        float py0 = engine->state.y0;
        float px1 = engine->state.x1;
        float py1 = engine->state.y1;
        float x0 = AMotionEvent_getX(event, 0);
        float y0 = AMotionEvent_getY(event, 0);
        float x1 = x0;
        float y1 = y0;
        if (AMotionEvent_getPointerCount(event) > 1) {
            x1 = AMotionEvent_getX(event, 1);
            y1 = AMotionEvent_getY(event, 1);
        }


        if (AMotionEvent_getPointerCount(event) == 1) {
            float dx0 = x0 - px0;
            float dy0 = y0 - py0;
            engine->kiwiApp->handleSingleTouchPanGesture(dx0, dy0);
        } else {

          int viewHeight = engine->kiwiApp->viewHeight();

          // Average positions of current and previous two touches.
          // Invert y since vesCamera expects y to go in opposite direction.
          float pcx = (px0 + px1)/2.0;
          float pcy = viewHeight - (py0 + py1)/2.0;
          float cx = (x0 + x1)/2.0;
          float cy = viewHeight - (y0 + y1)/2.0;


          engine->kiwiApp->handleTwoTouchPanGesture(pcx, pcy, cx, cy);


            // zoom and rotate too

          double previousDist = sqrt((px0 - px1) *
                                     (px0 - px1) +
                                     (py0 - py1) *
                                     (py0 - py1));
          double currentDist = sqrt((x0 - x1) *
                                    (x0 - x1) +
                                    (y0 - y1) *
                                    (y0 - y1));
          double dy = currentDist - previousDist;
          double dyf = 10.0 * dy / (viewHeight/2.0);
          double factor = pow(1.1, dyf);

          engine->kiwiApp->handleTwoTouchPinchGesture(factor);

          double pi = 3.14159265358979;
          double newAngle = atan2(y0 - y1,
                                  x0 - x1);

          double oldAngle = atan2(py0 - py1,
                                  px0 - px1);

          double rotation = newAngle - oldAngle;

          engine->kiwiApp->handleTwoTouchRotationGesture(rotation);


        }

        engine->state.x0 = x0;
        engine->state.y0 = y0;
        if (AMotionEvent_getPointerCount(event) > 1) {
            engine->state.x1 = x1;
            engine->state.y1 = y1;
        }

        return 1;
    }
    return 0;
}

/**
 * Process the next main command.
 */
static void engine_handle_cmd(struct android_app* app, int32_t cmd) {
    struct engine* engine = (struct engine*)app->userData;
    switch (cmd) {
        case APP_CMD_SAVE_STATE:
            // The system has asked us to save our current state.  Do so.
            engine->app->savedState = malloc(sizeof(struct saved_state));
            *((struct saved_state*)engine->app->savedState) = engine->state;
            engine->app->savedStateSize = sizeof(struct saved_state);
            break;
        case APP_CMD_INIT_WINDOW:
            if (app->window != NULL) {
                int32_t width = ANativeWindow_getWidth(app->window);
                int32_t height = ANativeWindow_getHeight(app->window);
                ANativeWindow_setBuffersGeometry(app->window, width, height, 1);
                LOGI("Window format is now %d",ANativeWindow_getFormat(app->window));
            }
            // The window is being shown, get it ready.
            if (engine->app->window != NULL) {
                engine_init_display(engine);
                engine_draw_frame(engine);
            }
            break;
        case APP_CMD_TERM_WINDOW:
            // The window is being hidden or closed, clean it up.
            engine_term_display(engine);
            break;
        case APP_CMD_GAINED_FOCUS:
            engine->animating = 1;
            // When our app gains focus, we start monitoring the accelerometer.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_enableSensor(engine->sensorEventQueue,
                        engine->accelerometerSensor);
                // We'd like to get 60 events per second (in us).
                ASensorEventQueue_setEventRate(engine->sensorEventQueue,
                        engine->accelerometerSensor, (1000L/60)*1000);
            }
            break;
        case APP_CMD_LOST_FOCUS:
            // When our app loses focus, we stop monitoring the accelerometer.
            // This is to avoid consuming battery while not being used.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_disableSensor(engine->sensorEventQueue,
                        engine->accelerometerSensor);
            }
            // Also stop animating.
            engine->animating = 0;
            engine_draw_frame(engine);
            break;
    }
}

/**
 * This is the main entry point of a native application that is using
 * android_native_app_glue.  It runs in its own thread, with its own
 * event loop for receiving input events and doing other things.
 */
void android_main(struct android_app* state) {
    struct engine engine;

    // Make sure glue isn't stripped.
    app_dummy();

    memset(&engine, 0, sizeof(engine));
    state->userData = &engine;
    state->onAppCmd = engine_handle_cmd;
    state->onInputEvent = engine_handle_input;
    engine.app = state;

    // Prepare to monitor accelerometer
    engine.sensorManager = ASensorManager_getInstance();
    engine.accelerometerSensor = ASensorManager_getDefaultSensor(engine.sensorManager,
            ASENSOR_TYPE_ACCELEROMETER);
    engine.sensorEventQueue = ASensorManager_createEventQueue(engine.sensorManager,
            state->looper, LOOPER_ID_USER, NULL, NULL);

    if (state->savedState != NULL) {
        // We are starting with a previous saved state; restore from it.
        engine.state = *(struct saved_state*)state->savedState;
    }

    // loop waiting for stuff to do.

    while (1) {
        // Read all pending events.
        int ident;
        int events;
        struct android_poll_source* source;

        // If not animating, we will block forever waiting for events.
        // If animating, we loop until all events are read, then continue
        // to draw the next frame of animation.
        while ((ident=ALooper_pollAll(engine.animating ? 0 : -1, NULL, &events,
                (void**)&source)) >= 0) {

            // Process this event.
            if (source != NULL) {
                source->process(state, source);
            }

            // If a sensor has data, process it now.
            if (ident == LOOPER_ID_USER) {
                if (engine.accelerometerSensor != NULL) {
                    ASensorEvent event;
                    while (ASensorEventQueue_getEvents(engine.sensorEventQueue,
                            &event, 1) > 0) {
                        //LOGI("accelerometer: x=%f y=%f z=%f",
                        //        event.acceleration.x, event.acceleration.y,
                        //        event.acceleration.z);
                    }
                }
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                engine_term_display(&engine);
                return;
            }
        }

        if (engine.animating) {
            // Done with events; draw next animation frame.
            // Drawing is throttled to the screen update rate, so there
            // is no need to do timing here.
            engine_draw_frame(&engine);
        }
    }
}
