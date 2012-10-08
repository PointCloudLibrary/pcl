/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.

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
 * Copyright (C) 2009 The Android Open Source Project
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
 */

package com.kitware.KiwiViewer;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.egl.EGLContext;
import javax.microedition.khronos.egl.EGLDisplay;
import javax.microedition.khronos.opengles.GL10;

import org.metalev.multitouch.controller.MultiTouchController;
import org.metalev.multitouch.controller.MultiTouchController.MultiTouchObjectCanvas;
import org.metalev.multitouch.controller.MultiTouchController.PointInfo;
import org.metalev.multitouch.controller.MultiTouchController.PositionAndScale;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.GestureDetector;
import android.view.GestureDetector.SimpleOnGestureListener;
import android.view.MotionEvent;


public class KiwiGLSurfaceView extends GLSurfaceView implements MultiTouchObjectCanvas<KiwiGLSurfaceView> {

    private static String TAG = "KiwiViewer";
    private static final boolean DEBUG = false;

    private MyRenderer mRenderer;

    private GestureDetector mGestureDetector;

    private MultiTouchController<KiwiGLSurfaceView> multiTouchController = new MultiTouchController<KiwiGLSurfaceView>(this);

    private PointInfo mLastTouchInfo = null;
    private PointInfo mCurrentTouchInfo = new PointInfo();


    public class MyRunnable implements Runnable {

      public float dx, dy, x0, y0, x1, y1, scale, angle;
      public boolean isMulti;

      public void run() {

        if (isMulti) {
          KiwiNative.handleTwoTouchPanGesture(x0, y0, x1, y1);
        }
        else {
          KiwiNative.handleSingleTouchPanGesture(dx, dy);
        }

        if (isMulti && scale != 1.0f) {
          KiwiNative.handleTwoTouchPinchGesture(scale);
        }

        if (isMulti && angle != 0.0f) {
          KiwiNative.handleTwoTouchRotationGesture(angle);
        }

        requestRender();
      }

    }


    public class MyGestureDetector extends SimpleOnGestureListener {
        @Override
        public boolean onDoubleTap(MotionEvent e) {

          final float displayX = e.getX();
          final float displayY = e.getY();

          queueEvent(new Runnable() {
                   public void run() {
                      KiwiNative.handleDoubleTap(displayX, displayY);
                      requestRender();
                   }});

          return true;
        }

        public void onLongPress(MotionEvent e) {

          final float displayX = e.getX();
          final float displayY = e.getY();

          queueEvent(new Runnable() {
                   public void run() {
                      KiwiNative.handleLongPress(displayX, displayY);
                      requestRender();
                   }});

        }

        public boolean onSingleTapConfirmed(MotionEvent e) {


          final float displayX = e.getX();
          final float displayY = e.getY();

          queueEvent(new Runnable() {
                   public void run() {
                      KiwiNative.handleSingleTouchTap(displayX, displayY);
                      requestRender();
                   }});

          return true;
        }

    }



    public KiwiGLSurfaceView(Context context) {
      super(context);
      init();
    }


    public KiwiGLSurfaceView(Context context, AttributeSet attrs) {
      super(context, attrs);
      init();
    }

    private void init() {
      mGestureDetector = new GestureDetector(new MyGestureDetector());
      tryPreserveEGLContext();
      initEGL(true, 20, 2);
    }


    private void tryPreserveEGLContext() {

      try {
        Method preserveEGLContextMethod =
          KiwiGLSurfaceView.class.getMethod("setPreserveEGLContextOnPause",
                                            new Class[] {boolean.class});
          try {
            preserveEGLContextMethod.invoke(KiwiGLSurfaceView.this, true);
           }
          catch (InvocationTargetException ite) {
            Log.i(TAG, String.format("exception invoking setPreserveEGLContextOnPause()"));
          }
          catch (IllegalAccessException ie) {
            Log.i(TAG, String.format("exception accessing setPreserveEGLContextOnPause()"));
          }
      }
      catch (NoSuchMethodException nsme) {
        Log.i(TAG, String.format("api does not have setPreserveEGLContextOnPause()"));
      }
    }


    @Override
    public void onPause() {
      stopRendering();
      super.onPause();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
      mGestureDetector.onTouchEvent(event);
      return multiTouchController.onTouchEvent(event);
    }


    @Override
    public void queueEvent(Runnable r) {

      if (mRenderer.isInitialized) {
        super.queueEvent(r);
      }
      else {
        mRenderer.queuePostInitEvent(r);
      }
    }

    public void postLoadDefaultDataset(final KiwiViewerActivity loader, final String storageDir) {

        this.queueEvent(new Runnable() {
           public void run() {

              KiwiNative.checkForAdditionalDatasets(storageDir);

              if (!KiwiNative.getDatasetIsLoaded()) {
                final int defaultDatasetIndex = KiwiNative.getDefaultBuiltinDatasetIndex();
                KiwiGLSurfaceView.this.post(new Runnable() {
                    public void run() {
                      loader.loadDataset(defaultDatasetIndex);
                    }
                });
              }

           }});
    }

    public void selectObject(KiwiGLSurfaceView obj, PointInfo touchPoint) {

      if (obj == null) {
        this.queueEvent(new Runnable() {
                 public void run() {
                    KiwiNative.handleSingleTouchUp();
                    requestRender();
                 }});
      }
    }


    public KiwiGLSurfaceView getDraggableObjectAtPoint(PointInfo touchPoint) {

      final float x = touchPoint.getX();
      final float y = touchPoint.getY();

      this.queueEvent(new Runnable() {
                 public void run() {
                    KiwiNative.handleSingleTouchDown(x, y);
                    requestRender();
                 }});

      return this;
    }


    public void getPositionAndScale(KiwiGLSurfaceView obj, PositionAndScale pos) {

      float xOff = 0.0f;
      float yOff = 0.0f;
      float scale = 1.0f;
      float scaleX = 1.0f;
      float scaleY = 1.0f;
      float angle = 0.0f;
      boolean updateScale = true;
      boolean updateAngle = true;
      boolean updateScaleXY = false;

      pos.set(xOff, yOff, updateScale, scale, updateScaleXY, scaleX, scaleY, updateAngle, angle);

      this.mLastTouchInfo = null;
    }


    public boolean setPositionAndScale(KiwiGLSurfaceView obj, PositionAndScale pos, PointInfo info) {

      if (this.mLastTouchInfo == null) {
        this.mLastTouchInfo = new PointInfo();
        this.mLastTouchInfo.set(info);
        return true;
      }

      this.mCurrentTouchInfo.set(info);

      boolean isMulti = mLastTouchInfo.isMultiTouch();

      float dx = mCurrentTouchInfo.getX() - mLastTouchInfo.getX();
      float dy = mCurrentTouchInfo.getY() - mLastTouchInfo.getY();
      float x1 = mCurrentTouchInfo.getX();
      float y1 = mCurrentTouchInfo.getY();
      float x0 = x1 - dx;
      float y0 = y1 + dy;
      float scale = 1.0f;
      float angle = 0.0f;

      if (isMulti && mLastTouchInfo.getMultiTouchDiameter() != 0.0f) {
        scale = mCurrentTouchInfo.getMultiTouchDiameter() / mLastTouchInfo.getMultiTouchDiameter();
      }

      if (isMulti) {
        angle = mCurrentTouchInfo.getMultiTouchAngle() - mLastTouchInfo.getMultiTouchAngle();
      }

      MyRunnable myrun = new MyRunnable();
      myrun.dx = dx;
      myrun.dy = dy;
      myrun.x0 = x0;
      myrun.y0 = y0;
      myrun.x1 = x1;
      myrun.y1 = y1;
      myrun.scale = scale;
      myrun.angle = angle;
      myrun.isMulti = isMulti;
      this.queueEvent(myrun);

      mLastTouchInfo.set(mCurrentTouchInfo);
      return true;
    }

    public void loadDataset(final String filename, final KiwiViewerActivity loader) {
      int builtinDatasetIndex = -1;
      loadDataset(filename, builtinDatasetIndex, loader);
    }

    public void loadDataset(final String filename, final int builtinDatasetIndex, final KiwiViewerActivity loader) {

      queueEvent(new Runnable() {
        public void run() {
     
          final boolean result = KiwiNative.loadDataset(filename, builtinDatasetIndex);
          final String errorTitle = KiwiNative.getLoadDatasetErrorTitle();
          final String errorMessage = KiwiNative.getLoadDatasetErrorMessage();

          requestRender();

          KiwiGLSurfaceView.this.post(new Runnable() {
            public void run() {
              loader.postLoadDataset(filename, result, errorTitle, errorMessage);
            }});
        }});
    }


    public void resetCamera() {
      queueEvent(new Runnable() {
                   public void run() {
                      KiwiNative.resetCamera();
                      requestRender();
                   }});
    }

    public void stopRendering() {
    
      queueEvent(new Runnable() {
                   public void run() {
                      KiwiNative.stopInertialMotion();
                      setRenderMode(RENDERMODE_WHEN_DIRTY);
                   }});
    }


    private void initEGL(boolean translucent, int depth, int stencil) {


        /* By default, GLSurfaceView() creates a RGB_565 opaque surface.
         * If we want a translucent one, we should change the surface's
         * format here, using PixelFormat.TRANSLUCENT for GL Surfaces
         * is interpreted as any 32-bit surface with alpha by SurfaceFlinger.
         */
        if (translucent) {
            this.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        }

        /* Setup the context factory for 2.0 rendering.
         * See ContextFactory class definition below
         */
        setEGLContextFactory(new ContextFactory());

        /* We need to choose an EGLConfig that matches the format of
         * our surface exactly. This is going to be done in our
         * custom config chooser. See ConfigChooser class definition
         * below.
         */
        setEGLConfigChooser( translucent ?
                             new ConfigChooser(8, 8, 8, 8, depth, stencil) :
                             new ConfigChooser(5, 6, 5, 0, depth, stencil) );

        /* Set the renderer responsible for frame rendering */
        mRenderer = new MyRenderer();
        mRenderer.parentView = this;
        setRenderer(mRenderer);
        setRenderMode(RENDERMODE_WHEN_DIRTY);

        requestRender();
    }

    private static class ContextFactory implements GLSurfaceView.EGLContextFactory {
        private static int EGL_CONTEXT_CLIENT_VERSION = 0x3098;
        public EGLContext createContext(EGL10 egl, EGLDisplay display, EGLConfig eglConfig) {
            Log.w(TAG, "creating OpenGL ES 2.0 context");
            checkEglError("Before eglCreateContext", egl);
            int[] attrib_list = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL10.EGL_NONE };
            EGLContext context = egl.eglCreateContext(display, eglConfig, EGL10.EGL_NO_CONTEXT, attrib_list);
            checkEglError("After eglCreateContext", egl);
            return context;
        }

        public void destroyContext(EGL10 egl, EGLDisplay display, EGLContext context) {
            egl.eglDestroyContext(display, context);
        }
    }

    private static void checkEglError(String prompt, EGL10 egl) {
        int error;
        while ((error = egl.eglGetError()) != EGL10.EGL_SUCCESS) {
            Log.e(TAG, String.format("%s: EGL error: 0x%x", prompt, error));
        }
    }

    private static class ConfigChooser implements GLSurfaceView.EGLConfigChooser {

        public ConfigChooser(int r, int g, int b, int a, int depth, int stencil) {
            mRedSize = r;
            mGreenSize = g;
            mBlueSize = b;
            mAlphaSize = a;
            mDepthSize = depth;
            mStencilSize = stencil;
        }

        /* This EGL config specification is used to specify 2.0 rendering.
         * We use a minimum size of 4 bits for red/green/blue, but will
         * perform actual matching in chooseConfig() below.
         */
        private static int EGL_OPENGL_ES2_BIT = 4;
        private static int[] s_configAttribs2 =
        {
            EGL10.EGL_RED_SIZE, 4,
            EGL10.EGL_GREEN_SIZE, 4,
            EGL10.EGL_BLUE_SIZE, 4,
            EGL10.EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL10.EGL_NONE
        };

        public EGLConfig chooseConfig(EGL10 egl, EGLDisplay display) {

            /* Get the number of minimally matching EGL configurations
             */
            int[] num_config = new int[1];
            egl.eglChooseConfig(display, s_configAttribs2, null, 0, num_config);

            int numConfigs = num_config[0];

            if (numConfigs <= 0) {
                throw new IllegalArgumentException("No configs match configSpec");
            }

            /* Allocate then read the array of minimally matching EGL configs
             */
            EGLConfig[] configs = new EGLConfig[numConfigs];
            egl.eglChooseConfig(display, s_configAttribs2, configs, numConfigs, num_config);

            if (DEBUG) {
                 printConfigs(egl, display, configs);
            }
            /* Now return the "best" one
             */
            return chooseConfig(egl, display, configs);
        }

        public EGLConfig chooseConfig(EGL10 egl, EGLDisplay display,
                EGLConfig[] configs) {
            for(EGLConfig config : configs) {
                int d = findConfigAttrib(egl, display, config,
                        EGL10.EGL_DEPTH_SIZE, 0);
                int s = findConfigAttrib(egl, display, config,
                        EGL10.EGL_STENCIL_SIZE, 0);

                // We need at least mDepthSize and mStencilSize bits
                if (d < mDepthSize || s < mStencilSize)
                    continue;

                // We want an *exact* match for red/green/blue/alpha
                int r = findConfigAttrib(egl, display, config,
                        EGL10.EGL_RED_SIZE, 0);
                int g = findConfigAttrib(egl, display, config,
                            EGL10.EGL_GREEN_SIZE, 0);
                int b = findConfigAttrib(egl, display, config,
                            EGL10.EGL_BLUE_SIZE, 0);
                int a = findConfigAttrib(egl, display, config,
                        EGL10.EGL_ALPHA_SIZE, 0);

                if (r == mRedSize && g == mGreenSize && b == mBlueSize && a == mAlphaSize) {

                    printConfig(egl, display, config);
                    return config;
                }
            }
            return null;
        }

        private int findConfigAttrib(EGL10 egl, EGLDisplay display,
                EGLConfig config, int attribute, int defaultValue) {

            if (egl.eglGetConfigAttrib(display, config, attribute, mValue)) {
                return mValue[0];
            }
            return defaultValue;
        }

        private void printConfigs(EGL10 egl, EGLDisplay display,
            EGLConfig[] configs) {
            int numConfigs = configs.length;
            Log.w(TAG, String.format("%d configurations", numConfigs));
            for (int i = 0; i < numConfigs; i++) {
                Log.w(TAG, String.format("Configuration %d:\n", i));
                printConfig(egl, display, configs[i]);
            }
        }

        private void printConfig(EGL10 egl, EGLDisplay display,
                EGLConfig config) {
            int[] attributes = {
                    EGL10.EGL_BUFFER_SIZE,
                    EGL10.EGL_ALPHA_SIZE,
                    EGL10.EGL_BLUE_SIZE,
                    EGL10.EGL_GREEN_SIZE,
                    EGL10.EGL_RED_SIZE,
                    EGL10.EGL_DEPTH_SIZE,
                    EGL10.EGL_STENCIL_SIZE,
                    EGL10.EGL_CONFIG_CAVEAT,
                    EGL10.EGL_CONFIG_ID,
                    EGL10.EGL_LEVEL,
                    EGL10.EGL_MAX_PBUFFER_HEIGHT,
                    EGL10.EGL_MAX_PBUFFER_PIXELS,
                    EGL10.EGL_MAX_PBUFFER_WIDTH,
                    EGL10.EGL_NATIVE_RENDERABLE,
                    EGL10.EGL_NATIVE_VISUAL_ID,
                    EGL10.EGL_NATIVE_VISUAL_TYPE,
                    0x3030, // EGL10.EGL_PRESERVED_RESOURCES,
                    EGL10.EGL_SAMPLES,
                    EGL10.EGL_SAMPLE_BUFFERS,
                    EGL10.EGL_SURFACE_TYPE,
                    EGL10.EGL_TRANSPARENT_TYPE,
                    EGL10.EGL_TRANSPARENT_RED_VALUE,
                    EGL10.EGL_TRANSPARENT_GREEN_VALUE,
                    EGL10.EGL_TRANSPARENT_BLUE_VALUE,
                    0x3039, // EGL10.EGL_BIND_TO_TEXTURE_RGB,
                    0x303A, // EGL10.EGL_BIND_TO_TEXTURE_RGBA,
                    0x303B, // EGL10.EGL_MIN_SWAP_INTERVAL,
                    0x303C, // EGL10.EGL_MAX_SWAP_INTERVAL,
                    EGL10.EGL_LUMINANCE_SIZE,
                    EGL10.EGL_ALPHA_MASK_SIZE,
                    EGL10.EGL_COLOR_BUFFER_TYPE,
                    EGL10.EGL_RENDERABLE_TYPE,
                    0x3042 // EGL10.EGL_CONFORMANT
            };
            String[] names = {
                    "EGL_BUFFER_SIZE",
                    "EGL_ALPHA_SIZE",
                    "EGL_BLUE_SIZE",
                    "EGL_GREEN_SIZE",
                    "EGL_RED_SIZE",
                    "EGL_DEPTH_SIZE",
                    "EGL_STENCIL_SIZE",
                    "EGL_CONFIG_CAVEAT",
                    "EGL_CONFIG_ID",
                    "EGL_LEVEL",
                    "EGL_MAX_PBUFFER_HEIGHT",
                    "EGL_MAX_PBUFFER_PIXELS",
                    "EGL_MAX_PBUFFER_WIDTH",
                    "EGL_NATIVE_RENDERABLE",
                    "EGL_NATIVE_VISUAL_ID",
                    "EGL_NATIVE_VISUAL_TYPE",
                    "EGL_PRESERVED_RESOURCES",
                    "EGL_SAMPLES",
                    "EGL_SAMPLE_BUFFERS",
                    "EGL_SURFACE_TYPE",
                    "EGL_TRANSPARENT_TYPE",
                    "EGL_TRANSPARENT_RED_VALUE",
                    "EGL_TRANSPARENT_GREEN_VALUE",
                    "EGL_TRANSPARENT_BLUE_VALUE",
                    "EGL_BIND_TO_TEXTURE_RGB",
                    "EGL_BIND_TO_TEXTURE_RGBA",
                    "EGL_MIN_SWAP_INTERVAL",
                    "EGL_MAX_SWAP_INTERVAL",
                    "EGL_LUMINANCE_SIZE",
                    "EGL_ALPHA_MASK_SIZE",
                    "EGL_COLOR_BUFFER_TYPE",
                    "EGL_RENDERABLE_TYPE",
                    "EGL_CONFORMANT"
            };
            int[] value = new int[1];
            for (int i = 0; i < attributes.length; i++) {
                int attribute = attributes[i];
                String name = names[i];
                if ( egl.eglGetConfigAttrib(display, config, attribute, value)) {
                    Log.w(TAG, String.format("  %s: %d\n", name, value[0]));
                } else {
                    // Log.w(TAG, String.format("  %s: failed\n", name));
                    while (egl.eglGetError() != EGL10.EGL_SUCCESS);
                }
            }
        }

        // Subclasses can adjust these values:
        protected int mRedSize;
        protected int mGreenSize;
        protected int mBlueSize;
        protected int mAlphaSize;
        protected int mDepthSize;
        protected int mStencilSize;
        private int[] mValue = new int[1];
    }

}


class MyRenderer implements GLSurfaceView.Renderer {

  public GLSurfaceView parentView;
  public boolean isInitialized = false;
  public ArrayList<Runnable> mPostInitRunnables = new ArrayList<Runnable>();
  public ArrayList<Runnable> mPreRenderRunnables = new ArrayList<Runnable>();

  synchronized void queuePostInitEvent(Runnable runnable) {
    mPostInitRunnables.add(runnable);
  }

  synchronized void queuePreRenderEvent(Runnable runnable) {
    mPreRenderRunnables.add(runnable);
  }

  public void onDrawFrame(GL10 gl) {

      boolean result = KiwiNative.render();
      if (result) {
        parentView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
      }
      else {
        parentView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
      }

      while (mPreRenderRunnables.size() > 0) {
        mPreRenderRunnables.remove(0).run();
      }
  }

  public void onSurfaceChanged(GL10 gl, int width, int height) {
      KiwiNative.reshape(width, height);
  }

  private AssetManager mgr;

  
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {

	  mgr=parentView.getResources().getAssets();
	
	  
	  KiwiNative.init(100, 100);
	  KiwiNative.myassets(mgr);
      isInitialized = true;

      while (mPostInitRunnables.size() > 0) {
        mPostInitRunnables.remove(0).run();
      }
  }
}

