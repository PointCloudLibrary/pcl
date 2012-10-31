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

package com.itseez.icpdemo;

import java.nio.ShortBuffer;

import android.content.res.AssetManager;

public class KiwiNative {

     static {
         System.loadLibrary("KiwiNative");
     }

     public static native synchronized void init(int width, int height);
     public static native synchronized void reshape(int width, int height);
     public static native synchronized void handleSingleTouchPanGesture(float dx, float dy);
     public static native synchronized void handleTwoTouchPanGesture(float x0, float y0, float x1, float y1);
     public static native synchronized void handleTwoTouchPinchGesture(float scale);
     public static native synchronized void handleTwoTouchRotationGesture(float rotation);
     public static native synchronized void handleSingleTouchUp();
     public static native synchronized void handleSingleTouchDown(float x, float y);
     public static native synchronized void handleSingleTouchTap(float x, float y);
     public static native synchronized void handleDoubleTap(float x, float y);
     public static native synchronized void handleLongPress(float x, float y);
     public static native synchronized boolean render();
     public static native synchronized void resetCamera();
     public static native synchronized void stopInertialMotion();
     public static native synchronized String getDatasetName(int offset);
     public static native synchronized String getDatasetFilename(int offset);
     public static native synchronized int getNumberOfBuiltinDatasets();
     public static native synchronized int getDefaultBuiltinDatasetIndex();
     public static native synchronized boolean getDatasetIsLoaded();
     public static native synchronized void clearExistingDataset();
     public static native synchronized boolean loadDataset(String filename, int builtinDatasetIndex);
     public static native synchronized void checkForAdditionalDatasets(String storageDir);
     public static native synchronized String getLoadDatasetErrorTitle();
     public static native synchronized String getLoadDatasetErrorMessage();
     public static native synchronized boolean PCLPath(String filename, int id);
     public static native synchronized int getNumberOfTriangles();
     public static native synchronized int getNumberOfLines();
     public static native synchronized int getNumberOfVertices();
     public static native synchronized int getFramesPerSecond();
     public static native synchronized int myassets(AssetManager mgr);
     public static native synchronized void showDepthFromDevice(ShortBuffer buf, int maxZ,int frame_width, int frame_height);
     
}
