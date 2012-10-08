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

package com.kitware.KiwiViewer;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;
import android.view.Window;
import android.view.WindowManager.LayoutParams;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.text.method.LinkMovementMethod;


public class InfoActivity extends Activity {


  @Override
  protected void onCreate(Bundle savedInstanceState) {

    super.onCreate(savedInstanceState);

    // Make us non-modal, so that others can receive touch events.
    getWindow().setFlags(LayoutParams.FLAG_NOT_TOUCH_MODAL, LayoutParams.FLAG_NOT_TOUCH_MODAL);

    // ...but notify us that it happened.
    getWindow().setFlags(LayoutParams.FLAG_WATCH_OUTSIDE_TOUCH, LayoutParams.FLAG_WATCH_OUTSIDE_TOUCH);

    // fullscreen with no title bar
    requestWindowFeature(Window.FEATURE_NO_TITLE);
    getWindow().setFlags(LayoutParams.FLAG_FULLSCREEN, LayoutParams.FLAG_FULLSCREEN);


    this.setContentView(R.layout.infoactivity);

    String detailsStr = String.format("Scene statistics:\n  Triangles: %d\n  Lines: %d\n  Vertices: %d\n  Drawing @ %d fps",
      KiwiNative.getNumberOfTriangles(),
      KiwiNative.getNumberOfLines(),
      KiwiNative.getNumberOfVertices(),
      KiwiNative.getFramesPerSecond());

    TextView detailsText = (TextView) findViewById(R.id.detailsText);
    detailsText.setText(detailsStr);

    TextView linkText = (TextView) findViewById(R.id.licenseLink);
    linkText.setMovementMethod(LinkMovementMethod.getInstance());

    TextView copyrightText = (TextView) findViewById(R.id.copyrightText);
    copyrightText.setMovementMethod(LinkMovementMethod.getInstance());


  }

  @Override
  public boolean onTouchEvent(MotionEvent event) {
    // If we've received a touch notification that the user has touched
    // outside the app, finish the activity.
    if (MotionEvent.ACTION_OUTSIDE == event.getAction()) {
      finish();
      return true;
    }

    // Delegate everything else to Activity.
    return super.onTouchEvent(event);
  }

}
