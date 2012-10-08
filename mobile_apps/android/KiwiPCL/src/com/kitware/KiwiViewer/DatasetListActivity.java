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
import android.view.WindowManager.LayoutParams;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.widget.ListView;

import java.util.ArrayList;

/**
 *
 *
 */
public class DatasetListActivity extends Activity {

  public ListView mListView;

  @Override
  protected void onCreate(Bundle savedInstanceState) {

    super.onCreate(savedInstanceState);

    // Make us non-modal, so that others can receive touch events.
    getWindow().setFlags(LayoutParams.FLAG_NOT_TOUCH_MODAL, LayoutParams.FLAG_NOT_TOUCH_MODAL);

    // ...but notify us that it happened.
    getWindow().setFlags(LayoutParams.FLAG_WATCH_OUTSIDE_TOUCH, LayoutParams.FLAG_WATCH_OUTSIDE_TOUCH);

    setTitle("Select Registration Method");

    this.setContentView(R.layout.datasetlistactivity);
    ArrayList<String> datasets = getIntent().getExtras().getStringArrayList("com.kitware.KiwiViewer.bundle.DatasetList");
    ArrayAdapter<String> adapter = new ArrayAdapter<String>(getBaseContext(),android.R.layout.simple_list_item_1,datasets);
    mListView = (ListView) findViewById(R.id.listView);
    mListView.setAdapter(adapter);
    mListView.setOnItemClickListener(new OnItemClickListener() {

      @Override
      public void onItemClick(AdapterView<?> adapterView, View view, int pos,
          long arg3) {
        String value = (String) adapterView.getItemAtPosition(pos);
        Intent curIntent = new Intent();
        curIntent.putExtra("com.kitware.KiwiViewer.bundle.DatasetName",value);
        curIntent.putExtra("com.kitware.KiwiViewer.bundle.DatasetOffset",pos);
        setResult(Activity.RESULT_OK, curIntent);
        finish();
      }
    });

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
