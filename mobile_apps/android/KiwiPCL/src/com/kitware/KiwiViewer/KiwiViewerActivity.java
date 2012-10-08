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

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;
import java.util.ArrayList;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.res.Configuration;
import android.content.DialogInterface;
import android.content.DialogInterface.OnDismissListener;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.os.Build;
import android.os.Environment;
import android.os.AsyncTask;
import android.util.Log;
import android.text.InputType;
import android.text.SpannableString;
import android.text.util.Linkify;
import android.text.method.LinkMovementMethod;
import android.net.Uri;
import android.view.View;
import android.view.LayoutInflater;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.app.ProgressDialog;

public class KiwiViewerActivity extends Activity {

    public static final String TAG = "KiwiViewerActivity";

    protected KiwiGLSurfaceView mView;

    protected ImageButton  mLoadButton;
    protected ImageButton  mInfoButton;
    protected ImageButton  mResetViewButton;

    protected ArrayList<String> mBuiltinDatasetNames;

    protected String fileToOpen;
    protected int datasetToOpen = -1;

    protected ProgressDialog mProgressDialog = null;

    public static final int DATASETTABLE_REQUEST_CODE = 1;


    protected void showProgressDialog() {
      showProgressDialog("Opening data...");
    }

    protected void showProgressDialog(String message) {
      mProgressDialog = new ProgressDialog(this);
      mProgressDialog.setIndeterminate(true);
      mProgressDialog.setCancelable(false);
      mProgressDialog.setMessage(message);
      mProgressDialog.show();
    }


    public void dismissProgressDialog() {
      if (mProgressDialog != null) {
        mProgressDialog.dismiss();
      }
    }


    public void showErrorDialog(String title, String message) {

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setIcon(R.drawable.alert_dialog_icon);
      dialog.setTitle(title);
      dialog.setMessage(message);
      dialog.setButton("Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});
      dialog.show();
    }


    public void showWelcomeDialog() {

      String title = getString(R.string.welcome_title);
      String message = getString(R.string.welcome_message);

      final SpannableString s = new SpannableString(message);
      Linkify.addLinks(s, Linkify.WEB_URLS);

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setTitle(title);
      dialog.setIcon(R.drawable.kiwi_small_icon);
      dialog.setMessage(s);
      dialog.setButton("Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});

      dialog.setOnDismissListener(new OnDismissListener() {
        @Override
        public void onDismiss(final DialogInterface iface) {
           maybeLoadDefaultDataset();
        }});

      dialog.show();

      ((TextView)dialog.findViewById(android.R.id.message)).setMovementMethod(LinkMovementMethod.getInstance());
    }


    public void showBrainAtlasDialog() {

      String title = getString(R.string.brainatlas_title);
      String message = getString(R.string.brainatlas_message);

      final SpannableString s = new SpannableString(message);
      Linkify.addLinks(s, Linkify.WEB_URLS);

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setIcon(R.drawable.info_icon);
      dialog.setTitle(title);
      dialog.setMessage(s);
      dialog.setButton("Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});

      dialog.show();

      ((TextView)dialog.findViewById(android.R.id.message)).setMovementMethod(LinkMovementMethod.getInstance());

    }

    public void showCanDialog() {

      String title = getString(R.string.can_title);
      String message = getString(R.string.can_message);

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setIcon(R.drawable.info_icon);
      dialog.setTitle(title);
      dialog.setMessage(message);
      dialog.setButton("Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});

      dialog.show();
    }

    public void showHeadImageDialog() {

      String title = getString(R.string.head_image_title);
      String message = getString(R.string.head_image_message);

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setIcon(R.drawable.info_icon);
      dialog.setTitle(title);
      dialog.setMessage(message);
      dialog.setButton("Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});

      dialog.show();
    }

    public void showCannotOpenAssetDialog() {

      String title = getString(R.string.cannot_open_asset_title);
      String message = getString(R.string.cannot_open_asset_message);

      AlertDialog dialog = new AlertDialog.Builder(this).create();
      dialog.setIcon(R.drawable.alert_dialog_icon);
      dialog.setTitle(title);
      dialog.setMessage(message);
      dialog.setButton(AlertDialog.BUTTON_NEUTRAL, "Ok",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
        return;
        }});

      dialog.setButton(AlertDialog.BUTTON_POSITIVE, "Open in Browser",  new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
          openUrlInBrowser(getString(R.string.external_data_url));
        }});

      dialog.show();
    }

    protected void openUrlInBrowser(String url) {
        Intent intent = new Intent(Intent.ACTION_VIEW, Uri.parse(url));
        startActivity(intent);
    }


    protected void handleUriFromIntent(Uri uri) {
      if (uri != null) {
        if (uri.getScheme().equals("file")) {
          fileToOpen = uri.getPath();
        }
      }
    }


    @Override protected void onNewIntent(Intent intent) {
      super.onNewIntent(intent);
      handleUriFromIntent(intent.getData());
    }



    protected void initBuiltinDatasetNames() {

      if (mBuiltinDatasetNames == null) {
          int numberOfDatasets = KiwiNative.getNumberOfBuiltinDatasets();
          mBuiltinDatasetNames = new ArrayList<String>();
          for(int i = 0; i < numberOfDatasets; ++i) {
            mBuiltinDatasetNames.add(KiwiNative.getDatasetName(i));
          }
      }
    }

    void maybeLoadDefaultDataset() {

      if (getIntent().getData() == null) {
        String storageDir = getExternalFilesDir(null).getAbsolutePath();
        mView.postLoadDefaultDataset(this, storageDir);
      }
      else {
        KiwiNative.clearExistingDataset();
      }
    }


    @Override
    public void onConfigurationChanged(Configuration newConfig) {
      super.onConfigurationChanged(newConfig);
      mView.stopRendering();
    }


    @Override
    protected void onCreate(Bundle bundle) {
      super.onCreate(bundle);

      handleUriFromIntent(getIntent().getData());

      this.setContentView(R.layout.kiwivieweractivity);

     
      
      mView = (KiwiGLSurfaceView) this.findViewById(R.id.glSurfaceView);
      String ic0=copyAssetFileToStorage("inputCloud0.pcd");
      String ic1=copyAssetFileToStorage("inputCloud1.pcd");
      KiwiNative.PCLPath(ic0, 0);
      KiwiNative.PCLPath(ic1, 1);
      SharedPreferences prefs = getPreferences(MODE_PRIVATE);
      String versionStr = getString(R.string.version_string);

      if (!versionStr.equals(prefs.getString("version_string", ""))) {
        prefs.edit().putString("version_string", versionStr).commit();
        showWelcomeDialog();
      }
      else {
        maybeLoadDefaultDataset();
      }


      mLoadButton = (ImageButton) this.findViewById(R.id.loadDataButton);
      mInfoButton = (ImageButton) this.findViewById(R.id.infoButton);
      mResetViewButton = (ImageButton) this.findViewById(R.id.resetButton);


      mLoadButton.setOnClickListener(new Button.OnClickListener() {
          public void onClick(View v) {
              Intent datasetTableIntent = new Intent();
              datasetTableIntent.setClass(KiwiViewerActivity.this, DatasetListActivity.class);
              initBuiltinDatasetNames();
              datasetTableIntent.putExtra("com.kitware.KiwiViewer.bundle.DatasetList", mBuiltinDatasetNames);
              startActivityForResult(datasetTableIntent, DATASETTABLE_REQUEST_CODE);
          }
      });

      mInfoButton.setOnClickListener(new Button.OnClickListener() {
          public void onClick(View v) {

              Intent infoIntent = new Intent();
              infoIntent.setClass(KiwiViewerActivity.this, InfoActivity.class);
              startActivity(infoIntent);
          }
      });

      mResetViewButton.setOnClickListener(new Button.OnClickListener() {
          public void onClick(View v) {
              mView.resetCamera();
          }
      });

    }


    private void copyFile(InputStream in, OutputStream out) throws IOException {
        byte[] buffer = new byte[1024];
        int read;
        while((read = in.read(buffer)) != -1) {
          out.write(buffer, 0, read);
        }
    }


    public String copyAssetFileToStorage(String filename) {

      // todo- check storage state first, show alert dialog in case of problem
      // Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)
      //MEDIA_MOUNTED_READ_ONLY

      String storageDir = getExternalFilesDir(null).getAbsolutePath();

      String destFilename = storageDir + "/" + filename;

      File destFile = new File(destFilename);
      if (destFile.exists()) {
        return destFilename;
      }


      InputStream in = null;
      OutputStream out = null;
      try {
        in = getAssets().open(filename);
        out = new FileOutputStream(storageDir + "/" + filename);
        copyFile(in, out);
        in.close();
        in = null;
        out.flush();
        out.close();
        out = null;
      }
      catch(Exception e) {
        Log.e(TAG, e.getMessage());
      }

      return destFilename;
    }


    private class BuiltinDataLoader extends AsyncTask<String, Integer, String> {

      public int mBuiltinDatasetIndex;

      BuiltinDataLoader(int builtinDatasetIndex) {
        mBuiltinDatasetIndex = builtinDatasetIndex;
      }

      protected String doInBackground(String... filename) {

        if (filename[0].equals("textured_sphere.vtp")) {
          copyEarthAssets();
        }

        return copyAssetFileToStorage(filename[0]);
      }

      protected void onPreExecute() {
        showProgressDialog();
      }

      protected void onPostExecute(String filename) {
        mView.loadDataset(filename, mBuiltinDatasetIndex, KiwiViewerActivity.this);
      }
    }


    public void loadDataset(int builtinDatasetIndex) {

      String filename = KiwiNative.getDatasetFilename(builtinDatasetIndex);

      // don't attempt to open large asset files on android api 8
      int sdkVersion = Build.VERSION.SDK_INT;
      if (sdkVersion <= 8
          && (filename.equals("visible-woman-hand.vtp")
              || filename.equals("AppendedKneeData.vtp")
              || filename.equals("cturtle.vtp")
              || filename.equals("model_info.txt"))) {
        showCannotOpenAssetDialog();
        return;
      }

      new BuiltinDataLoader(builtinDatasetIndex).execute(filename);
    }


    public void loadDataset(String filename) {
      showProgressDialog();
      mView.loadDataset(filename, KiwiViewerActivity.this);
    }

    public void postLoadDataset(String filename, boolean result, String errorTitle, String errorMessage) {
      dismissProgressDialog();
      if (!result) {
        showErrorDialog(errorTitle, errorMessage);
      }
      else {
        if (filename.endsWith("model_info.txt")) {
          showBrainAtlasDialog();
        }
        else if (filename.endsWith("can0000.vtp")) {
          showCanDialog();
        }
        else if (filename.endsWith("head.vti")) {
          showHeadImageDialog();
        }
      }
    }


    @Override protected void onPause() {
        super.onPause();
        mView.onPause();
    }

    @Override protected void onResume() {
        super.onResume();
        mView.onResume();

        if (fileToOpen != null) {
          loadDataset(fileToOpen);
          fileToOpen = null;
        }
        else if (datasetToOpen >= 0) {
          loadDataset(datasetToOpen);
          datasetToOpen = -1;
        }
    }

    /**
     * Get results from the dataset dialog
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
      Bundle curBundle = null;

      if (data != null) {
        curBundle = data.getExtras();
      }
      if (requestCode == DATASETTABLE_REQUEST_CODE && curBundle != null
          && curBundle.containsKey("com.kitware.KiwiViewer.bundle.DatasetName")) {

        String name = curBundle.getString("com.kitware.KiwiViewer.bundle.DatasetName");
        int offset = curBundle.getInt("com.kitware.KiwiViewer.bundle.DatasetOffset");
        datasetToOpen = offset;
      }

      super.onActivityResult(requestCode, resultCode, data);
    }


  protected void copyEarthAssets() {
    copyAssetFileToStorage("earth.jpg");
  }

}
