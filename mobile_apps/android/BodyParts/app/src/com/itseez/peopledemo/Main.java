package com.itseez.peopledemo;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLContext;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;

public class Main extends Activity {
    private static final String TAG = "Main";

    ImageView picture;
    TextView timingText;
    private final EGL10 egl = (EGL10) EGLContext.getEGL();
    MainLoop loop;

    private static byte[] readFile(File f) throws IOException {
        byte[] contents = new byte[(int) f.length()];
        InputStream stream = new FileInputStream(f);

        try {
            if (stream.read(contents) != contents.length)
                Log.e(TAG, "Couldn't read the full file.");
        } finally {
            stream.close();
        }

        return contents;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        egl.eglInitialize(egl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY), null);

        picture = (ImageView) findViewById(R.id.picture);
        timingText = (TextView) findViewById(R.id.timing_text);

        ListView color_ref = (ListView) findViewById(R.id.color_list);
        color_ref.setAdapter(new BodyPartLabelAdapter(this, android.R.layout.simple_list_item_1, BodyPartLabel.values(),
                (LayoutInflater) getSystemService(LAYOUT_INFLATER_SERVICE)));

        File[] treeFiles = new File("/mnt/sdcard2/trees").listFiles();
        Arrays.sort(treeFiles);
        byte[][] trees = new byte[treeFiles.length][];

        for (int ti = 0; ti < trees.length; ++ti) {
            try {
                trees[ti] = readFile(treeFiles[ti]);
            } catch (IOException ioe) {
                Log.e(TAG, ioe.getMessage(), ioe);
                return;
            }
        }

        loop = new MainLoop(this, new BodyPartsRecognizer(trees));
    }

    @Override
    public void onDestroy() {
        loop.destroy();
        egl.eglTerminate(egl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY));
        super.onDestroy();
    }

    @Override
    protected void onPause() {
        super.onPause();
        //grabber.stop();
    }

    @Override
    protected void onResume() {
        super.onResume();
        //grabber.start();
    }

    @Override
    protected void onStop() {
        super.onStop();
        loop.stop();
    }

    @Override
    protected void onStart() {
        super.onStart();
        loop.start();
    }
}
