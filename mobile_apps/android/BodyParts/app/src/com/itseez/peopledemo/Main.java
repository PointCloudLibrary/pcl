package com.itseez.peopledemo;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLContext;
import java.io.*;
import java.util.Arrays;

public class Main extends Activity {
    private static final String TAG = "PEOPLE.MAIN";

    private ImageView picture;
    private TextView timing_text;
    private BodyPartsRecognizer bpr;
    private final EGL10 egl = (EGL10) EGLContext.getEGL();
    private Grabber grabber = Grabber.createOpenNIGrabber();
    private RGBDImage img = new RGBDImage();

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

    private Bitmap labelsToBitmap(int width, int height, byte[] labels) {
        BodyPartLabel[] all_labels = BodyPartLabel.values();
        Bitmap bmp = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);

        int[] pixels = new int[labels.length];

        for (int i = 0; i < labels.length; ++i) {
            byte label = labels[i];
            pixels[i] = label >= 0 && label < all_labels.length ? all_labels[label].color : 0xFFFFFFFF;
        }

        bmp.setPixels(pixels, 0, width, 0, 0, width, height);
        return bmp;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        egl.eglInitialize(egl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY), null);

        picture = (ImageView) findViewById(R.id.picture);
        timing_text = (TextView) findViewById(R.id.timing_text);

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

        bpr = new BodyPartsRecognizer(trees);
        grabber.start();
        processImage();
    }

    @Override
    public void onDestroy() {
        grabber.stop();
        grabber.free();
        img.free();
        egl.eglTerminate(egl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY));
        super.onDestroy();
    }

    private void processImage() {
        long millis_before, millis_after, total_before, total_after;

        total_before = SystemClock.uptimeMillis();

        millis_before = SystemClock.uptimeMillis();

        grabber.getFrame(img);

        Bitmap image_bmp = img.getColorsAsBitmap();

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Reading image: %d ms", millis_after - millis_before));

        millis_before = SystemClock.uptimeMillis();

        byte[] labels_array = bpr.recognize(img);

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Recognition: %d ms", millis_after - millis_before));

        millis_before = SystemClock.uptimeMillis();

        Bitmap labels_bmp = labelsToBitmap(img.getWidth(), img.getHeight(), labels_array);

        Canvas canvas = new Canvas(image_bmp);
        canvas.drawBitmap(labels_bmp, 0, 0, null);

        picture.setImageBitmap(image_bmp);

        millis_after = SystemClock.uptimeMillis();

        Log.i(TAG, String.format("Drawing: %d ms", millis_after - millis_before));

        total_after = SystemClock.uptimeMillis();
        timing_text.setText(String.format("%d ms", total_after - total_before));
        Log.i(TAG, "Total: " + timing_text.getText());
    }

    public void pictureClicked(@SuppressWarnings("UnusedParameters") View view) {
        processImage();
    }
}

