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

    private int imgNum = 5;
    private ImageView picture;
    private TextView timing_text;
    private File[] imageFiles;
    private BodyPartsRecognizer bpr;
    private EGL10 egl = (EGL10) EGLContext.getEGL();

    private static byte[] readInputStream(InputStream is) throws IOException {
        byte[] buffer = new byte[1024 * 1024];
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        int bytes_read;

        while ((bytes_read = is.read(buffer)) != -1)
            baos.write(buffer, 0, bytes_read);

        return baos.toByteArray();
    }

    private Bitmap labelsToBitmap(int width, int height, byte[] labels) {
        BodyPartLabel[] all_labels = BodyPartLabel.values();
        Bitmap bmp = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);

        int[] pixels = new int[labels.length];

        for (int i = 0; i < labels.length; ++i)
            pixels[i] = labels[i] >= 0 && labels[i] < all_labels.length ? all_labels[labels[i]].color : 0xFFFFFFFF;


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

        imageFiles = new File("/mnt/sdcard2/rgbd").listFiles();
        Arrays.sort(imageFiles);

        ListView color_ref = (ListView) findViewById(R.id.color_list);
        color_ref.setAdapter(new BodyPartLabelAdapter(this, android.R.layout.simple_list_item_1, BodyPartLabel.values(),
                (LayoutInflater) getSystemService(LAYOUT_INFLATER_SERVICE)));

        File[] treeFiles = new File("/mnt/sdcard2/trees").listFiles();
        Arrays.sort(treeFiles);
        byte[][] trees = new byte[treeFiles.length][];

        for (int ti = 0; ti < trees.length; ++ti) {
            try {
                FileInputStream tree_stream = new FileInputStream(treeFiles[ti]);
                try {
                    trees[ti] = readInputStream(tree_stream);
                }
                finally {
                    tree_stream.close();
                }
            }
            catch (IOException ioe) {
                Log.e(TAG, ioe.getMessage(), ioe);
                return;
            }
        }

        bpr = new BodyPartsRecognizer(trees);

        processImage();
    }

    @Override
    public void onDestroy() {
        egl.eglTerminate(egl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY));
        super.onDestroy();
    }

    private void processImage() {
        long millis_before, millis_after, total_before, total_after;
        RGBDImage img;

        total_before = SystemClock.uptimeMillis();

        millis_before = SystemClock.uptimeMillis();

        try {
            InputStream rgbd_in = new FileInputStream(imageFiles[imgNum]);
            try {
                img = RGBDImage.parse(readInputStream(rgbd_in));
            } finally {
                rgbd_in.close();
            }
        } catch (IOException ioe) {
            Log.e(TAG, ioe.getMessage(), ioe);
            return;
        }

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
        img.free();

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Drawing: %d ms", millis_after - millis_before));

        total_after = SystemClock.uptimeMillis();
        timing_text.setText(String.format("%d ms", total_after - total_before));
        Log.i(TAG, "Total: " + timing_text.getText());
    }

    public void pictureClicked(View view) {
        ++imgNum;
        if (imgNum >= imageFiles.length)
            imgNum = 0;

        processImage();
    }
}

