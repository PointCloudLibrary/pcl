package com.itseez.peopledemo;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;

import java.io.*;
import java.util.Arrays;

public class Main extends Activity {
    private static final String TAG = "PEOPLE.MAIN";

    private int imgNum = 0;
    private ImageView picture;
    private File[] imageFiles;
    private BodyPartsRecognizer bpr;

    private static byte[] readInputStream(InputStream is) throws IOException {
        byte[] buffer = new byte[1024 * 1024];
        ByteArrayOutputStream baos = new ByteArrayOutputStream();

        while (is.read(buffer) != -1)
            baos.write(buffer);

        return baos.toByteArray();
    }

    private Bitmap labelsToBitmap(int width, int height, byte[] labels) {
        Bitmap bmp = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);

        int[] pixels = new int[labels.length];

        for (int i = 0; i < labels.length; ++i)
            pixels[i] = labels[i] > 0 ? 0xFF00FF00 : 0x00000000;

        bmp.setPixels(pixels, 0, width, 0, 0, width, height);
        return bmp;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        picture = (ImageView) findViewById(R.id.picture);
        imageFiles = new File("/mnt/sdcard2/rgbd").listFiles();
        Arrays.sort(imageFiles);

        File[] treeFiles = new File("/mnt/sdcard2/trees").listFiles();
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

        showImage();
    }

    private void showImage() {
        RGBDImage img;

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
        Bitmap labels_bmp = labelsToBitmap(img.getWidth(), img.getHeight(), bpr.recognize(img));

        Canvas canvas = new Canvas(image_bmp);
        canvas.drawBitmap(labels_bmp, 0, 0, null);

        picture.setImageBitmap(image_bmp);
        img.free();
    }

    public void pictureClicked(View view) {
        ++imgNum;
        if (imgNum >= imageFiles.length)
            imgNum = 0;

        showImage();
    }
}

