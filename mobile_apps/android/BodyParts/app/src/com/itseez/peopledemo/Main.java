package com.itseez.peopledemo;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class Main extends Activity {
    private static final String TAG = "PEOPLE.MAIN";

    private static byte[] readInputStream(InputStream is) throws IOException {
        byte[] buffer = new byte[1024 * 1024];
        ByteArrayOutputStream baos = new ByteArrayOutputStream();

        while (is.read(buffer) != -1)
            baos.write(buffer);

        return baos.toByteArray();
    }

    private Bitmap labelsToBitmap(int width, int height, byte[] labels)
    {
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
        RGBDImage img;

        ImageView picture = (ImageView) findViewById(R.id.picture);

        try {
            InputStream rgbd_in = getAssets().open("1.rgbd");
            img = RGBDImage.parse(readInputStream(rgbd_in));
            rgbd_in.close();
        } catch (IOException ioe) {
            Log.e(TAG, ioe.getMessage());
            return;
        }

        Bitmap image_bmp = img.getColorsAsBitmap();
        BodyPartsRecognizer bpr = new BodyPartsRecognizer();
        Bitmap labels_bmp = labelsToBitmap(img.getWidth(), img.getHeight(), bpr.recognize(img));

        Canvas canvas = new Canvas(image_bmp);
        canvas.drawBitmap(labels_bmp, 0, 0, null);

        picture.setImageBitmap(image_bmp);
    }
}

