package com.itseez.peopledemo;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class Main extends Activity {
    private ImageView picture;
    private static final String TAG = "PEOPLE.MAIN";

    private static byte[] readInputStream(InputStream is) throws IOException {
        byte[] buffer = new byte[1024 * 1024];
        ByteArrayOutputStream baos = new ByteArrayOutputStream();

        while (is.read(buffer) != -1)
            baos.write(buffer);

        return baos.toByteArray();
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        android.graphics.Point size = new android.graphics.Point();
        getWindowManager().getDefaultDisplay().getSize(size);

        Log.i(TAG, size.toString());

        picture = (ImageView) findViewById(R.id.picture);

        try {

            InputStream rgbd_in = getAssets().open("1.rgbd");

            picture.setImageBitmap(RGBDImage.parse(readInputStream(rgbd_in)).getColorsAsBitmap());

            rgbd_in.close();
        } catch (IOException ioe) {
            Log.e(TAG, ioe.getMessage());
        }
    }
}

