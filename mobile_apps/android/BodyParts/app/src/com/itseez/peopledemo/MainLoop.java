package com.itseez.peopledemo;

import android.graphics.Bitmap;
import android.os.*;
import android.util.Log;

public class MainLoop {
    private static final String TAG = "MainLoop";
    private Main main;
    private HandlerThread thread;
    private Handler handler;
    private IdleHandler idleHandler = new IdleHandler();
    private Grabber grabber;
    private boolean running = false;

    Bitmap bmp = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
    RGBDImage img = new RGBDImage();
    BodyPartsRecognizer bpr;

    private static void buildBitmap(Bitmap bmp, RGBDImage image, byte[] labels) {
        BodyPartLabel[] all_labels = BodyPartLabel.values();

        int[] pixels = new int[labels.length];
        image.readColors(pixels);

        for (int i = 0; i < labels.length; ++i) {
            byte label = labels[i];
            if (label != BodyPartLabel.BACKGROUND.ordinal())
                pixels[i] = label >= 0 && label < all_labels.length ? all_labels[label].color : 0xFFFFFFFF;
        }

        bmp.setPixels(pixels, 0, image.getWidth(), 0, 0, image.getWidth(), image.getHeight());
    }

    private long processImage()
    {
        long millis_before, millis_after, total_before, total_after;

        total_before = SystemClock.uptimeMillis();

        millis_before = SystemClock.uptimeMillis();

        grabber.getFrame(img);

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Reading image: %d ms", millis_after - millis_before));

        millis_before = SystemClock.uptimeMillis();

        byte[] labels_array = bpr.recognize(img);

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Recognition: %d ms", millis_after - millis_before));

        millis_before = SystemClock.uptimeMillis();

        if (bmp.getWidth() != img.getWidth() || bmp.getHeight() != img.getHeight())
            bmp = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.ARGB_8888);
        buildBitmap(bmp, img, labels_array);

        millis_after = SystemClock.uptimeMillis();
        Log.i(TAG, String.format("Creating image: %d ms", millis_after - millis_before));

        total_after = SystemClock.uptimeMillis();

        return total_after - total_before;
    }

    class IdleHandler implements MessageQueue.IdleHandler {
        @Override
        public boolean queueIdle() {
            handler.post(new Runnable() {
                @Override
                public void run() {
                    if (!running) return;

                    final long total_ms = processImage();

                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.picture.setImageBitmap(bmp);

                            main.timingText.setText(String.format("%d ms", total_ms));
                            Log.i(TAG, "Total: " + main.timingText.getText());
                        }
                    });
                }
            });
            return true;
        }
    }

    public MainLoop(Main main, BodyPartsRecognizer bpr) {
        this.main = main;
        this.bpr = bpr;
        //this.grabber = Grabber.createFileGrabber("/mnt/sdcard2/rgbd");
        this.grabber = Grabber.createOpenNIGrabber();
        thread = new HandlerThread("Grabber/Processor");
        thread.start();
        handler = new Handler(thread.getLooper());
    }

    public void start() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                running = true;
                grabber.start();
                Looper.myQueue().addIdleHandler(idleHandler);
            }
        });
    }

    public void stop() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                running = false;
                grabber.stop();
                Looper.myQueue().removeIdleHandler(idleHandler);
            }
        });
    }

    public void destroy() {
        thread.quit();
        try {
            thread.join();
        } catch (InterruptedException e) {

        }
        grabber.free();
        img.free();
    }
}
