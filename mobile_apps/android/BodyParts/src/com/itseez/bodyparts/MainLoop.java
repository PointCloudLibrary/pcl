package com.itseez.bodyparts;

import android.graphics.Bitmap;
import android.os.*;
import android.util.Log;

import java.io.File;

public class MainLoop {
    private static final String TAG = "bodyparts.MainLoop";
    private HandlerThread thread;
    private Handler handler;
    private Grabber grabber;

    private Bitmap bmp = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
    private Cloud img = new Cloud();
    private BodyPartsRecognizer bpr;
    private File rgbdDir;
    private Feedback feedback;

    public interface Feedback {
        void initFinished(boolean success);
        void closeFinished();
        void grabberBroken();
        void newFrame(long timeMs, Bitmap frame);
    }

    private static void buildBitmap(Bitmap bmp, Cloud image, byte[] labels) {
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

    private long processImage() {
        long millis_before, millis_after, total_before, total_after;

        total_before = SystemClock.uptimeMillis();

        millis_before = SystemClock.uptimeMillis();

        grabber.getFrame(img);

        if (!grabber.isConnected())
            return -1;

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

    public MainLoop(BodyPartsRecognizer bpr, File rgbdDir, Feedback feedback) {
        this.feedback = feedback;
        this.bpr = bpr;
        this.rgbdDir = rgbdDir;

        thread = new HandlerThread("Grabber/Processor");
        thread.start();
        handler = new Handler(thread.getLooper());

        handler.post(new Runnable() {
            @Override
            public void run() {
                grabber = MainLoop.this.rgbdDir == null
                        ? Grabber.createOpenNIGrabber()
                        : Grabber.createFileGrabber(MainLoop.this.rgbdDir.getAbsolutePath());
                grabber.start();
                MainLoop.this.feedback.initFinished(grabber.isConnected());
            }
        });

    }

    public void close() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                grabber.stop();
                grabber.free();
                feedback.closeFinished();
            }
        });
    }

    public void destroy() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                Looper.myLooper().quit();
            }
        });

        try {
            thread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        img.free();
    }

    public void doOneFrame() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                final long total_ms = processImage();
                if (total_ms < 0)
                    feedback.grabberBroken();
                else
                    feedback.newFrame(total_ms, bmp);
            }
        });
    }

    public boolean isLive() {
        return rgbdDir == null;
    }
}
