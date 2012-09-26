package com.itseez.onirec;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.os.*;
import android.util.Log;
import android.view.SurfaceHolder;
import org.OpenNI.*;

import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

class CaptureThreadManager {
    public interface Feedback {
        void setFps(double fps);
    }

    static final String TAG = "onirec.CaptureThreadManager";
    private final HandlerThread thread;
    private final Handler handler;
    private final Handler uiHandler = new Handler();
    private final SurfaceHolder holderColor;
    private final SurfaceHolder holderDepth;
    private Feedback feedback;

    private Context context;
    private ImageGenerator color;
    private DepthGenerator depth;

    private Bitmap colorBitmap;
    private Bitmap depthBitmap;

    {
        colorBitmap = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
        colorBitmap.setHasAlpha(false);
        depthBitmap = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
        depthBitmap.setHasAlpha(false);
    }

    int frameCount = 0;
    long lastUpdateTime = SystemClock.uptimeMillis();

    private native static void imageBufferToBitmap(ByteBuffer buf, Bitmap bm);
    private native static void depthBufferToBitmap(ShortBuffer buf, Bitmap bm, int maxZ);

    static {
        System.loadLibrary("onirec");
    }

    private Runnable processFrame = new Runnable() {
        @Override
        public void run() {
            try {
                context.waitAndUpdateAll();
            } catch (StatusException se) {
                Log.e(TAG, Log.getStackTraceString(se));
                // TODO: handle this
            }

            int frame_width = color.getMetaData().getXRes();
            int frame_height = color.getMetaData().getYRes();

            if (colorBitmap.getWidth() != frame_width || colorBitmap.getHeight() != frame_height) {
                colorBitmap.recycle();
                colorBitmap = Bitmap.createBitmap(frame_width, frame_height, Bitmap.Config.ARGB_8888);
                colorBitmap.setHasAlpha(false);
            }

            imageBufferToBitmap(color.getMetaData().getData().createByteBuffer(), colorBitmap);

            Canvas canvas = holderColor.lockCanvas();

            if (canvas != null) {
                Rect canvas_size = holderColor.getSurfaceFrame();
                Rect bm_size = new Rect(0, 0, frame_width, frame_height);

                canvas.drawBitmap(colorBitmap, bm_size, canvas_size, null);

                holderColor.unlockCanvasAndPost(canvas);
            }

            frame_width = depth.getMetaData().getXRes();
            frame_height = depth.getMetaData().getYRes();

            if (depthBitmap.getWidth() != frame_width || depthBitmap.getHeight() != frame_height) {
                depthBitmap.recycle();
                depthBitmap = Bitmap.createBitmap(frame_width, frame_height, Bitmap.Config.ARGB_8888);
                depthBitmap.setHasAlpha(false);
            }

            long time_before = SystemClock.uptimeMillis();
            depthBufferToBitmap(depth.getMetaData().getData().createShortBuffer(), depthBitmap,
                    depth.getMetaData().getZRes() - 1);
            Log.i(TAG, String.format("time: %d", SystemClock.uptimeMillis() - time_before));

            canvas = holderDepth.lockCanvas();

            if (canvas != null) {
                Rect canvas_size = holderDepth.getSurfaceFrame();
                Rect bm_size = new Rect(0, 0, frame_width, frame_height);

                canvas.drawBitmap(depthBitmap, bm_size, canvas_size, null);

                holderDepth.unlockCanvasAndPost(canvas);
            }

            ++frameCount;
            final long new_time = SystemClock.uptimeMillis();
            if (new_time >= lastUpdateTime + 500) {
                final double fps = frameCount / (double) (new_time - lastUpdateTime) * 1000;

                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        feedback.setFps(fps);
                    }
                });

                frameCount = 0;
                lastUpdateTime = new_time;
            }

            handler.post(processFrame);
        }
    };

    public CaptureThreadManager(SurfaceHolder holderColor, SurfaceHolder holderDepth, Feedback feedback) {
        this.holderColor = holderColor;
        this.holderDepth = holderDepth;
        this.feedback = feedback;

        thread = new HandlerThread("Capture Thread");
        thread.start();

        handler = new Handler(thread.getLooper());

        handler.post(new Runnable() {
            @Override
            public void run() {
                initOpenNI();
                handler.post(processFrame);
            }
        });
    }

    public void stop() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                handler.removeCallbacks(processFrame);
                terminateOpenNI();
                Looper.myLooper().quit();
            }
        });

        try {
            thread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void initOpenNI() {
        try {
            context = new Context();

            color = ImageGenerator.create(context);
            depth = DepthGenerator.create(context);

            context.startGeneratingAll();
        } catch (GeneralException ge) {
            Log.e(TAG, Log.getStackTraceString(ge));
            // TODO: handle this
        }
    }

    private void terminateOpenNI() {
        try {
            context.stopGeneratingAll();
        } catch (StatusException e) {
            Log.e(TAG, "OpenNI context failed to stop generating.");
        }

        depth.dispose();
        color.dispose();

        context.dispose();
    }
}
