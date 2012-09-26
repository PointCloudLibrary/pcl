package com.itseez.onirec;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.view.SurfaceHolder;
import org.OpenNI.Context;
import org.OpenNI.GeneralException;
import org.OpenNI.ImageGenerator;
import org.OpenNI.StatusException;

import java.nio.ByteBuffer;

class CaptureThreadManager {
    static final String TAG = "onirec.CaptureThreadManager";
    final HandlerThread thread;
    final Handler handler;
    private final SurfaceHolder holderColor;
    private final SurfaceHolder holderDepth;
    private final Paint paint = new Paint();

    private Context context;
    private ImageGenerator color;
    //private DepthGenerator depth;

    private native static void imageBufferToBitmap(ByteBuffer buf, Bitmap bm);

    static {
        System.loadLibrary("onirec");
    }

    private Runnable processFrame = new Runnable() {
        @Override
        public void run() {
            long clock_before;
            long time_grabbing, time_reading, time_drawing;

            clock_before = SystemClock.uptimeMillis();
            try {
                context.waitAndUpdateAll();
            } catch (StatusException se) {
                Log.e(TAG, Log.getStackTraceString(se));
                // TODO: handle this
            }
            time_grabbing = SystemClock.uptimeMillis() - clock_before;

            clock_before = SystemClock.uptimeMillis();

            int frame_width = color.getMetaData().getXRes(), frame_height = color.getMetaData().getYRes();

            ByteBuffer color_buffer = color.getMetaData().getData().createByteBuffer();

            Bitmap color_bitmap = Bitmap.createBitmap(frame_width, frame_height, Bitmap.Config.ARGB_8888);

            imageBufferToBitmap(color_buffer, color_bitmap);

            time_reading = SystemClock.uptimeMillis() - clock_before;

            clock_before = SystemClock.uptimeMillis();
            Canvas canvas = holderColor.lockCanvas();

            if (canvas != null) {
                Rect canvas_size = holderColor.getSurfaceFrame();

                canvas.drawBitmap(color_bitmap, 0, 0, paint);

                holderColor.unlockCanvasAndPost(canvas);
            }

            time_drawing = SystemClock.uptimeMillis() - clock_before;
            Log.i(TAG, String.format("grabbing %d reading %d drawing %d", time_grabbing, time_reading, time_drawing));

/*
            ShortBuffer depth_buffer = depth.getMetaData().getData().createShortBuffer();

            for (int i = 0; i < num_pixels; ++i)
            {
                int gray = (int) (depth_buffer.get(i) / 32767. * 255.);
                color_array[i] = Color.rgb(gray, gray, gray);
            }

            canvas = holderDepth.lockCanvas();

            if (canvas != null) {
                Rect canvas_size = holderColor.getSurfaceFrame();

                canvas.drawBitmap(color_array, 0, frame_width, 0, 0, canvas_size.width(), canvas_size.height(),
                        false, paint);

                holderDepth.unlockCanvasAndPost(canvas);
            }
*/

            handler.post(processFrame);
        }
    };

    public CaptureThreadManager(SurfaceHolder holderColor, SurfaceHolder holderDepth) {
        this.holderColor = holderColor;
        this.holderDepth = holderDepth;
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
            //depth = DepthGenerator.create(context);

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

        //depth.dispose();
        color.dispose();

        context.dispose();
    }
}
