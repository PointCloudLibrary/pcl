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
import org.OpenNI.*;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

class CaptureThreadManager {
    public interface Feedback {
        public enum Error {FailedToStartCapture, FailedDuringCapture, FailedToStartRecording}

        void setFps(double fps);

        void reportError(Error error, String oniMessage);

        void reportRecordingFinished();

        void reportCaptureStarted(MapOutputMode[] colorModes, MapOutputMode currentColorMode,
                                  MapOutputMode[] depthModes, MapOutputMode currentDepthMode);
    }

    public interface ContextHolder {
        Context getContext();

        ImageGenerator createImageGenerator() throws GeneralException;

        DepthGenerator createDepthGenerator() throws GeneralException;

        void dispose();
    }

    public interface ContextHolderFactory {
        ContextHolder createHolder() throws GeneralException;
    }

    private static final String TAG = "onirec.CaptureThreadManager";
    private final HandlerThread thread;
    private final Handler handler;
    private final Handler uiHandler = new Handler();
    private final SurfaceHolder holderColor;
    private final SurfaceHolder holderDepth;
    private final Feedback feedback;

    private ContextHolder contextHolder;
    private ImageGenerator color;
    private DepthGenerator depth;
    private Recorder recorder;

    private Bitmap colorBitmap;
    private Bitmap depthBitmap;

    private boolean hasError = false;

    {
        colorBitmap = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
        colorBitmap.setHasAlpha(false);
        depthBitmap = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
        depthBitmap.setHasAlpha(false);
    }

    private int frameCount = 0;
    private long lastUpdateTime = SystemClock.uptimeMillis();

    private native static void imageBufferToBitmap(ByteBuffer buf, Bitmap bm);

    private native static void depthBufferToBitmap(ShortBuffer buf, Bitmap bm, int maxZ);

    static {
        System.loadLibrary("onirec");
    }

    private void reportError(final Feedback.Error error, final String oniMessage) {
        hasError = true;
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                feedback.reportError(error, oniMessage);
            }
        });
    }

    private final Runnable processFrame = new Runnable() {
        @Override
        public void run() {
            try {
                contextHolder.getContext().waitAndUpdateAll();
            } catch (final StatusException se) {
                final String message = "Failed to acquire a frame.";
                Log.e(TAG, message, se);
                reportError(Feedback.Error.FailedDuringCapture, se.getMessage());
                return;
            }

            if (color != null) {
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

                    canvas.drawBitmap(colorBitmap, null, canvas_size, null);

                    holderColor.unlockCanvasAndPost(canvas);
                }
            }

            if (depth != null) {
                int frame_width = depth.getMetaData().getXRes();
                int frame_height = depth.getMetaData().getYRes();

                if (depthBitmap.getWidth() != frame_width || depthBitmap.getHeight() != frame_height) {
                    depthBitmap.recycle();
                    depthBitmap = Bitmap.createBitmap(frame_width, frame_height, Bitmap.Config.ARGB_8888);
                    depthBitmap.setHasAlpha(false);
                }

                depthBufferToBitmap(depth.getMetaData().getData().createShortBuffer(), depthBitmap,
                        depth.getMetaData().getZRes() - 1);

                Canvas canvas = holderDepth.lockCanvas();

                if (canvas != null) {
                    Rect canvas_size = holderDepth.getSurfaceFrame();

                    canvas.drawBitmap(depthBitmap, null, canvas_size, new Paint(Paint.DITHER_FLAG));

                    holderDepth.unlockCanvasAndPost(canvas);
                }
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

    public CaptureThreadManager(SurfaceHolder holderColor, SurfaceHolder holderDepth, Feedback feedback,
                                final ContextHolderFactory contextHolderFactory) {
        this.holderColor = holderColor;
        this.holderDepth = holderDepth;
        this.feedback = feedback;

        thread = new HandlerThread("Capture Thread");
        thread.start();

        handler = new Handler(thread.getLooper());

        handler.post(new Runnable() {
            @Override
            public void run() {
                initOpenNI(contextHolderFactory);
            }
        });
    }

    public void stop() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                handler.removeCallbacks(processFrame);
                terminateOpenNI();
                uiHandler.removeCallbacksAndMessages(null);
                Looper.myLooper().quit();
            }
        });

        try {
            thread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void startRecording(final File file) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                try {
                    recorder = Recorder.create(contextHolder.getContext(), "oni");
                    recorder.setDestination(RecordMedium.FILE, file.getAbsolutePath());
                    if (color != null) recorder.addNodeToRecording(color);
                    recorder.addNodeToRecording(depth);
                } catch (final GeneralException ge) {
                    if (recorder != null) recorder.dispose();
                    final String message = "Failed to start recording.";
                    Log.e(TAG, message, ge);
                    reportError(Feedback.Error.FailedToStartRecording, ge.getMessage());
                }
            }
        });
    }

    public void stopRecording() {
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (recorder != null) {
                    recorder.dispose();
                    recorder = null;
                    uiHandler.post(new Runnable() {
                        @Override
                        public void run() {
                            feedback.reportRecordingFinished();
                        }
                    });
                }
            }
        });
    }

    public void setColorMode(final MapOutputMode mode) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (mode == null && color == null) return;

                try {
                    if (depth != null || color != null) {
                        contextHolder.getContext().stopGeneratingAll();
                        handler.removeCallbacks(processFrame);
                    }

                    if (mode == null) {
                        color.dispose();
                        color = null;
                    } else if (color == null) {
                        color = contextHolder.createImageGenerator();
                        color.setMapOutputMode(mode);
                    } else {
                        color.setMapOutputMode(mode);
                    }

                    if (depth != null || color != null) {
                        contextHolder.getContext().startGeneratingAll();
                        handler.post(processFrame);
                    }
                } catch (GeneralException ge) {
                    Log.e(TAG, "Failed to switch mode.", ge);
                    reportError(Feedback.Error.FailedDuringCapture, ge.getMessage());
                }
            }
        });
    }

    public void setDepthMode(final MapOutputMode mode) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (mode == null && depth == null) return;

                try {
                    if (depth != null || color != null) {
                        contextHolder.getContext().stopGeneratingAll();
                        handler.removeCallbacks(processFrame);
                    }

                    if (mode == null) {
                        depth.dispose();
                        depth = null;
                    } else if (depth == null) {
                        depth = contextHolder.createDepthGenerator();
                        depth.setMapOutputMode(mode);
                    } else {
                        depth.setMapOutputMode(mode);
                    }

                    if (depth != null || color != null) {
                        contextHolder.getContext().startGeneratingAll();
                        handler.post(processFrame);
                    }
                } catch (GeneralException ge) {
                    Log.e(TAG, "Failed to switch mode.", ge);
                    reportError(Feedback.Error.FailedDuringCapture, ge.getMessage());
                }
            }
        });
    }

    private void reportCaptureStart() throws StatusException {
        final MapOutputMode[] color_modes = color == null ? null : color.getSupportedMapOutputModes();
        final MapOutputMode color_current_mode = color == null ? null : color.getMapOutputMode();
        final MapOutputMode[] depth_modes = depth == null ? null : depth.getSupportedMapOutputModes();
        final MapOutputMode depth_current_mode = depth == null ? null : depth.getMapOutputMode();

        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                feedback.reportCaptureStarted(color_modes, color_current_mode, depth_modes, depth_current_mode);
            }
        });
    }

    private void initOpenNI(ContextHolderFactory factory) {
        try {
            contextHolder = factory.createHolder();

            try {
                color = contextHolder.createImageGenerator();
            } catch (StatusException ste) {
                // There is no color output. Or there's an error, but we can't distinguish between the two cases.
            }

            try {
                depth = contextHolder.createDepthGenerator();
            } catch (StatusException ste) {
                // If there's no depth or color, let's bail.
                if (color == null) throw ste;
            }

            contextHolder.getContext().startGeneratingAll();
            reportCaptureStart();
        } catch (final GeneralException ge) {
            final String message = "Failed to initialize OpenNI.";
            Log.e(TAG, message, ge);
            reportError(Feedback.Error.FailedToStartCapture, ge.getMessage());
            return;
        }

        handler.post(processFrame);
    }

    private void terminateOpenNI() {
        if (contextHolder != null)
            try {
                contextHolder.getContext().stopGeneratingAll();
            } catch (StatusException e) {
                Log.e(TAG, "OpenNI context failed to stop generating.", e);
            }

        if (recorder != null) recorder.dispose();
        if (depth != null) depth.dispose();
        if (color != null) color.dispose();
        if (contextHolder != null) contextHolder.dispose();
    }

    public boolean hasError() {
        return hasError;
    }
}
