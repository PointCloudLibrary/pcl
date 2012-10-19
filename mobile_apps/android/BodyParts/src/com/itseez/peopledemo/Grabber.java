package com.itseez.peopledemo;

public class Grabber {
    @SuppressWarnings("UnusedDeclaration")
    private long ptr;

    private static native void cacheIds();

    @Override
    protected void finalize() throws Throwable {
        free();
        super.finalize();
    }

    static {
        System.loadLibrary("bodyparts");
        cacheIds();
    }

    private Grabber() {}

    public native void free();

    public static native Grabber createOpenNIGrabber();
    public static native Grabber createFileGrabber(String directory);

    public native boolean isConnected();

    public native void getFrame(RGBDImage frame);

    public native void start();
    public native void stop();
}
