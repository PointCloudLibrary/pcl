package com.itseez.peopledemo;

public class BodyPartsRecognizer {
    @SuppressWarnings("UnusedDeclaration")
    private long ptr;

    private static native void cacheIds();

    private native void create(byte[][] trees);

    @Override
    protected void finalize() throws Throwable {
        free();
        super.finalize();
    }

    public native void free();

    static {
        System.loadLibrary("bodyparts");
        cacheIds();
    }

    public BodyPartsRecognizer(byte[][] trees) {
        create(trees);
    }

    public native byte[] recognize(RGBDImage image);
}
