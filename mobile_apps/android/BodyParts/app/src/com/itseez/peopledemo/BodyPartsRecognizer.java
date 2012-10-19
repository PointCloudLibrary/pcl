package com.itseez.peopledemo;

public class BodyPartsRecognizer {
    @SuppressWarnings("UnusedDeclaration")
    private long ptr;

    private static native void cacheIds();

    private native void create();

    @Override
    protected void finalize() throws Throwable {
        free();
        super.finalize();
    }

    public native void free();

    static {
        System.loadLibrary("people_demo");
        cacheIds();
    }

    public BodyPartsRecognizer() {
        create();
    }

    public native byte[] recognize(RGBDImage image);
}
