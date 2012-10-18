package com.itseez.peopledemo;

import android.graphics.Bitmap;

public class RGBDImage {
    private long ptr;

    private static native void cacheIds();

    @Override
    protected void finalize() throws Throwable {
        free();
        super.finalize();
    }

    public native void free();

    public native void readColors(int[] colors);

    public native int getHeight();

    public native int getWidth();

    public native static RGBDImage parse(byte[] bytes);

    static {
        System.loadLibrary("people_demo");
        cacheIds();
    }

    public Bitmap getColorsAsBitmap() {
        int w = getWidth(), h = getHeight();
        int[] colors = new int[w * h];
        readColors(colors);

        Bitmap bm = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        bm.setPixels(colors, 0, w, 0, 0, w, h);
        return bm;
    }

    private RGBDImage() { }
}
