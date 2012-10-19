package com.itseez.bodyparts;

public class Cloud {
    private long ptr;

    private static native void cacheIds();

    @Override
    protected void finalize() throws Throwable {
        free();
        super.finalize();
    }

    @SuppressWarnings("UnusedDeclaration")
    private Cloud(long ptr)
    {
        this.ptr = ptr;
    }

    private native void create();
    public native void free();

    public native void readColors(int[] colors);

    public native int getHeight();

    public native int getWidth();

    static {
        System.loadLibrary("bodyparts");
        cacheIds();
    }

    public Cloud()
    {
        create();
    }
}
