package org.pointclouds.onirec;

import android.os.SystemClock;
import android.util.Log;

class Timer {
    public static void time(String description, Timeable code) throws ReturnException {
        long before = SystemClock.uptimeMillis();
        try {
            code.run();
        } finally {
            long after = SystemClock.uptimeMillis();
            Log.d("onirec.Timer", String.format("%s: %d", description, after - before));
        }
    }

    public interface Timeable {
        void run() throws ReturnException;
    }

    public static class ReturnException extends Exception {}
}
