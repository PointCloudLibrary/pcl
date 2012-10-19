package com.itseez.bodyparts;

import android.app.Activity;
import android.graphics.Bitmap;

class ProxyFeedback implements MainLoop.Feedback {
    Activity activity;
    MainLoop.Feedback underlying;

    ProxyFeedback(Activity activity, MainLoop.Feedback underlying) {
        this.activity = activity;
        this.underlying = underlying;
    }

    public void replaceUnderlying(MainLoop.Feedback newUnderlying) {
        underlying = newUnderlying;
    }

    @Override
    public void initFinished(final boolean success) {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                underlying.initFinished(success);
            }
        });
    }

    @Override
    public void closeFinished() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                underlying.closeFinished();
            }
        });
    }

    @Override
    public void grabberBroken() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                underlying.grabberBroken();
            }
        });
    }

    @Override
    public void newFrame(final long timeMs, final Bitmap frame) {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                underlying.newFrame(timeMs, frame);
            }
        });
    }
}
