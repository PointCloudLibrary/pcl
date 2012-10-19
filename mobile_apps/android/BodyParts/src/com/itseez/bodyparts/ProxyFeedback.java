package com.itseez.bodyparts;

import android.app.Activity;

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
}
