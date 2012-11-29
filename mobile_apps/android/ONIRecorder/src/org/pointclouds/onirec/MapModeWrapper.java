package org.pointclouds.onirec;

import android.content.Context;
import org.OpenNI.MapOutputMode;

class MapModeWrapper {
    private final MapOutputMode mode;
    private final String modeStr;

    public MapModeWrapper(Context context, MapOutputMode mode) {
        this.mode = mode;
        this.modeStr = mode == null
                ? context.getResources().getString(R.string.mode_disabled)
                : context.getResources().getString(R.string.mode_format,
                    mode.getXRes(), mode.getYRes(), mode.getFPS());
    }

    @Override
    public String toString() {
        return modeStr;
    }

    public MapOutputMode getMode() {
        return mode;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }

        if (!(o instanceof MapModeWrapper)) {
            return false;
        }

        MapModeWrapper that = (MapModeWrapper) o;
        MapOutputMode m1 = this.mode, m2 = that.mode;

        if (m1 == null || m2 == null)
            return m1 == m2;

        return m1.getFPS() == m2.getFPS() && m1.getXRes() == m2.getXRes() && m1.getYRes() == m2.getYRes();
    }

    @Override
    public int hashCode() {
        int mode_hash = 0;

        if (mode != null) {
            mode_hash = 17;
            mode_hash = 31 * mode_hash + mode.getFPS();
            mode_hash = 31 * mode_hash + mode.getXRes();
            mode_hash = 31 * mode_hash + mode.getYRes();
        }

        return 31 * 17 + mode_hash;
    }
}
