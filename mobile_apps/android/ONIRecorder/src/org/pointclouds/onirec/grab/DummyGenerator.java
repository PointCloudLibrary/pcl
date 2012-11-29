package org.pointclouds.onirec.grab;

import org.OpenNI.MapOutputMode;

public abstract class DummyGenerator implements Generator {
    private static final MapOutputMode mode;

    static {
        mode = new MapOutputMode(640, 480, 30);
    }

    public abstract void refresh();

    @Override
    public MapOutputMode[] getSupportedModes() {
        return new MapOutputMode[] { mode };
    }

    @Override
    public MapOutputMode getMode() {
        return mode;
    }

    @Override
    public void setMode(MapOutputMode mode) {
    }

    @Override
    public void dispose() {
    }
}
