package org.pointclouds.onirec.grab;

import org.OpenNI.MapGenerator;
import org.OpenNI.MapOutputMode;
import org.OpenNI.StatusException;

class OpenNIGenerator implements Generator {
    private final MapGenerator wrapped;

    OpenNIGenerator(MapGenerator wrapped) {
        this.wrapped = wrapped;
    }

    @Override
    public MapOutputMode[] getSupportedModes() throws StatusException {
        return wrapped.getSupportedMapOutputModes();
    }

    @Override
    public MapOutputMode getMode() throws StatusException {
        return wrapped.getMapOutputMode();
    }

    @Override
    public void setMode(MapOutputMode mode) throws StatusException {
        wrapped.setMapOutputMode(mode);
    }

    @Override
    public void dispose() {
        wrapped.dispose();
    }
}
