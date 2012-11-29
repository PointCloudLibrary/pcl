package com.itseez.onirec.grab;

import org.OpenNI.MapOutputMode;
import org.OpenNI.StatusException;

public interface Generator {
    MapOutputMode[] getSupportedModes() throws StatusException;
    MapOutputMode getMode() throws StatusException;
    void setMode(MapOutputMode mode) throws StatusException;
    void dispose();
}
