package com.itseez.onirec.grab;

import org.OpenNI.GeneralException;
import org.OpenNI.StatusException;

import java.io.File;

public interface Context {
    void startAll() throws StatusException;
    void stopAll() throws StatusException;
    void waitAndUpdateAll() throws StatusException;

    ColorGenerator createColorGenerator() throws GeneralException;

    DepthGenerator createDepthGenerator() throws GeneralException;

    void startRecording(File fileName) throws GeneralException;
    void stopRecording();

    void dispose();
}
