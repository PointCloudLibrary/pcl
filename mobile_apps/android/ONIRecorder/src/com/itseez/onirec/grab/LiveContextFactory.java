package com.itseez.onirec.grab;

import org.OpenNI.GeneralException;
import org.OpenNI.ImageGenerator;

public class LiveContextFactory implements ContextFactory {
    @Override
    public Context createContext() throws GeneralException {
        return new OpenNIContext() {
            @Override
            public ColorGenerator createColorGenerator() throws GeneralException {
                return new OpenNIColorGenerator(ImageGenerator.create(getRealContext()));
            }

            @Override
            public DepthGenerator createDepthGenerator() throws GeneralException {
                return new OpenNIDepthGenerator(org.OpenNI.DepthGenerator.create(getRealContext()));
            }
        };
    }
}
