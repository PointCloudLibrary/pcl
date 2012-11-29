package com.itseez.onirec.grab;

import org.OpenNI.DepthMetaData;

public class OpenNIDepthGenerator extends OpenNIGenerator implements DepthGenerator {
    private final org.OpenNI.DepthGenerator wrapped;

    public OpenNIDepthGenerator(org.OpenNI.DepthGenerator wrapped) {
        super(wrapped);
        this.wrapped = wrapped;
    }

    @Override
    public DepthMetaData getMetaData() {
        return wrapped.getMetaData();
    }
}
