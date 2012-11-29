package com.itseez.onirec.grab;

import org.OpenNI.ImageMetaData;
import org.OpenNI.MapOutputMode;

import java.nio.ByteBuffer;

public class DummyColorGenerator extends DummyGenerator implements ColorGenerator {
    private final ByteBuffer buffer;
    private final ImageMetaData metaData;

    {
        MapOutputMode mode = getMode();
        buffer = ByteBuffer.allocateDirect(mode.getXRes() * mode.getYRes() * 3);

        metaData = new ImageMetaData();
        metaData.setXRes(mode.getXRes());
        metaData.setYRes(mode.getYRes());
        metaData.setFPS(mode.getFPS());
        metaData.setDataPtr(NativeBuffer.getPtr(buffer));
    }

    @Override
    public ImageMetaData getMetaData() {
        return metaData;
    }

    @Override
    public void refresh() {
        NativeBuffer.fillBuffer(buffer);
    }
}
