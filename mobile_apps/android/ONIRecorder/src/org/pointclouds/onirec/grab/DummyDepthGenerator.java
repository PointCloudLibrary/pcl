package org.pointclouds.onirec.grab;

import org.OpenNI.DepthMetaData;
import org.OpenNI.MapOutputMode;

import java.nio.ByteBuffer;

public class DummyDepthGenerator extends DummyGenerator implements DepthGenerator {
    private final ByteBuffer buffer;
    private final DepthMetaData metaData;

    {
        MapOutputMode mode = getMode();
        buffer = ByteBuffer.allocateDirect(mode.getXRes() * mode.getYRes() * 2);

        metaData = new DepthMetaData();
        metaData.setXRes(mode.getXRes());
        metaData.setYRes(mode.getYRes());
        metaData.setZRes(Short.MAX_VALUE);
        metaData.setFPS(mode.getFPS());
        metaData.setDataPtr(NativeBuffer.getPtr(buffer));
    }

    @Override
    public DepthMetaData getMetaData() {
        return metaData;
    }

    @Override
    public void refresh() {
        NativeBuffer.fillBuffer(buffer);
    }
}
