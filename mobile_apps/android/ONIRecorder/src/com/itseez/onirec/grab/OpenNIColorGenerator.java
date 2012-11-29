package com.itseez.onirec.grab;

import org.OpenNI.ImageGenerator;
import org.OpenNI.ImageMetaData;

public class OpenNIColorGenerator extends OpenNIGenerator implements ColorGenerator {
    private final ImageGenerator wrapped;

    public OpenNIColorGenerator(ImageGenerator wrapped) {
        super(wrapped);
        this.wrapped = wrapped;
    }

    @Override
    public ImageMetaData getMetaData() {
        return wrapped.getMetaData();
    }
}
