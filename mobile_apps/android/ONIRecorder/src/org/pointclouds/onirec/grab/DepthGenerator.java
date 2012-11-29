package org.pointclouds.onirec.grab;

import org.OpenNI.DepthMetaData;

public interface DepthGenerator extends Generator {
    DepthMetaData getMetaData();
}
