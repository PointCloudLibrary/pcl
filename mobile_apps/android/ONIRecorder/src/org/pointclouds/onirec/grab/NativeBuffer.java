package org.pointclouds.onirec.grab;

import java.nio.Buffer;

class NativeBuffer {
    native static long getPtr(Buffer buffer);
    native static long fillBuffer(Buffer buffer);
}
