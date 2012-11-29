package org.pointclouds.onirec.grab;

import org.OpenNI.GeneralException;

public interface ContextFactory {
    Context createContext() throws GeneralException;
}
