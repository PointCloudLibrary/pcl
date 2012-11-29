package com.itseez.onirec.grab;

import org.OpenNI.GeneralException;

public interface ContextFactory {
    Context createHolder() throws GeneralException;
}
