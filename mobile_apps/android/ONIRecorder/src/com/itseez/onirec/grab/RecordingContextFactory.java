package com.itseez.onirec.grab;

import org.OpenNI.*;

import java.io.File;

public class RecordingContextFactory implements ContextFactory {
    private final File recording;

    public RecordingContextFactory(File recording) {
        this.recording = recording;
    }

    @Override
    public Context createHolder() throws GeneralException {
        return new OpenNIContext() {
            private Player player;

            {
                try {
                    player = getRealContext().openFileRecordingEx(recording.getAbsolutePath());
                } catch (GeneralException e) {
                    dispose();
                    throw e;
                }
            }

            @Override
            public ColorGenerator createColorGenerator() throws GeneralException {
                return new OpenNIColorGenerator(
                        (ImageGenerator) getRealContext().findExistingNode(NodeType.IMAGE));
            }

            @Override
            public DepthGenerator createDepthGenerator() throws GeneralException {
                return new OpenNIDepthGenerator(
                        (org.OpenNI.DepthGenerator) getRealContext().findExistingNode(NodeType.DEPTH));
            }

            @Override
            public void dispose() {
                if (player != null) player.dispose();
            }
        };
    }
}
