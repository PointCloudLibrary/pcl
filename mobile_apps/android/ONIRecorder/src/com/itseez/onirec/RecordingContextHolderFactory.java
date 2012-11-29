package com.itseez.onirec;

import org.OpenNI.*;

import java.io.File;

class RecordingContextHolderFactory implements CaptureThreadManager.ContextHolderFactory {
    private final File recording;

    public RecordingContextHolderFactory(File recording) {
        this.recording = recording;
    }

    @Override
    public CaptureThreadManager.ContextHolder createHolder() throws GeneralException {
        return new CaptureThreadManager.ContextHolder() {
            private Context context;
            private Player player;

            {
                try {
                    context = new Context();
                    player = context.openFileRecordingEx(recording.getAbsolutePath());
                } catch (GeneralException ge) {
                    if (context != null) context.dispose();
                    throw ge;
                }
            }

            @Override
            public Context getContext() {
                return context;
            }

            @Override
            public ImageGenerator createImageGenerator() throws GeneralException {
                return (ImageGenerator) context.findExistingNode(NodeType.IMAGE);
            }

            @Override
            public DepthGenerator createDepthGenerator() throws GeneralException {
                return (DepthGenerator) context.findExistingNode(NodeType.DEPTH);
            }

            @Override
            public void dispose() {
                player.dispose();
                context.dispose();
            }
        };
    }
}
