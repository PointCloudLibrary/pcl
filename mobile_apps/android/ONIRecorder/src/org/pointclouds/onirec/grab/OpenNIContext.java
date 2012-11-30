package org.pointclouds.onirec.grab;

import org.OpenNI.*;

import java.io.File;

abstract class OpenNIContext implements Context {
    private org.OpenNI.Context context;
    private Recorder recorder;

    public OpenNIContext() throws GeneralException {
        context = new org.OpenNI.Context();
    }

    org.OpenNI.Context getRealContext() {
        return context;
    }

    @Override
    public void dispose() {
        if (recorder != null) recorder.dispose();
        context.dispose();
    }

    @Override
    public void startAll() throws StatusException {
        context.startGeneratingAll();
    }

    @Override
    public void stopAll() throws StatusException {
        context.stopGeneratingAll();
    }

    @Override
    public void waitAndUpdateAll() throws StatusException {
        context.waitAndUpdateAll();
    }

    @Override
    public void startRecording(File fileName) throws GeneralException {
        recorder = Recorder.create(context, "oni");

        try {
            recorder.setDestination(RecordMedium.FILE, fileName.getAbsolutePath());

            for (NodeInfo node_info: context.enumerateExistingNodes(NodeType.IMAGE))
                recorder.addNodeToRecording(node_info.getInstance());

            for (NodeInfo node_info: context.enumerateExistingNodes(NodeType.DEPTH))
                recorder.addNodeToRecording(node_info.getInstance());
        } catch (GeneralException e) {
            recorder.dispose();
            throw e;
        }
    }

    @Override
    public void stopRecording() {
        recorder.dispose();
    }
}
