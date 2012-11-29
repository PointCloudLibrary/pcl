package com.itseez.onirec;

import org.OpenNI.*;

class LiveContextHolderFactory implements CaptureThreadManager.ContextHolderFactory {
    @Override
    public CaptureThreadManager.ContextHolder createHolder() throws GeneralException {
        return new CaptureThreadManager.ContextHolder() {
            private Context context;

            {
                context = new Context();
            }

            @Override
            public Context getContext() {
                return context;
            }

            @Override
            public ImageGenerator createImageGenerator() throws GeneralException {
                return ImageGenerator.create(context);
            }

            @Override
            public DepthGenerator createDepthGenerator() throws GeneralException {
                return DepthGenerator.create(context);
            }

            @Override
            public void dispose() {
                context.dispose();
            }
        };
    }
}
