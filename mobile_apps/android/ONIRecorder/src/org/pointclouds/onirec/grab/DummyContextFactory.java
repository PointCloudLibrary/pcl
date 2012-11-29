package org.pointclouds.onirec.grab;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class DummyContextFactory implements ContextFactory {
    @Override
    public Context createContext() {
        return new Context() {
            List<DummyGenerator> generators = new ArrayList<DummyGenerator>();

            @Override
            public void startAll() {
            }

            @Override
            public void stopAll() {
            }

            @Override
            public void waitAndUpdateAll() {
                for (DummyGenerator gen: generators) gen.refresh();
            }

            @Override
            public ColorGenerator createColorGenerator() {
                DummyColorGenerator gen = new DummyColorGenerator() {
                    @Override
                    public void dispose() {
                        generators.remove(this);
                    }
                };
                generators.add(gen);
                return gen;
            }

            @Override
            public DepthGenerator createDepthGenerator() {
                DummyDepthGenerator gen = new DummyDepthGenerator() {
                    @Override
                    public void dispose() {
                        generators.remove(this);
                    }
                };
                generators.add(gen);
                return gen;
            }

            @Override
            public void startRecording(File fileName) {
            }

            @Override
            public void stopRecording() {
            }

            @Override
            public void dispose() {
            }
        };
    }
}
