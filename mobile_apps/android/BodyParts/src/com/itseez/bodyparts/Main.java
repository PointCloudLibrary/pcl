package com.itseez.bodyparts;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.*;
import java.util.Arrays;

public class Main extends Activity implements View.OnClickListener {
    private static final String TAG = "bodyparts.Main";

    ImageView picture;
    TextView textTiming;
    TextView textStatus;
    private State state;
    private BodyPartsRecognizer recognizer;

    private void setState(State newState) {
        if (state == null) return; // either we're not yet born, or we're already dead
        state.leave();
        state = newState;
        state.enter();
        invalidateOptionsMenu();
    }

    private byte[] readStream(InputStream stream) throws IOException {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        final int chunk_size = 1000000;
        byte[] chunk = new byte[chunk_size];
        int read;

        while ((read = stream.read(chunk)) != -1)
            baos.write(chunk, 0, read);

        return baos.toByteArray();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        return true;
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        return state.prepareMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_item_open_folder:
                state.menuItemOpenFolder();
                return true;
            case R.id.menu_item_open_device:
                state.menuItemOpenDevice();
                return true;
            case R.id.menu_item_close:
                state.menuItemClose();
                return true;
            default:
                return false;
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        picture = (ImageView) findViewById(R.id.picture);
        picture.setOnClickListener(this);
        textTiming = (TextView) findViewById(R.id.text_timing);
        textStatus = (TextView) findViewById(R.id.text_status);

        ListView color_ref = (ListView) findViewById(R.id.color_list);
        color_ref.setAdapter(new BodyPartLabelAdapter(this, android.R.layout.simple_list_item_1, BodyPartLabel.values(),
                (LayoutInflater) getSystemService(LAYOUT_INFLATER_SERVICE)));

        try {
            File[] treePaths = new File(Environment.getExternalStorageDirectory(), "trees").listFiles();
            Arrays.sort(treePaths);
            byte[][] trees = new byte[treePaths.length][];

            for (int ti = 0; ti < trees.length; ++ti)
            {
                FileInputStream tree_stream = new FileInputStream(treePaths[ti]);

                try {
                    trees[ti] = readStream(new FileInputStream(treePaths[ti]));
                } finally {
                    tree_stream.close();
                }
            }

            recognizer = new BodyPartsRecognizer(trees);
        } catch (IOException ioe) {
            Log.e(TAG, "Couldn't read tree assets.", ioe);
            throw new RuntimeException(ioe);
        }

        state = new StateStopped();
        state.enter();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        state.leave();
        state = null;
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onStop() {
        super.onStop();
        state.activityStop();
    }

    @Override
    protected void onStart() {
        super.onStart();
        state.activityStart();
    }

    private File checkRgbdDirectory() {
        File rgbd_dir = new File(Environment.getExternalStorageDirectory(), "rgbd");

        if (!rgbd_dir.isDirectory() || rgbd_dir.listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File file, String s) {
                return s.endsWith(".rgbd");
            }
        }).length == 0)
            return null;

        return rgbd_dir;
    }

    @Override
    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.picture:
                state.pictureClicked();
        }
    }

    private abstract static class State {
        public void enter() {}
        public void leave() {}

        public void activityStart() {
            throw new IllegalStateException();
        }

        public void activityStop() {
            throw new IllegalStateException();
        }

        public boolean prepareMenu(Menu menu) { return false; }

        public void menuItemOpenFolder() {
            throw new IllegalStateException();
        }

        public void menuItemOpenDevice() {
            throw new IllegalStateException();
        }

        public void menuItemClose() {
            throw new IllegalStateException();
        }

        public void pictureClicked() { }
    }

    private class StateStopped extends State {
        @Override
        public void activityStart() {
            setState(new StateIdle());
        }
    }

    private class StateIdle extends State {
        @Override
        public void activityStop() {
            setState(new StateStopped());
        }

        @Override
        public void enter() {
            textStatus.setText(R.string.status_idle);
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_open_folder).setVisible(true);
            menu.findItem(R.id.menu_item_open_device).setVisible(true);
            menu.findItem(R.id.menu_item_close).setVisible(false);
            return true;
        }

        @Override
        public void menuItemOpenFolder() {
            File rgbd_dir = checkRgbdDirectory();

            if (rgbd_dir == null) {
                Toast.makeText(Main.this, R.string.toast_no_rgbd_directory, Toast.LENGTH_SHORT).show();
                return;
            }

            setState(new StateOpening(rgbd_dir));
        }

        @Override
        public void menuItemOpenDevice() {
            super.menuItemOpenDevice();    //To change body of overridden methods use File | Settings | File Templates.
            // TODO: write this
        }
    }

    private class StatePaused extends State {
        private final MainLoop loop;
        private final ProxyFeedback proxyFeedback;
        private final MainLoop.Feedback feedback = new MainLoop.Feedback() {
            @Override
            public void initFinished(boolean success) {
                throw new IllegalStateException();
            }

            @Override
            public void closeFinished() {
                throw new IllegalStateException();
            }

            @Override
            public void newFrame(long timeMs, Bitmap frame) {
                picture.setImageBitmap(frame);

                textTiming.setText(String.format(getResources().getString(R.string.timing_ms), timeMs));
                Log.i(TAG, "Total: " + textTiming.getText());
            }

            @Override
            public void grabberBroken() {
                Toast.makeText(Main.this, R.string.toast_source_lost, Toast.LENGTH_LONG);
                setState(new StateClosing(loop, proxyFeedback, new StateIdle()));
            }
        };

        public StatePaused(MainLoop loop, ProxyFeedback proxyFeedback) {
            this.loop = loop;
            this.proxyFeedback = proxyFeedback;
        }

        @Override
        public void enter() {
            proxyFeedback.replaceUnderlying(feedback);
            textStatus.setText(R.string.status_paused);
            loop.doOneFrame();
        }

        @Override
        public void activityStop() {
            setState(new StateClosing(loop, proxyFeedback, new StateStopped()));
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_open_folder).setVisible(loop.isLive());
            menu.findItem(R.id.menu_item_open_device).setVisible(!loop.isLive());
            menu.findItem(R.id.menu_item_close).setVisible(true);
            return true;
        }

        @Override
        public void menuItemOpenFolder() {
            File rgbd_dir = checkRgbdDirectory();

            if (rgbd_dir == null) {
                Toast.makeText(Main.this, R.string.toast_no_rgbd_directory, Toast.LENGTH_SHORT).show();
                return;
            }

            setState(new StateClosing(loop, proxyFeedback, new StateOpening(rgbd_dir)));
        }

        @Override
        public void menuItemOpenDevice() {
            super.menuItemOpenDevice();    //To change body of overridden methods use File | Settings | File Templates.
            // TODO: write this
        }

        @Override
        public void menuItemClose() {
            setState(new StateClosing(loop, proxyFeedback, new StateIdle()));
        }

        @Override
        public void pictureClicked() {
            loop.doOneFrame();
        }
    }

    private class StateOpening extends State {
        private File rgbdDir;
        private MainLoop loop;
        private MainLoop.Feedback feedback = new MainLoop.Feedback() {
            @Override
            public void initFinished(boolean success) {
                if (success) {
                    setState(new StatePaused(loop, proxyFeedback));
                }
                else {
                    Toast.makeText(Main.this, R.string.toast_failed_to_open, Toast.LENGTH_LONG);
                    setState(new StateClosing(loop, proxyFeedback, new StateIdle()));
                }
            }

            @Override
            public void closeFinished() {
                throw new IllegalStateException();
            }

            @Override
            public void grabberBroken() {
                throw new IllegalStateException();
            }

            @Override
            public void newFrame(long timeMs, Bitmap frame) {
                throw new IllegalStateException();
            }
        };
        private ProxyFeedback proxyFeedback = new ProxyFeedback(Main.this, feedback);

        public StateOpening(File rgbdDir) {
            this.rgbdDir = rgbdDir;
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_open_folder).setVisible(rgbdDir == null);
            menu.findItem(R.id.menu_item_open_device).setVisible(rgbdDir != null);
            menu.findItem(R.id.menu_item_close).setVisible(true);
            return true;
        }

        @Override
        public void enter() {
            loop = new MainLoop(recognizer, rgbdDir, proxyFeedback);
            textStatus.setText(R.string.status_opening);
        }

        @Override
        public void activityStop() {
            setState(new StateClosing(loop, proxyFeedback, new StateStopped()));
        }

        @Override
        public void menuItemOpenFolder() {
            File rgbd_dir = checkRgbdDirectory();

            if (rgbd_dir == null) {
                Toast.makeText(Main.this, R.string.toast_no_rgbd_directory, Toast.LENGTH_SHORT).show();
                return;
            }

            setState(new StateClosing(loop, proxyFeedback, new StateOpening(rgbd_dir)));
        }

        @Override
        public void menuItemOpenDevice() {
            super.menuItemOpenDevice();    //To change body of overridden methods use File | Settings | File Templates.
            // TODO: write this
        }

        @Override
        public void menuItemClose() {
            setState(new StateClosing(loop, proxyFeedback, new StateIdle()));
        }
    }

    private class StateClosing extends State {
        private final MainLoop loop;
        private final ProxyFeedback proxyFeedback;
        private State nextState;

        private final MainLoop.Feedback feedback = new MainLoop.Feedback() {
            @Override
            public void initFinished(boolean success) { /* ignore */ }

            @Override
            public void closeFinished() {
                loop.destroy();
                setState(nextState);
            }

            @Override
            public void grabberBroken() { /* ignore */ }

            @Override
            public void newFrame(long timeMs, Bitmap frame) { /* ignore */ }
        };

        public StateClosing(MainLoop loop, ProxyFeedback proxyFeedback, State nextState) {
            this.loop = loop;
            this.proxyFeedback = proxyFeedback;
            this.nextState = nextState;
        }

        @Override
        public void enter() {
            proxyFeedback.replaceUnderlying(feedback);
            loop.close();
            textStatus.setText(R.string.status_closing);
        }

        @Override
        public void activityStop() {
            nextState = new StateStopped();
        }

        @Override
        public void activityStart() {
            nextState = new StateIdle();
        }
    }

    private class StateRunning extends State {

    }
}
