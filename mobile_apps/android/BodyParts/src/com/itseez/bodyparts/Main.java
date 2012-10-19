package com.itseez.bodyparts;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.*;
import org.libusb.UsbHelper;

import java.io.*;
import java.util.*;

public class Main extends Activity implements View.OnClickListener, CompoundButton.OnCheckedChangeListener {
    private static final String TAG = "bodyparts.Main";

    ImageView picture;
    TextView textTiming;
    TextView textStatus;
    CheckBox checkLoop;
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

        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_USB_HOST))
            menu.findItem(R.id.menu_item_open_device).setEnabled(false);

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
        checkLoop = (CheckBox) findViewById(R.id.check_loop);
        checkLoop.setOnCheckedChangeListener(this);

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

        state = new StateStopped(new StateIdle());
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

    private UsbDevice[] findUsableDevices() {
        UsbManager manager = UsbHelper.getManager();

        HashMap<String, UsbDevice> dev_list = manager.getDeviceList();

        List<UsbDevice> usable = new LinkedList<UsbDevice>();

        for (UsbDevice device: dev_list.values())
            if (device.getVendorId() == 0x045e && device.getProductId() == 0x02ae)
                usable.add(device);

        return usable.toArray(new UsbDevice[usable.size()]);
    }

    private State decideNextState(boolean live, boolean vocal) {
        if (live) {
            UsbDevice[] devices = findUsableDevices();

            if (devices.length == 0) {
                if (vocal)
                    Toast.makeText(Main.this, R.string.toast_no_usable_devices, Toast.LENGTH_SHORT).show();
                return null;
            }

            return new StateRequestingPermission(devices);
        } else {
            File rgbd_dir = checkRgbdDirectory();

            if (rgbd_dir == null) {
                if (vocal)
                    Toast.makeText(Main.this, R.string.toast_no_rgbd_directory, Toast.LENGTH_SHORT).show();
                return null;
            }

            return new StateOpening(rgbd_dir);
        }
    }

    private State getNextStateDecider(final boolean live) {
        return new State() {
            @Override
            public void enter() {
                State nextState = decideNextState(live, false);
                setState(nextState == null ? new StateIdle() : nextState);
            }
        };
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.picture:
                state.pictureClicked();
        }
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        switch (buttonView.getId()) {
            case R.id.check_loop:
                state.loopChanged(isChecked);
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

        public void loopChanged(boolean isChecked) {
            throw new IllegalStateException();
        }
    }

    private class StateStopped extends State {
        private final State startState;

        private StateStopped(State startState) {
            this.startState = startState;
        }

        @Override
        public void activityStart() {
            setState(startState);
        }
    }

    private class StateIdle extends State {
        @Override
        public void activityStop() {
            setState(new StateStopped(new StateIdle()));
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
            State nextState = decideNextState(false, true);
            if (nextState != null) setState(nextState);
        }

        @Override
        public void menuItemOpenDevice() {
            State nextState = decideNextState(true, true);
            if (nextState != null) setState(nextState);
        }
    }

    private class StateOpen extends State {
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

                if (checkLoop.isChecked()) loop.doOneFrame();
            }

            @Override
            public void grabberBroken() {
                Toast.makeText(Main.this, R.string.toast_source_lost, Toast.LENGTH_LONG).show();
                setState(new StateClosing(loop, proxyFeedback, false, new StateIdle()));
            }
        };

        public StateOpen(MainLoop loop, ProxyFeedback proxyFeedback) {
            this.loop = loop;
            this.proxyFeedback = proxyFeedback;
        }

        @Override
        public void enter() {
            proxyFeedback.replaceUnderlying(feedback);
            textStatus.setText(R.string.status_open);
            checkLoop.setEnabled(true);
            loop.doOneFrame();
        }

        @Override
        public void loopChanged(boolean isChecked) {
            if (isChecked) loop.doOneFrame();
        }

        @Override
        public void leave() {
            checkLoop.setEnabled(false);
            checkLoop.setChecked(false);
        }

        @Override
        public void activityStop() {
            setState(new StateClosing(loop, proxyFeedback, true, getNextStateDecider(loop.isLive())));
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
            State nextState = decideNextState(false, true);
            if (nextState != null) setState(new StateClosing(loop, proxyFeedback, false, nextState));
        }

        @Override
        public void menuItemOpenDevice() {
            State nextState = decideNextState(true, true);
            if (nextState != null) setState(new StateClosing(loop, proxyFeedback, false, nextState));
        }

        @Override
        public void menuItemClose() {
            setState(new StateClosing(loop, proxyFeedback, false, new StateIdle()));
        }

        @Override
        public void pictureClicked() {
            if (!checkLoop.isChecked()) loop.doOneFrame();
        }
    }

    private class StateOpening extends State {
        private File rgbdDir;
        private MainLoop loop;
        private MainLoop.Feedback feedback = new MainLoop.Feedback() {
            @Override
            public void initFinished(boolean success) {
                if (success) {
                    setState(new StateOpen(loop, proxyFeedback));
                }
                else {
                    Toast.makeText(Main.this, R.string.toast_failed_to_open, Toast.LENGTH_LONG).show();
                    setState(new StateClosing(loop, proxyFeedback, false, new StateIdle()));
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
            setState(new StateClosing(loop, proxyFeedback, true, getNextStateDecider(rgbdDir == null)));
        }

        @Override
        public void menuItemOpenFolder() {
            State nextState = decideNextState(false, true);
            if (nextState != null) setState(new StateClosing(loop, proxyFeedback, false, nextState));
        }

        @Override
        public void menuItemOpenDevice() {
            State nextState = decideNextState(true, true);
            if (nextState != null) setState(new StateClosing(loop, proxyFeedback, false, nextState));
        }

        @Override
        public void menuItemClose() {
            setState(new StateClosing(loop, proxyFeedback, false, new StateIdle()));
        }
    }

    private class StateClosing extends State {
        private final MainLoop loop;
        private final ProxyFeedback proxyFeedback;
        private State nextState, originalNextState;

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

        public StateClosing(MainLoop loop, ProxyFeedback proxyFeedback, boolean stopping, State nextState) {
            this.loop = loop;
            this.proxyFeedback = proxyFeedback;
            this.originalNextState = nextState;
            this.nextState = stopping ? new StateStopped(originalNextState) : originalNextState;
        }

        @Override
        public void enter() {
            proxyFeedback.replaceUnderlying(feedback);
            loop.close();
            textStatus.setText(R.string.status_closing);
        }

        @Override
        public void activityStop() {
            nextState = new StateStopped(originalNextState);
        }

        @Override
        public void activityStart() {
            nextState = originalNextState;
        }
    }

    private class StateRequestingPermission extends State {
        private static final String ACTION_USB_PERMISSION = "com.itseez.bodyparts.USB_PERMISSION";
        private final Set<UsbDevice> remaining;
        private PendingIntent pendingIntent;

        private final BroadcastReceiver permReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                if (!intent.getAction().equals(ACTION_USB_PERMISSION)) return;
                if (!intent.hasExtra(UsbManager.EXTRA_DEVICE)) return;

                UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);

                if (granted) {
                    remaining.remove(device);
                    if (remaining.isEmpty())
                        setState(new StateOpening(null));
                } else {
                    Toast.makeText(Main.this, R.string.toast_permission_denied, Toast.LENGTH_LONG).show();
                    setState(new StateIdle());
                }
            }
        };

        public StateRequestingPermission(UsbDevice[] devices) {
            remaining = new HashSet<UsbDevice>(Arrays.asList(devices));
        }

        @Override
        public void enter() {
            pendingIntent = PendingIntent.getBroadcast(Main.this, 0, new Intent(ACTION_USB_PERMISSION), 0);
            textStatus.setText(R.string.status_requesting_permission);
            registerReceiver(permReceiver, new IntentFilter(ACTION_USB_PERMISSION));

            UsbManager manager = UsbHelper.getManager();

            for (UsbDevice device: remaining)
                manager.requestPermission(device, pendingIntent);
        }

        @Override
        public void leave() {
            pendingIntent.cancel();
            unregisterReceiver(permReceiver);
        }

        @Override
        public void activityStop() {
            setState(new StateStopped(getNextStateDecider(true)));
        }
    }
}
