package com.itseez.onirec;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.*;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.*;
import android.widget.*;
import org.OpenNI.MapOutputMode;
import org.libusb.UsbHelper;

import java.io.File;
import java.io.FilenameFilter;
import java.util.*;

public class MainActivity extends Activity {
    private static final String TAG = "onirec.MainActivity";

    private TextView textStatus;
    private TextView textFps;
    private SurfaceView surfaceColor;
    private SurfaceView surfaceDepth;
    private Spinner spinnerColorMode, spinnerDepthMode;

    private ArrayAdapter<MapModeWrapper> spinnerAdapterColor, spinnerAdapterDepth;

    private static final String ACTION_USB_PERMISSION = "com.itseez.onirec.USB_PERMISSION";

    private State state;

    {
        state = new StateStopped();
        state.enter();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        return true;
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        for (int i = 0; i < menu.size(); ++i)
            menu.getItem(i).setVisible(false);
        menu.findItem(R.id.menu_item_preferences).setVisible(true);
        return state.prepareMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == R.id.menu_item_preferences) {
            startActivity(new Intent(this, PreferencesActivity.class));
            return true;
        }
        return state.menuItemClicked(item);
    }

    private int surfaces = 0;

    private final SurfaceHolder.Callback surface_callbacks = new SurfaceHolder.Callback() {
        @Override
        public void surfaceCreated(SurfaceHolder holder) {
            Log.d(TAG, "surfaceCreated");
            ++surfaces;
            state.surfaceStateChange();
        }

        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
            Log.d(TAG, "surfaceChanged");
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder holder) {
            Log.d(TAG, "surfaceDestroyed");
            --surfaces;
            state.surfaceStateChange();
        }
    };

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        PreferenceManager.setDefaultValues(this, R.xml.preferences, false);

        textStatus = (TextView) findViewById(R.id.text_status);
        textFps = (TextView) findViewById(R.id.text_fps);
        surfaceColor = (SurfaceView) findViewById(R.id.surface_color);
        surfaceDepth = (SurfaceView) findViewById(R.id.surface_depth);
        spinnerColorMode = (Spinner) findViewById(R.id.spinner_color_mode);
        spinnerDepthMode = (Spinner) findViewById(R.id.spinner_depth_mode);

        surfaceColor.getHolder().addCallback(surface_callbacks);
        surfaceDepth.getHolder().addCallback(surface_callbacks);

        spinnerAdapterColor = new ArrayAdapter<MapModeWrapper>(this, android.R.layout.simple_spinner_item);
        spinnerAdapterColor.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerAdapterColor.add(new MapModeWrapper(this, null)); // so that it has proper size right away
        spinnerColorMode.setAdapter(spinnerAdapterColor);

        spinnerAdapterDepth = new ArrayAdapter<MapModeWrapper>(this, android.R.layout.simple_spinner_item);
        spinnerAdapterDepth.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerAdapterDepth.add(new MapModeWrapper(this, null));
        spinnerDepthMode.setAdapter(spinnerAdapterDepth);

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    protected void onStart() {
        super.onStart();
        state.start();
    }

    @Override
    protected void onStop() {
        super.onStop();
        state.stop();
    }

    private void setState(State newState) {
        state.leave();
        state = newState;
        Log.d(TAG, "New state: " + state.getClass().getName());
        state.enter();
        invalidateOptionsMenu();
    }

    private void goToDefaultState() {
        Collection<UsbDevice> devices = findInterestingDevices();

        if (devices.isEmpty())
            setState(new StateIdle(R.string.status_no_devices));
        else
            setState(new StateWaiting(devices, new StateCapturing()));
    }

    private Collection<UsbDevice> findInterestingDevices() {
        UsbManager manager = UsbHelper.getManager();

        HashMap<String, UsbDevice> dev_list = manager.getDeviceList();

        List<UsbDevice> devices = new ArrayList<UsbDevice>();

        for (String dev_name : dev_list.keySet()) {
            UsbDevice device = dev_list.get(dev_name);
            int vid = device.getVendorId(), pid = device.getProductId();

            if ((vid == 0x045e && pid == 0x02ae) || // Microsoft Kinect for Xbox 360
                    (vid == 0x1d27 && pid == 0x0600)) { // ASUS Xtion PRO
                Log.i(TAG, "Requesting USB permission for device " + device.getDeviceName() + ".");
                devices.add(device);
            }
        }

        return devices;
    }

    private void initiateReplay() {
        final String[] recordings = Environment.getExternalStorageDirectory().list(new FilenameFilter() {
            @Override
            public boolean accept(File file, String s) {
                return s.endsWith(".oni");
            }
        });

        new RecordingPicker(recordings, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                setState(new StateWaiting(new ArrayList<UsbDevice>(), new StateReplaying(
                        new File(Environment.getExternalStorageDirectory(), recordings[i]))));
            }
        }).show(getFragmentManager(), "recordings");
    }

    private abstract static class State {
        public void enter() {
        }

        public void leave() {
        }

        public void start() {
            throw new IllegalStateException();
        }

        public void stop() {
            throw new IllegalStateException();
        }

        public void surfaceStateChange() {
            throw new IllegalStateException();
        }

        public boolean prepareMenu(Menu menu) {
            return false;
        }

        public boolean menuItemClicked(MenuItem item) {
            return false;
        }
    }

    private class StateStopped extends State {
        @Override
        public void start() {
            goToDefaultState();
        }

        @Override
        public void surfaceStateChange() {
        }
    }

    private class StateIdle extends State {
        private String status;

        public StateIdle(int formatId, Object... args) {
            status = getResources().getString(formatId, args);
        }

        @Override
        public void enter() {
            textStatus.setText(status);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_reconnect).setVisible(true);
            menu.findItem(R.id.menu_item_replay).setVisible(true);
            return true;
        }

        @Override
        public boolean menuItemClicked(MenuItem item) {
            switch (item.getItemId()) {
                case R.id.menu_item_reconnect:
                    goToDefaultState();
                    return true;
                case R.id.menu_item_replay:
                    initiateReplay();
                    return true;
            }

            return false;
        }
    }

    private class StateWaiting extends State {
        private State stateOnSuccess;
        PendingIntent permIntent;
        private final Set<UsbDevice> awaitingPermission;

        private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();
                if (ACTION_USB_PERMISSION.equals(action)) {
                    synchronized (this) {
                        UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

                        if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                            Log.i(TAG, "USB permission granted for device " + device.getDeviceName() + ".");
                            awaitingPermission.remove(device);
                            setLabel();
                            checkReady();
                        } else {
                            Log.i(TAG, "USB permission denied for device " + device.getDeviceName() + ".");
                            setState(new StateIdle(R.string.status_permission_denied));
                        }
                    }
                }
            }
        };

        public StateWaiting(Collection<UsbDevice> devices, State stateOnSuccess) {
            this.awaitingPermission = new HashSet<UsbDevice>(devices);
            this.stateOnSuccess = stateOnSuccess;
        }

        @Override
        public void enter() {
            registerReceiver(usbReceiver, new IntentFilter(ACTION_USB_PERMISSION));
            permIntent = PendingIntent.getBroadcast(MainActivity.this, 0, new Intent(ACTION_USB_PERMISSION), 0);

            if (!awaitingPermission.isEmpty()) {
                UsbManager manager = UsbHelper.getManager();
                for (UsbDevice device : awaitingPermission)
                    manager.requestPermission(device, permIntent);
            }

            setLabel();
            checkReady();
        }

        @Override
        public void leave() {
            permIntent.cancel();
            unregisterReceiver(usbReceiver);
        }

        private void checkReady() {
            if (surfaces == 2 && awaitingPermission.isEmpty())
                setState(stateOnSuccess);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            setLabel();
            checkReady();
        }

        private void setLabel() {
            if (!awaitingPermission.isEmpty())
                textStatus.setText(R.string.status_waiting_for_permission);
            else
                textStatus.setText(R.string.status_waiting_for_surfaces);
        }
    }

    private class StateReplaying extends State {
        CaptureThreadManager manager;
        final File recording;

        final CaptureThreadManager.Feedback feedback = new CaptureThreadManager.Feedback() {
            @Override
            public void setFps(double fps) {
                textFps.setText(getResources().getString(R.string.x_fps, fps));
            }

            @Override
            public void reportError(Error error, String oniMessage) {
                switch (error) {
                    case FailedToStartCapture:
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_to_start_replay), oniMessage));
                        return;
                    case FailedDuringCapture:
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_during_replay), oniMessage));
                        return;
                    default:
                        throw new IllegalStateException();
                }
            }

            @Override
            public void reportRecordingFinished() {
                throw new IllegalStateException();
            }

            @Override
            public void reportCaptureStarted(MapOutputMode[] colorModes, MapOutputMode currentColorMode,
                                             MapOutputMode[] depthModes, MapOutputMode currentDepthMode) {
                configureModeSpinner(currentColorMode, spinnerColorMode, spinnerAdapterColor);
                configureModeSpinner(currentDepthMode, spinnerDepthMode, spinnerAdapterDepth);
            }

            private void configureModeSpinner(MapOutputMode mode, Spinner spinner, ArrayAdapter<MapModeWrapper> adapter) {
                adapter.clear();
                adapter.add(new MapModeWrapper(MainActivity.this, mode));
                spinner.setVisibility(View.VISIBLE);
                spinner.setEnabled(false);
            }
        };

        public StateReplaying(File recording) {
            this.recording = recording;
        }

        @Override
        public void enter() {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(getResources().getString(R.string.x_fps, 0.));
            textStatus.setText(getResources().getString(R.string.status_replaying, recording.getName()));

            boolean enable_vis = PreferenceManager.getDefaultSharedPreferences(MainActivity.this)
                    .getBoolean(PreferencesActivity.KEY_PREF_ENABLE_VISUALIZATION, true);
            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), feedback,
                    new RecordingContextHolderFactory(recording), enable_vis);
        }

        @Override
        public void leave() {
            manager.stop();

            textFps.setVisibility(View.INVISIBLE);
            spinnerColorMode.setVisibility(View.INVISIBLE);
            spinnerDepthMode.setVisibility(View.INVISIBLE);
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) setState(new StateWaiting(new ArrayList<UsbDevice>(), new StateReplaying(recording)));
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_stop_replaying).setVisible(true);
            return true;
        }

        @Override
        public boolean menuItemClicked(MenuItem item) {
            switch (item.getItemId()) {
                case R.id.menu_item_stop_replaying:
                    goToDefaultState();
                    return true;
            }

            return false;
        }
    }

    private class StateCapturing extends State {
        CaptureThreadManager manager;
        boolean isRecording = false, stoppingRecording = false;
        File currentRecording;

        final CaptureThreadManager.Feedback feedback = new CaptureThreadManager.Feedback() {
            @Override
            public void setFps(double fps) {
                textFps.setText(getResources().getString(R.string.x_fps, fps));
            }

            @Override
            public void reportError(CaptureThreadManager.Feedback.Error error, String oniMessage) {
                switch (error) {
                    case FailedToStartCapture:
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_to_start_capture), oniMessage));
                        return;
                    case FailedDuringCapture:
                        if (isRecording) //noinspection ResultOfMethodCallIgnored
                            currentRecording.delete();
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_during_capture), oniMessage));
                        return;
                    case FailedToStartRecording:
                        setRecordingState(false, false);
                        Toast.makeText(MainActivity.this,
                                getResources().getString(R.string.status_openni_error,
                                        getResources().getString(R.string.error_failed_to_start_recording),
                                        oniMessage),
                                Toast.LENGTH_LONG).show();

                        //noinspection ResultOfMethodCallIgnored
                        currentRecording.delete();
                        return;
                    default:
                        throw new IllegalStateException();
                }
            }

            @Override
            public void reportRecordingFinished() {
                setRecordingState(false, false);
                textStatus.setText(R.string.status_previewing);
            }

            @Override
            public void reportCaptureStarted(MapOutputMode[] colorModes, MapOutputMode currentColorMode,
                                             MapOutputMode[] depthModes, MapOutputMode currentDepthMode) {
                configureModeSpinner(colorModes, currentColorMode, spinnerColorMode, spinnerAdapterColor);
                configureModeSpinner(depthModes, currentDepthMode, spinnerDepthMode, spinnerAdapterDepth);
            }

            private void configureModeSpinner(MapOutputMode[] modes, MapOutputMode currentMode,
                                              Spinner spinner, final ArrayAdapter<MapModeWrapper> adapter) {
                adapter.clear();
                adapter.add(new MapModeWrapper(MainActivity.this, null));
                MapModeWrapper current_mode_wrapper = new MapModeWrapper(MainActivity.this, currentMode);
                int current_mode_index = 0;

                if (modes != null) {
                    Set<MapModeWrapper> unique_modes = new HashSet<MapModeWrapper>(modes.length);

                    for (MapOutputMode mode : modes) {
                        MapModeWrapper new_mode_wrapper = new MapModeWrapper(MainActivity.this, mode);
                        if (!unique_modes.contains(new_mode_wrapper)) {
                            unique_modes.add(new_mode_wrapper);
                            adapter.add(new_mode_wrapper);
                            if (current_mode_wrapper.equals(new_mode_wrapper))
                                current_mode_index = unique_modes.size();
                        }
                    }
                }

                spinner.setSelection(current_mode_index);
                spinner.setVisibility(View.VISIBLE);
                spinner.setEnabled(true);

                final int current_mode_index_final = current_mode_index;

                spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                    int last_position = current_mode_index_final;

                    @Override
                    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                        if (position == last_position) return;
                        last_position = position;

                        switch (parent.getId()) {
                            case R.id.spinner_color_mode:
                                manager.setColorMode(adapter.getItem(position).getMode());
                                break;
                            case R.id.spinner_depth_mode:
                                manager.setDepthMode(adapter.getItem(position).getMode());
                                break;
                        }
                    }

                    @Override
                    public void onNothingSelected(AdapterView<?> adapterView) {
                    }
                });
            }
        };

        private void setRecordingState(boolean enabled, boolean stopping) {
            isRecording = enabled;
            stoppingRecording = stopping;
            spinnerColorMode.setEnabled(!enabled);
            spinnerDepthMode.setEnabled(!enabled);
            invalidateOptionsMenu();
        }

        @Override
        public void enter() {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(getResources().getString(R.string.x_fps, 0.));
            setRecordingState(false, false);

            textStatus.setText(R.string.status_previewing);
            boolean enable_vis = PreferenceManager.getDefaultSharedPreferences(MainActivity.this)
                    .getBoolean(PreferencesActivity.KEY_PREF_ENABLE_VISUALIZATION, true);
            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), feedback,
                    new LiveContextHolderFactory(), enable_vis);
        }

        @Override
        public void leave() {
            manager.stop();
            if (isRecording && manager.hasError()) //noinspection ResultOfMethodCallIgnored
                currentRecording.delete();

            textFps.setVisibility(View.INVISIBLE);
            spinnerColorMode.setVisibility(View.INVISIBLE);
            spinnerDepthMode.setVisibility(View.INVISIBLE);
            spinnerColorMode.setOnItemSelectedListener(null);
            spinnerDepthMode.setOnItemSelectedListener(null);
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) {
                setState(new StateWaiting(new ArrayList<UsbDevice>(), new StateCapturing()));
            }
        }

        @Override
        public boolean prepareMenu(Menu menu) {
            menu.findItem(R.id.menu_item_replay).setVisible(!isRecording);
            menu.findItem(R.id.menu_item_record).setVisible(!isRecording);
            menu.findItem(R.id.menu_item_stop_recording).setVisible(isRecording);

            menu.findItem(R.id.menu_item_stop_recording).setEnabled(!stoppingRecording);
            return true;
        }

        @Override
        public boolean menuItemClicked(MenuItem item) {
            switch (item.getItemId()) {
                case R.id.menu_item_replay: {
                    initiateReplay();
                    return true;
                }
                case R.id.menu_item_record: {
                    int file_no = 0;

                    do
                        currentRecording = new File(Environment.getExternalStorageDirectory(), "recording" + file_no++ + ".oni");
                    while (currentRecording.exists());

                    manager.startRecording(currentRecording);

                    setRecordingState(true, false);
                    textStatus.setText(getResources().getString(R.string.status_recording_to,
                            currentRecording.getAbsolutePath()));

                    return true;
                }
                case R.id.menu_item_stop_recording: {
                    manager.stopRecording();
                    setRecordingState(true, true);
                    return true;
                }
            }

            return false;
        }
    }
}
