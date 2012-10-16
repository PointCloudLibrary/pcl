package com.itseez.onirec;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.*;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import org.libusb.UsbHelper;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class MainActivity extends Activity {
    private static final String TAG = "onirec.MainActivity";

    private Button buttonRecord;
    private Button buttonReplay;
    private TextView textStatus;
    private TextView textFps;
    private SurfaceView surfaceColor;
    private SurfaceView surfaceDepth;

    private static final String ACTION_USB_PERMISSION = "com.itseez.onirec.USB_PERMISSION";

    private State state;

    {
        state = new StateStopped();
        state.enter();
    }

    private final Set<UsbDevice> awaitingPermission = new HashSet<UsbDevice>();
    private int surfaces = 0;

    private File lastRecording;

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
                        state.usbPermissionChange(device, true);
                    }
                    else {
                        Log.i(TAG, "USB permission denied for device " + device.getDeviceName() + ".");
                        state.usbPermissionChange(device, false);
                    }
                }
            } else if (action.equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
                UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                Log.i(TAG, "USB device attached: " + device.getDeviceName());
            } else if (action.equals(UsbManager.ACTION_USB_DEVICE_DETACHED)) {
                UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                Log.i(TAG, "USB device detached: " + device.getDeviceName());
            }
        }
    };

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

        buttonRecord = (Button) findViewById(R.id.button_record);
        buttonReplay = (Button) findViewById(R.id.button_replay);
        textStatus = (TextView) findViewById(R.id.text_status);
        textFps = (TextView) findViewById(R.id.text_fps);
        surfaceColor = (SurfaceView) findViewById(R.id.surface_color);
        surfaceDepth = (SurfaceView) findViewById(R.id.surface_depth);

        String lastRecordingName = getPreferences(MODE_PRIVATE).getString("lastRecording", null);
        updateLastRecording(lastRecordingName == null ? null : new File(lastRecordingName));

        surfaceColor.getHolder().addCallback(surface_callbacks);
        surfaceDepth.getHolder().addCallback(surface_callbacks);

        registerReceiver(usbReceiver, new IntentFilter(ACTION_USB_PERMISSION));
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(usbReceiver);
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

        SharedPreferences settings = getPreferences(MODE_PRIVATE);
        SharedPreferences.Editor settings_editor = settings.edit();
        settings_editor.putString("lastRecording", lastRecording == null ? null : lastRecording.getAbsolutePath());
        settings_editor.commit();
    }

    @SuppressWarnings("UnusedDeclaration")
    public void onClick(View view) {
        if (view == buttonRecord)
            state.recordClicked();
        else if (view == buttonReplay)
            state.replayClicked();
    }

    private void setState(State newState) {
        state.leave();
        state = newState;
        Log.d(TAG, "New state: " + state.getClass().getName());
        state.enter();
    }

    private void updateLastRecording(File lastRecording) {
        this.lastRecording = lastRecording;
        String format = getResources().getString(R.string.replay_start);

        if (lastRecording != null)
        {
            buttonReplay.setText(String.format(format, lastRecording.getName()));
            buttonReplay.setEnabled(true);
        }
        else
        {
            buttonReplay.setText(String.format(format, getResources().getString(R.string.last_recording)));
            buttonReplay.setEnabled(false);
        }
    }

    private void findInterestingDevices() {
        UsbManager manager = UsbHelper.getManager();

        HashMap<String, UsbDevice> dev_list = manager.getDeviceList();

        awaitingPermission.clear();

        for (String dev_name: dev_list.keySet()) {
            UsbDevice device = dev_list.get(dev_name);
            int vid = device.getVendorId(), pid = device.getProductId();

            if ((vid == 0x045e && pid == 0x02ae) || // Microsoft Kinect for Xbox 360
                (vid == 0x1d27 && pid == 0x0600)) { // ASUS Xtion PRO
                Log.i(TAG, "Requesting USB permission for device " + device.getDeviceName() + ".");
                awaitingPermission.add(device);
            }
        }
    }

    private abstract static class State {
        public void enter() {}
        public void leave() {}

        public void start() { throw new IllegalStateException(); }
        public void stop() { throw new IllegalStateException(); }
        public void surfaceStateChange() { throw new IllegalStateException(); }
        public void usbPermissionChange(UsbDevice device, boolean granted) { throw new IllegalStateException(); }
        public void recordClicked() { throw new IllegalStateException(); }
        public void replayClicked() { throw new IllegalStateException(); }
    }

    private class StateStopped extends State {
        @Override
        public void start() {
            findInterestingDevices();

            if (awaitingPermission.isEmpty())
                setState(new StateIdle(R.string.status_no_devices));
            else
                setState(new StateWaiting(new StateCapturing()));
        }

        @Override
        public void surfaceStateChange() { }
    }

    private class StateIdle extends State {
        private String status;

        public StateIdle(int formatId, Object... args) {
            status = String.format(getResources().getString(formatId), args);
        }

        @Override
        public void enter() {
            textStatus.setText(status);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override public void surfaceStateChange() { }

        @Override public void usbPermissionChange(UsbDevice device, boolean granted) { }

        @Override
        public void replayClicked() {
            awaitingPermission.clear();
            setState(new StateWaiting(new StateReplaying()));
        }
    }

    private class StateWaiting extends State {
        private State stateOnSuccess;
        PendingIntent permIntent;

        public StateWaiting(State stateOnSuccess) {
            this.stateOnSuccess = stateOnSuccess;
        }

        @Override
        public void enter() {
            permIntent = PendingIntent.getBroadcast(MainActivity.this, 0, new Intent(ACTION_USB_PERMISSION), 0);

            if (!awaitingPermission.isEmpty()) {
                UsbManager manager = UsbHelper.getManager();
                for (UsbDevice device: awaitingPermission)
                    manager.requestPermission(device, permIntent);
            }

            setLabel();
            checkReady();
        }

        @Override
        public void leave() {
            permIntent.cancel();
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

        @Override
        public void usbPermissionChange(UsbDevice device, boolean granted) {
            if (granted) {
                setLabel();
                checkReady();
            } else {
                setState(new StateIdle(R.string.status_permission_denied));
            }
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

        @Override
        public void enter() {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            buttonReplay.setText(R.string.replay_stop);
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(String.format(getResources().getString(R.string.x_fps), 0.));
            textStatus.setText(String.format(getResources().getString(R.string.status_replaying), lastRecording.getName()));

            CaptureThreadManager.Feedback feedback = new CaptureThreadManager.Feedback() {
                @Override
                public void setFps(double fps) {
                    textFps.setText(String.format(getResources().getString(R.string.x_fps), fps));
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
            };

            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), feedback, lastRecording);
        }

        @Override
        public void leave() {
            manager.stop();

            textFps.setVisibility(View.INVISIBLE);
            updateLastRecording(lastRecording);
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) setState(new StateWaiting(new StateCapturing()));
        }

        @Override
        public void usbPermissionChange(UsbDevice device, boolean granted) { }

        @Override
        public void replayClicked() {
            updateLastRecording(lastRecording);

            findInterestingDevices();

            if (awaitingPermission.isEmpty())
                setState(new StateIdle(R.string.status_no_devices));
            else
                setState(new StateWaiting(new StateCapturing()));
        }
    }

    private class StateCapturing extends State {
        CaptureThreadManager manager;
        boolean isRecording = false;
        File currentRecording;

        private void setRecordingState(boolean enabled) {
            isRecording = enabled;
            buttonRecord.setEnabled(true);
            buttonRecord.setText(enabled ? R.string.record_stop : R.string.record_start);
        }

        @Override
        public void enter() {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(String.format(getResources().getString(R.string.x_fps), 0.));
            buttonRecord.setVisibility(View.VISIBLE);
            setRecordingState(false);

            textStatus.setText(R.string.status_previewing);
            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), new CaptureThreadManager.Feedback() {
                @Override
                public void setFps(double fps) {
                    textFps.setText(String.format(getResources().getString(R.string.x_fps), fps));
                }

                @Override
                public void reportError(Error error, String oniMessage) {
                    switch (error) {
                    case FailedToStartCapture:
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_to_start_capture), oniMessage));
                        return;
                    case FailedDuringCapture:
                        if (isRecording) updateLastRecording(null);
                        setState(new StateIdle(R.string.status_openni_error,
                                getResources().getString(R.string.error_failed_during_capture), oniMessage));
                        return;
                    case FailedToStartRecording:
                        setRecordingState(false);
                        Toast.makeText(MainActivity.this,
                                String.format(getResources().getString(R.string.status_openni_error),
                                        getResources().getString(R.string.error_failed_to_start_recording),
                                        oniMessage),
                                Toast.LENGTH_LONG).show();
                        updateLastRecording(null);
                    default:
                        throw new IllegalStateException();
                    }
                }

                @Override
                public void reportRecordingFinished() {
                    setRecordingState(false);
                    textStatus.setText(R.string.status_previewing);
                    updateLastRecording(currentRecording);
                }
            });
        }

        @Override
        public void leave() {
            manager.stop();
            if (isRecording)
                updateLastRecording(manager.hasError() ? null : currentRecording);

            textFps.setVisibility(View.INVISIBLE);
            buttonRecord.setVisibility(View.INVISIBLE);
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) {
                setState(new StateWaiting(new StateCapturing()));
            }
        }

        @Override
        public void usbPermissionChange(UsbDevice device, boolean granted) {
            if (!granted) {
                setState(new StateIdle(R.string.status_permission_denied));
            }
        }

        @Override
        public void recordClicked() {
            if (isRecording) {
                manager.stopRecording();
                buttonRecord.setEnabled(false);
            } else {
                int file_no = 0;

                do
                    currentRecording = new File(Environment.getExternalStorageDirectory(), "recording" + file_no++ + ".oni");
                while (currentRecording.exists());

                manager.startRecording(currentRecording);

                setRecordingState(true);
                textStatus.setText(String.format(getResources().getString(R.string.status_recording_to),
                        currentRecording.getAbsolutePath()));
                buttonReplay.setEnabled(false);
            }
        }

        @Override
        public void replayClicked() {
            setState(new StateReplaying());
        }
    }
}
