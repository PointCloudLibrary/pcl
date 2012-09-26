package com.itseez.onirec;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import org.libusb.UsbHelper;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class MainActivity extends Activity {
    static final String TAG = "onirec.MainActivity";

    private Button buttonRecord;
    private TextView textStatus;
    private TextView textFps;
    private SurfaceView surfaceColor;
    private SurfaceView surfaceDepth;

    private static final String ACTION_USB_PERMISSION = "com.itseez.onirec.USB_PERMISSION";

    State state;

    {
        state = new StateStopped();
        state.enter();
    }

    Set<UsbDevice> awaitingPermission = new HashSet<UsbDevice>();
    private int surfaces = 0;

    private BroadcastReceiver usb_receiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                        Log.i(TAG, "USB permission granted for device " + device.getDeviceName() + ".");
                        awaitingPermission.remove(device);

                        if (awaitingPermission.isEmpty())
                            state.usbPermissionChange(true);
                    }
                    else {
                        Log.d(TAG, "USB permission denied.");
                        state.usbPermissionChange(false);
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

    private SurfaceHolder.Callback surface_callbacks = new SurfaceHolder.Callback() {
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
        textStatus = (TextView) findViewById(R.id.text_status);
        textFps = (TextView) findViewById(R.id.text_fps);
        surfaceColor = (SurfaceView) findViewById(R.id.surface_color);
        surfaceDepth = (SurfaceView) findViewById(R.id.surface_depth);

        surfaceColor.getHolder().addCallback(surface_callbacks);
        surfaceDepth.getHolder().addCallback(surface_callbacks);

        registerReceiver(usb_receiver, new IntentFilter(ACTION_USB_PERMISSION));
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

    @SuppressWarnings("UnusedDeclaration")
    public void buttonRecordOnClick(View view) {
        state.recordClicked();
    }

    private void setState(State newState) {
        state.leave();
        state = newState;
        Log.d(TAG, "New state: " + state.getClass().getName());
        state.enter();
    }

    private abstract static class State {
        public void enter() {}
        public void leave() {}

        public void start() { throw new IllegalStateException(); }
        public void stop() { throw new IllegalStateException(); }
        public void surfaceStateChange() { throw new IllegalStateException(); }
        public void usbPermissionChange(boolean granted) { throw new IllegalStateException(); }
        public void recordClicked() { throw new IllegalStateException(); }
    }

    private class StateStopped extends State {
        @Override
        public void start() {
            UsbManager manager = UsbHelper.getManager();
            PendingIntent perm_intent = PendingIntent.getBroadcast(MainActivity.this, 0, new Intent(ACTION_USB_PERMISSION), 0);

            HashMap<String, UsbDevice> dev_list = manager.getDeviceList();

            awaitingPermission.clear();

            for (String dev_name: dev_list.keySet()) {
                UsbDevice device = dev_list.get(dev_name);

                if (device.getVendorId() == 0x045e && device.getProductId() == 0x02ae) {
                    Log.i(TAG, "Requesting USB permission.");
                    awaitingPermission.add(device);
                    manager.requestPermission(device, perm_intent);
                }
            }

            if (awaitingPermission.isEmpty())
                setState(new StateNoDevice());
            else
                setState(new StateWaiting());
        }
    }

    private class StateNoDevice extends State {
        @Override
        public void enter() {
            textStatus.setText(R.string.status_no_devices);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override public void surfaceStateChange() { }

        @Override public void usbPermissionChange(boolean granted) { }
    }

    private class StateWaiting extends State {
        private void setLabel() {
            if (!awaitingPermission.isEmpty())
                textStatus.setText(R.string.status_waiting_for_permission);
            else
                textStatus.setText(R.string.status_waiting_for_surfaces);
        }

        private void maybeGoReady() {
            if (awaitingPermission.isEmpty() && surfaces == 2)
                setState(new StateCapturing());
        }

        @Override
        public void enter() {
            setLabel();
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            setLabel();
            maybeGoReady();
        }

        @Override
        public void usbPermissionChange(boolean granted) {
            if (granted) {
                setLabel();
                maybeGoReady();
            } else {
                setState(new StateNoDevice());
            }
        }
    }

    private class StateCapturing extends State {
        CaptureThreadManager manager;
        boolean isRecording = false;

        @Override
        public void enter() {
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(String.format(getResources().getString(R.string.x_fps), 0.));
            buttonRecord.setVisibility(View.VISIBLE);
            buttonRecord.setText(R.string.record_start);

            textStatus.setText(R.string.status_previewing);
            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), new CaptureThreadManager.Feedback() {
                @Override
                public void setFps(double fps) {
                    textFps.setText(String.format(getResources().getString(R.string.x_fps), fps));
                }
            });
        }

        @Override
        public void leave() {
            manager.stop();
            textFps.setVisibility(View.INVISIBLE);
            buttonRecord.setVisibility(View.INVISIBLE);
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) {
                setState(new StateWaiting());
            }
        }

        @Override
        public void usbPermissionChange(boolean granted) {
            if (!granted) {
                setState(new StateNoDevice());
            }
        }

        @Override
        public void recordClicked() {
            if (isRecording) {
                manager.stopRecording();
                isRecording = false;
                buttonRecord.setText(R.string.record_start);
                textStatus.setText(R.string.status_previewing);
            } else {
                int file_no = 0;
                File capture_path;

                do
                    capture_path = new File(Environment.getExternalStorageDirectory(), "test" + file_no++ + ".oni");
                while (capture_path.exists());

                manager.startRecording(capture_path);

                isRecording = true;
                buttonRecord.setText(R.string.record_stop);
                textStatus.setText(String.format(getResources().getString(R.string.status_recording_to),
                        capture_path.getAbsolutePath()));
            }
        }
    }
}
