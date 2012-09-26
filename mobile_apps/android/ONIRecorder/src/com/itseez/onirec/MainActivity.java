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
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.TextView;
import org.libusb.UsbHelper;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class MainActivity extends Activity {
    static final String TAG = "onirec.MainActivity";

    private TextView textStatus;
    private TextView textFps;
    private SurfaceView surfaceColor;
    private SurfaceView surfaceDepth;

    private static final String ACTION_USB_PERMISSION = "com.itseez.onirec.USB_PERMISSION";

    State state;
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

        textStatus = (TextView) findViewById(R.id.text_status);
        textFps = (TextView) findViewById(R.id.text_fps);
        surfaceColor = (SurfaceView) findViewById(R.id.surface_color);
        surfaceDepth = (SurfaceView) findViewById(R.id.surface_depth);

        surfaceColor.getHolder().addCallback(surface_callbacks);
        surfaceDepth.getHolder().addCallback(surface_callbacks);

        registerReceiver(usb_receiver, new IntentFilter(ACTION_USB_PERMISSION));
        setState(new StateStopped());
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
        state = newState;
        Log.d(TAG, "New state: " + state.getClass().getName());
        state.activate();
    }

    private interface State {
        void activate();

        void start();
        void stop();
        void surfaceStateChange();
        void usbPermissionChange(boolean granted);
    }

    private class StateStopped implements State {
        @Override
        public void activate() {
            textFps.setVisibility(View.INVISIBLE);
        }

        @Override
        public void stop() {
            throw new IllegalStateException("already stopped");
        }

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

        @Override
        public void usbPermissionChange(boolean granted) {
            throw new IllegalStateException("stopped");
        }

        @Override
        public void surfaceStateChange() {
            throw new IllegalStateException("stopped");
        }
    }

    private class StateNoDevice implements State {
        @Override
        public void activate() {
            textFps.setVisibility(View.INVISIBLE);
            textStatus.setText(R.string.status_no_devices);
        }

        @Override
        public void start() {
            throw new IllegalStateException("already started");
        }

        @Override
        public void stop() {
            setState(new StateStopped());
        }

        @Override public void surfaceStateChange() { }

        @Override public void usbPermissionChange(boolean granted) { }
    }

    private class StateWaiting implements State {
        private void setLabel() {
            textFps.setVisibility(View.INVISIBLE);

            if (!awaitingPermission.isEmpty())
                textStatus.setText(R.string.status_waiting_for_permission);
            else
                textStatus.setText(R.string.status_waiting_for_surfaces);
        }

        private void maybeGoReady() {
            if (awaitingPermission.isEmpty() && surfaces == 2)
                setState(new StatePreviewing());
        }

        @Override
        public void activate() {
            setLabel();
        }

        @Override
        public void start() {
            throw new IllegalStateException("already started");
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

    private class StatePreviewing implements State {
        CaptureThreadManager manager;

        @Override
        public void activate() {
            textFps.setVisibility(View.VISIBLE);
            textFps.setText(String.format(getResources().getString(R.string.x_fps), 0.));
            textStatus.setText(R.string.status_previewing);
            manager = new CaptureThreadManager(surfaceColor.getHolder(), surfaceDepth.getHolder(), new CaptureThreadManager.Feedback() {
                @Override
                public void setFps(double fps) {
                    textFps.setText(String.format(getResources().getString(R.string.x_fps), fps));
                }
            });
        }

        @Override
        public void start() {
            throw new IllegalStateException("already started");
        }

        @Override
        public void stop() {
            manager.stop();
            setState(new StateStopped());
        }

        @Override
        public void surfaceStateChange() {
            if (surfaces < 2) {
                manager.stop();
                setState(new StateWaiting());
            }
        }

        @Override
        public void usbPermissionChange(boolean granted) {
            if (!granted) {
                manager.stop();
                setState(new StateNoDevice());
            }
        }
    }
}
