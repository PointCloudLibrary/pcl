package com.itseez.onirec;

import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.DialogInterface;
import android.os.Bundle;

class RecordingPicker extends DialogFragment {
    private final String[] recordings;
    private final DialogInterface.OnClickListener listener;

    public RecordingPicker(String[] recordings, DialogInterface.OnClickListener listener) {
        this.recordings = recordings;
        this.listener = listener;
    }

    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        builder.setTitle(R.string.title_select_recording);
        builder.setItems(recordings, listener);
        return builder.create();
    }
}
