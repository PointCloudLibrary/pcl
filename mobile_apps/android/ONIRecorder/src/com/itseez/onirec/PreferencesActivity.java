package com.itseez.onirec;

import android.app.Activity;
import android.os.Bundle;
import android.preference.PreferenceFragment;

public class PreferencesActivity extends Activity {
    public static String KEY_PREF_ENABLE_VISUALIZATION = "enable_visualization";

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getFragmentManager().beginTransaction()
                .replace(android.R.id.content, new Fragment())
                .commit();
    }

    public static class Fragment extends PreferenceFragment {
        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.preferences);
        }
    }
}
