package org.pointclouds.onirec;

import android.app.Activity;
import android.os.Bundle;
import android.preference.PreferenceFragment;

public class PreferencesActivity extends Activity {
    public static final String KEY_PREF_ENABLE_VISUALIZATION = "enable_visualization";
    public static final String KEY_PREF_USE_DUMMY_GRABBER = "use_dummy_grabber";

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getFragmentManager().beginTransaction()
                .replace(android.R.id.content, new Fragment())
                .commit();
    }

    @SuppressWarnings("WeakerAccess") // this has to be public, otherwise it breaks in subtle ways
    public static class Fragment extends PreferenceFragment {
        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.preferences);
        }
    }
}
