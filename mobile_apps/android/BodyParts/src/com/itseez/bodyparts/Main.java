package com.itseez.bodyparts;

import android.app.Activity;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.LayoutInflater;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import org.libusb.UsbHelper;

import java.io.*;
import java.util.Arrays;

public class Main extends Activity {
    private static final String TAG = "bodyparts.Main";

    ImageView picture;
    TextView timingText;
    MainLoop loop;

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
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        picture = (ImageView) findViewById(R.id.picture);
        timingText = (TextView) findViewById(R.id.timing_text);

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

            File rgbd_dir = new File(Environment.getExternalStorageDirectory(), "rgbd");
            if (rgbd_dir.isDirectory() && rgbd_dir.listFiles(new FilenameFilter() {
                @Override
                public boolean accept(File file, String s) {
                    return s.endsWith(".rgbd");
                }
            }).length >= 1)
                loop = new MainLoop(this, new BodyPartsRecognizer(trees), rgbd_dir);
            else if (UsbHelper.getManager().getDeviceList().size() >= 1)
                loop = new MainLoop(this, new BodyPartsRecognizer(trees));
        } catch (IOException ioe) {
            Log.e(TAG, "Couldn't read tree assets.", ioe);
            throw new RuntimeException(ioe);
        }
    }

    @Override
    public void onDestroy() {
        loop.destroy();
        super.onDestroy();
    }

    @Override
    protected void onPause() {
        super.onPause();
        loop.pause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        loop.resume();
    }

    @Override
    protected void onStop() {
        super.onStop();
        loop.stop();
    }

    @Override
    protected void onStart() {
        super.onStart();
        loop.start();
    }
}
