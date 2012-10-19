package com.itseez.bodyparts;

import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.util.Log;
import android.util.Xml;
import org.libusb.UsbHelper;
import org.xmlpull.v1.XmlSerializer;

import java.io.*;

public class Application extends android.app.Application {
    private static final String TAG = "bodyparts.Application";

    static {
        System.loadLibrary("usb-1.0");
        System.loadLibrary("OpenNI");
        System.loadLibrary("bodyparts");
    }

    private void extractAssets(File niOutDir) {
        File ni_data_dir = new File(niOutDir, "data");

        AssetManager assets = getAssets();

        try {
            if (!ni_data_dir.mkdirs() && !ni_data_dir.isDirectory()) throw new IOException();

            InputStream asset_dir_in = assets.open("ni/dir.txt");
            BufferedReader asset_dir_reader = new BufferedReader(new InputStreamReader(asset_dir_in));

            String asset_name;
            while ((asset_name = asset_dir_reader.readLine()) != null) {
                File asset_in_path = new File("ni", asset_name);
                InputStream asset_in = assets.open(asset_in_path.getPath());
                BufferedInputStream buffered_in = new BufferedInputStream(asset_in);

                File asset_out_path = new File(niOutDir, asset_name);

                if (!asset_out_path.getParentFile().mkdirs()
                        && !asset_out_path.getParentFile().isDirectory()) throw new IOException();

                OutputStream asset_out = new FileOutputStream(asset_out_path);
                BufferedOutputStream buffered_out = new BufferedOutputStream(asset_out);

                int b;

                while ((b = buffered_in.read()) != -1)
                    buffered_out.write(b);

                buffered_out.close();
                asset_out.close();

                buffered_in.close();
                asset_in.close();

            }

            asset_dir_reader.close();
            asset_dir_in.close();


        } catch (IOException e) {
            Log.e(TAG, Log.getStackTraceString(e));
            throw new RuntimeException(e);
        }
    }

    private void writeModulesXml(File niOutDir, String[] modules, File libDir) {
        XmlSerializer config_writer = Xml.newSerializer();

        try {
            FileOutputStream config_out = new FileOutputStream(new File(niOutDir, "data/modules.xml"));
            config_writer.setOutput(new OutputStreamWriter(config_out, "UTF-8"));

            config_writer.startDocument("UTF-8", null);
            config_writer.startTag(null, "Modules");

            for (String module_name : modules) {
                config_writer.startTag(null, "Module");
                config_writer.attribute(null, "path", new File(libDir, "lib" + module_name + ".so").getAbsolutePath());
                config_writer.attribute(null, "configDir", new File(niOutDir, "data").getAbsolutePath());
                config_writer.endTag(null, "Module");
            }

            config_writer.endTag(null, "Modules");
            config_writer.endDocument();
        } catch (IOException ioe) {
            Log.e(TAG, Log.getStackTraceString(ioe));
            throw new RuntimeException(ioe);
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        UsbHelper.useContext(getApplicationContext());

        String lib_dir;

        try {
            lib_dir = getPackageManager().getPackageInfo(getPackageName(), 0).applicationInfo.nativeLibraryDir;
        } catch (PackageManager.NameNotFoundException e) {
            throw new RuntimeException(e); // this can't really happen
        }

        File ni_out_dir = getDir("ni", MODE_PRIVATE);

        extractAssets(ni_out_dir);
        writeModulesXml(ni_out_dir, new String[] { "XnDeviceSensorV2" }, new File(lib_dir));

        setenv("OPEN_NI_INSTALL_PATH", ni_out_dir.getAbsolutePath());

        for (String dep_name : new String[]{"XnCore", "XnFormats", "XnDDK"})
            System.loadLibrary(dep_name);
    }

    private static native void setenv(String name, String value);
}
