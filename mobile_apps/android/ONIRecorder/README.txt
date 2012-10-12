This is a demo app that grabs and displays (and optionally records) images
from the Kinect.

To build, you will need special versions of libusb and OpenNI, available in
/3rdparty/libusb-android and /3rdparty/openni/android.

* Fetch and build libusb using the provided scripts fetch.sh and build.sh.
* Fetch OpenNI and its Sensor module using fetch-openni.sh and
  fetch-sensor.sh. You don't need to build these.
* Run the following (<directory> means the full path to that directory):

export NDK_MODULE_PATH=<libusb>/android/ndk-modules:<OpenNI>/Platforms/Android/jni:<OpenNI>/Platforms/Android/jni/Modules:<Sensor>/Platforms/Android/jni
ndk-build
android update project -p . -n ONIRecorder -l ../../../3rdparty/openni/android/OpenNI/Wrappers/OpenNI.java
ant debug
