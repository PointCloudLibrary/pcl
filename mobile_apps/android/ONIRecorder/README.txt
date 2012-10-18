This is a demo app that grabs and displays (and optionally records) images
from the Kinect.

To build, you will need special versions of libusb and OpenNI. To build
them, go to /3rdparty/android and run the provided scripts:

./libusb-fetch.sh
./libusb-build.sh
./openni-fetch.sh
./openni-build.sh

After that, run the following in this directory (<pcl> means the full path to the PCL sources):

export NDK_MODULE_PATH=<pcl>/3rdparty/android/ndk-modules
ndk-build
android update project -p . -n ONIRecorder \
  -l ../../../3rdparty/android/OpenNI/Wrappers/OpenNI.java \
  -l ../../../3rdparty/android/libusb/android/LibusbSupport
ant debug
