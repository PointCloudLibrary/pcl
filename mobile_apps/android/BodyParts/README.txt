This is a demo app that uses a random decision forest to recognize human
body parts on a range image. It can either read from prerecorded images or
from a Kinect.

To build, you will need libusb, OpenNI, Boost and TBB. To build
them, go to /3rdparty/android and run the provided scripts:

./{libusb,openni,boost,tbb}-{fetch,build}.sh

After that, run the following in this directory (<pcl> means the full path to the PCL sources):

export NDK_MODULE_PATH=<pcl>/3rdparty/android/ndk-modules
ndk-build
android update project -p . -n BodyParts \
  -l ../../../3rdparty/android/libusb/android/LibusbSupport
ant debug
