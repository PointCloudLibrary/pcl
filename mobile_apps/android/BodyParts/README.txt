This is a demo app that uses a random decision forest to recognize human
body parts on a range image. It can either read from prerecorded images or
from a Kinect.

In the tools directory you will find two programs (build them normally with
CMake):

* treetxt2bin - converts the random trees from /gpu/people/data/results into
  a compact binary format.
* pcd2rgbd - converts a cloud in PCD format into a (slightly more) compact
  binary format.

Before building the demo, you have to convert the trees with treetxt2bin
and put them into either the assets/trees directory here, or the trees
directory on your device's storage. The former option is more convenient for
distribution, the latter for development.

To view prerecorded images, convert them with pcd2rgbd and put them in the
rgbd directory on your device's storage.

To build the demo, you will need libusb, OpenNI, Boost and TBB. To build
them, go to /3rdparty/android and run the provided scripts:

./{libusb,openni,boost,tbb}-{fetch,build}.sh

After that, run the following in this directory (<pcl> means the full path to the PCL sources):

export NDK_MODULE_PATH=<pcl>/3rdparty/android/ndk-modules
ndk-build
android update project -p . -n BodyParts \
  -l ../../../3rdparty/android/libusb/android/LibusbSupport
ant debug
