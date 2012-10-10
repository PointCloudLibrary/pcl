#we can use the latest svn code by replacing the source of the build tree

#WARNING: must run the build script before this.
#Also, we must run these by HAND and enable the 2d, feature, etc... options in the manual.
#


####################################
#export ANDROID_NDK=${NDKROOT}

#DO NOT RUN THIS AGAIN 
#cd ../../pcl-superbuild/build/CMakeExternals/Source && mv pcl pcl_old && svn co svn+ssh://svn@svn.pointclouds.org/pcl/trunk pcl && cd -

#cd ../../pcl-superbuild/build/CMakeExternals/Build/pcl-android && make clean && cmake-gui -DBUILD_examples:BOOL="0" . && make install && cd -


########################################
