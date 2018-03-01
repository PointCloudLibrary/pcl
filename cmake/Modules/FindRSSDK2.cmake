###############################################################################
# Find Intel RealSense SDK 2.0 (librealsense)
#
#     find_package(RSSDK2)
#
# Variables defined by this module:
#
#  RSSDK2_FOUND                 True if RealSense SDK 2.0 was found
#  RSSDK2_INCLUDE_DIRS          The location(s) of RealSense SDK 2.0 headers
#  RSSDK2_LIBRARIES             Libraries needed to use RealSense SDK 2.0

find_package(realsense2)

set(RSSDK2_FOUND ${realsense2_FOUND})
set(RSSDK2_INCLUDE_DIRS ${realsense_INCLUDE_DIR})
set(RSSDK2_LIBRARIES ${realsense2_LIBRARY})
