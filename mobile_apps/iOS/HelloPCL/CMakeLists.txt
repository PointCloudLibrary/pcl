cmake_minimum_required(VERSION 2.8.5)
project(HelloPCL)

set(CMAKE_OSX_SYSROOT iphoneos5.0)
set(CMAKE_OSX_ARCHITECTURES "armv7")
set(CMAKE_XCODE_EFFECTIVE_PLATFORMS "-iphoneos;-iphonesimulator")


set(CMAKE_FIND_ROOT_PATH ${CMAKE_OSX_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


if(NOT DEFINED SUPERBUILD_DIR)
  message(FATAL_ERROR "Please set the cmake variable SUPERBUILD_DIR")
endif()

set(cmakeexternals ${SUPERBUILD_DIR}/CMakeExternals)
if(NOT IS_DIRECTORY ${cmakeexternals})
  message(FATAL_ERROR "The directory ${cmakeexternals} does not exist.  Check the value of SUPERBUILD_DIR.")
endif()

list(APPEND CMAKE_FIND_ROOT_PATH
  ${cmakeexternals}/Install/boost-ios-device
  ${cmakeexternals}/Install/flann-ios-device
  ${cmakeexternals}/Install/pcl-ios-device)


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(VES REQUIRED)
include_directories(${VES_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(M_SRCS
  EAGLView.mm
  ES2Renderer.mm
  GLViewController.m
  InfoView.m
  LoadDataController.m
  TitleBarViewContainer.m
  kiwiAppDelegate.mm
  main.m
  )

set(cpp_sources
  vesKiwiPCLDemo.cpp
  vesKiwiPCLDemo.h
  vesKiwiPCLApp.h

  vtkPCLConversions.cxx
  vtkPCLConversions.h
  vtkPCLSACSegmentationPlane.cxx
  vtkPCLSACSegmentationPlane.h
  vtkPCLVoxelGrid.cxx
  vtkPCLVoxelGrid.h
  )

set(HEADERS
  EAGLView.h
  ES2Renderer.h
  GLViewController.h
  InfoView.h
  Kiwi-Prefix.pch
  LoadDataController.h
  TitleBarViewContainer.h
  kiwiAppDelegate.h
  )

set(XIB_FILES
  InfoView.xib
  MainWindow.xib
  TitleBarViewContainer.xib
)

set(RESOURCES
  ${XIB_FILES}

  images/Kiwi_K.png
  images/fullscreen_32.png
  images/opendata_32.png
  images/Default-Landscape~ipad.png
  images/Default-Portrait~ipad.png
  images/kiwi_icon_57.png
  images/kiwi_icon_72.png
  images/kiwi_icon_114.png

  data/pointcloud.pcd
  )

add_executable(HelloPCL MACOSX_BUNDLE ${M_SRCS} ${cpp_sources} ${HEADERS} ${RESOURCES})

target_link_libraries(HelloPCL
  ${VES_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARY}
  ${PCL_SAMPLE_CONSENSUS_LIBRARY}
  ${PCL_FILTERS_LIBRARY}
  ${PCL_KDTREE_LIBRARY}
  ${PCL_IO_LIBRARY}
  ${PCL_COMMON_LIBRARY}
  ${BOOST_LIBRARIES}
  "-framework CoreGraphics"
  "-framework Foundation"
  "-framework UIKit"
  "-framework OpenGLES"
  "-framework QuartzCore"
  )

set_target_properties(HelloPCL PROPERTIES
  MACOSX_BUNDLE_GUI_IDENTIFIER "com.kitware.HelloPCL"
  MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Kiwi-Info.plist
  RESOURCE "${RESOURCES}"
  XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
  XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf-with-dsym"
  XCODE_ATTRIBUTE_GCC_PRECOMPILE_PREFIX_HEADER YES
  XCODE_ATTRIBUTE_GCC_PREFIX_HEADER ${CMAKE_CURRENT_LIST_DIR}/Kiwi-Prefix.pch
  XCODE_ATTRIBUTE_INFOPLIST_PREPROCESS YES
  XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET 5.0
  )
