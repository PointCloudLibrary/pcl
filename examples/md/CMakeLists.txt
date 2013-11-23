cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(md)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )

add_executable (md src/md.cpp src/imageFromCloud.cpp src/CTCDriver src/processFeatures2D.cpp src/processFeatures3D.cpp src/modelQuery2D.cpp src/multiDescriptorVoting.cpp src/descriptorVoteProcessing.cpp src/utilityFuncs3D.cpp src/modelQuery3D.cpp )
target_link_libraries (md ${OpenCV_LIBS} ${PCL_LIBRARIES})
