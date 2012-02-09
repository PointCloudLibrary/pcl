cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE( CUDA REQUIRED )

#############################
# The Kernel sources
message(STATUS ${CUDA_CUT_INCLUDE_DIR})
SET( SRC_KERNEL src/kernels/CUDA_runTree.cu 
                src/kernels/CUDA_runMultiTree.cu)
INCLUDE_DIRECTORIES( ${CUDA_CUT_INCLUDE_DIR} )
SET( CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G -O3 --ptxas-options=-v -arch=sm_11 -Xcompiler -fPIC)
CUDA_COMPILE(COMPILED_KERNELS STATIC ${SRC_KERNEL} )

add_definitions("-DVERBOSE")

#############################
# The sources
set( SRC src/CUDATree.cpp
         src/TreeLiveProc.cpp
         src/MultiTreeLiveProc.cpp
         src/colorLMap.cpp)

#############################
# The compilation
INCLUDE_DIRECTORIES( include  ${CUDA_INCLUDE_DIRS})
LINK_LIBRARIES( ${CUDA_CUDART_LIBRARY} )
ADD_LIBRARY( TreeLive SHARED ${SRC} ${COMPILED_KERNELS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
