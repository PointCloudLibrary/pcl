cmake_minimum_required(VERSION 2.8)

project(HelloPCLWorld)

#notice the "recycling" of CMAKE_C_FLAGS
#this is necessary to pick up android flags
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )

find_package(Boost 1.45.0 REQUIRED COMPONENTS date_time system filesystem thread iostreams)
find_package(PCL REQUIRED COMPONENTS common io octree kdtree features search)
find_package(VTK REQUIRED)

include(${VTK_USE_FILE})
include_directories(${Boost_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}) 
message (STATUS ${CMAKE_MODULE_PATH})
find_package(VES REQUIRED)
find_package(Flann REQUIRED)

include_directories(${VES_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/jni)

message(STATUS VES libraries: ${VES_LIBRARIES})
set(deps GLESv2   EGL   android   log   ${VES_LIBRARIES} )

include_directories( ${PCL_INCLUDE_DIRS} )  

set(the_target KiwiNative)

add_library(${the_target} SHARED jni/KiwiNative.cpp jni/myFunctions.cpp jni/profiler.cpp jni/pcl_demo.cpp)

message(STATUS ${PCL_LIBRARIES})

target_link_libraries(${the_target} ${BOOST_LIBRARIES}
${BOOST_FILESYSTEM_LIBRARY} log  ${FLANN_LIBRARIES} ${PCL_LIBRARIES}  ${deps})


