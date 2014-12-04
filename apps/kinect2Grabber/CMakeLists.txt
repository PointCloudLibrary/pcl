set(SUBSUBSYS_NAME kinect2grabber)
set(SUBSUBSYS_DESC "kinect2grabber")
set(SUBSUBSYS_DEPS     common     features     io     kdtree apps)
set(SUBSUBSYS_LIBS pcl_common pcl_features pcl_io pcl_kdtree)

cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)


pcl_subsubsys_option(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
pcl_subsubsys_depend(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

if(build)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package( OpenCV REQUIRED )

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${GLEW_INCLUDE_DIRS})
include_directories( ${GLEWF_INCLUDE_DIRS})

add_definitions(-DGLEW_MX -DGLEW_STATIC)

add_executable(Kinect2Grabber test.cpp )

target_link_libraries(Kinect2Grabber ${OpenCV_LIBS} ${LIBFREENECT2_LIB} ${GLEW_LIB} ${GLFW_LIB} ${PCL_LIBRARIES})

endif()