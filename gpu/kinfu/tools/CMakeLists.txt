set(SUBSUBSYS_NAME tools)
set(SUBSUBSYS_DESC "Kinfu tools")
set(SUBSUBSYS_DEPS gpu_kinfu visualization)
set(SUBSUBSYS_OPT_DEPS opencv)
set(EXT_DEPS openni)
set(DEFAULT TRUE)
set(REASON "")

PCL_SUBSUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSUBSYS_NAME} ${SUBSUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build NAME ${SUBSUBSYS_NAME} PARENT_NAME ${SUBSYS_NAME} DEPS ${SUBSUBSYS_DEPS} OPT_DEPS ${SUBSUBSYS_OPT_DEPS} EXT_DEPS ${EXT_DEPS})

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB hdrs "*.h*")
include_directories(SYSTEM ${OPENNI_INCLUDE_DIRS})

## KINECT FUSION
set(the_target pcl_kinfu_app)
set(srcs kinfu_app.cpp capture.cpp evaluation.cpp)

source_group("Source Files" FILES ${srcs})

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu)

if(OpenCV_FOUND)
  target_link_libraries("${the_target}" ${OpenCV_LIBS})
endif()

## RECORD TSDFVOLUME
set(the_target pcl_record_tsdfvolume)
set(srcs record_tsdfvolume.cpp capture.cpp)

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} ${hdrs})
target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu)
