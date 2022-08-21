set(SUBSUBSYS_NAME tools)
set(SUBSUBSYS_DESC "Kinfu large scale tools")
set(SUBSUBSYS_DEPS gpu_kinfu_large_scale visualization)
set(SUBSUBSYS_OPT_DEPS )
set(EXT_DEPS openni openni2)
set(DEFAULT TRUE)
set(REASON "")

PCL_SUBSUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSUBSYS_NAME} ${SUBSUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build NAME ${SUBSUBSYS_NAME} PARENT_NAME ${SUBSYS_NAME} DEPS ${SUBSUBSYS_DEPS} OPT_DEPS ${SUBSUBSYS_OPT_DEPS} EXT_DEPS ${EXT_DEPS})

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB hdrs "*.h*")
include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

## STANDALONE TEXTURE MAPPING
set(the_target pcl_kinfu_largeScale_texture_output)
set(srcs standalone_texture_mapping.cpp)

source_group("Source Files" FILES ${srcs})

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} BUNDLE)
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_kdtree pcl_features pcl_surface)

## KINECT FUSION
set(the_target pcl_kinfu_largeScale)
set(srcs kinfuLS_app.cpp capture.cpp evaluation.cpp)

source_group("Source Files" FILES ${srcs})

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_octree)

## STANDALONE MARCHING CUBES
set(the_target pcl_kinfu_largeScale_mesh_output)
set(srcs process_kinfu_large_scale_output.cpp)

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_filters)

## RECORD MAPS_RGB
set(the_target pcl_record_kinect_maps_rgb)
set(srcs record_maps_rgb.cpp)

PCL_ADD_EXECUTABLE(${the_target} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${srcs} ${hdrs} BUNDLE)
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_filters)
