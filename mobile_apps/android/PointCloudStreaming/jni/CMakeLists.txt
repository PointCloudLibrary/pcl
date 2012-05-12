set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC" )


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(VES REQUIRED)
include_directories(${VES_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(ndksource_dir ${ANDROID_NDK}/sources/android/native_app_glue)
include_directories(${ndksource_dir})

set(sources
  ${ndksource_dir}/android_native_app_glue.c
  PointCloudStreaming.cpp
)

set(deps
  GLESv2
  EGL
  android
  log
  ${VES_LIBRARIES}
)

add_library(PointCloudStreaming SHARED ${sources})
target_link_libraries(PointCloudStreaming ${deps})
