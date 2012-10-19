# Build with android-cmake!

cmake_minimum_required(VERSION 2.8)
project(AndroidPeopleLibrary)

function(embed_text_file name cpp_name_var)
  set(cpp_name ${CMAKE_CURRENT_BINARY_DIR}/${name}.cpp)
  set(${cpp_name_var} ${cpp_name} PARENT_SCOPE)
  string(REPLACE . _ array_name_tail ${name})
  add_custom_command(
    OUTPUT ${cpp_name}
    COMMAND bin2c -p 0 -n source_${array_name_tail} ${name} > ${cpp_name}
    MAIN_DEPENDENCY ${name}
  )
endfunction()

embed_text_file(generic.vsh generic_vsh_cpp)
embed_text_file(tree_walk.fsh tree_walk_fsh_cpp)
embed_text_file(consensus.fsh consensus_fsh_cpp)

add_library(people_demo SHARED
  com_itseez_peopledemo_RGBDImage.cpp
  com_itseez_peopledemo_BodyPartsRecognizer.cpp
  rgbd_image.cpp body_parts_recognizer.cpp stopwatch.cpp
  gles_helper.cpp
  ${generic_vsh_cpp} ${tree_walk_fsh_cpp} ${consensus_fsh_cpp}
)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_path(TBB_INCLUDE_DIR tbb/tbb.h)
find_library(TBB_LIBRARY tbb)
include_directories(${TBB_INCLUDE_DIR})

target_link_libraries(people_demo log EGL GLESv2 ${TBB_LIBRARY})

if(CMAKE_COMPILER_IS_GNUCXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -funroll-loops")
endif()
