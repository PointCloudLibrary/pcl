cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iros11)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (capture_tool src/capture_tool.cpp src/openni_capture.cpp)
target_link_libraries (capture_tool ${PCL_LIBRARIES})

add_executable (test_filters src/test_filters.cpp)
target_link_libraries (test_filters ${PCL_LIBRARIES})

add_executable (test_segmentation src/test_segmentation.cpp)
target_link_libraries (test_segmentation ${PCL_LIBRARIES})

add_executable (test_feature_estimation src/test_feature_estimation.cpp)
target_link_libraries (test_feature_estimation ${PCL_LIBRARIES})

add_executable (test_registration src/test_registration.cpp)
target_link_libraries (test_registration ${PCL_LIBRARIES})

add_executable (test_surface src/test_surface.cpp)
target_link_libraries (test_surface ${PCL_LIBRARIES})

add_executable (test_object_recognition src/test_object_recognition.cpp)
target_link_libraries (test_object_recognition ${PCL_LIBRARIES})

add_executable (build_object_model src/build_object_model.cpp)
target_link_libraries (build_object_model ${PCL_LIBRARIES})

add_executable (build_all_object_models src/build_all_object_models.cpp)
target_link_libraries (build_all_object_models ${PCL_LIBRARIES})

add_executable (correspondence_viewer src/correspondence_viewer.cpp)
target_link_libraries (correspondence_viewer ${PCL_LIBRARIES})