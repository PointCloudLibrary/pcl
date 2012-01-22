cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iccv11)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iccv2011_capture_tool src/capture_tool.cpp src/openni_capture.cpp)
target_link_libraries (iccv2011_capture_tool ${PCL_LIBRARIES})

add_executable (iccv2011_correspondence_viewer src/correspondence_viewer.cpp)
target_link_libraries (iccv2011_correspondence_viewer ${PCL_LIBRARIES})

add_executable (iccv2011_test_filters src/test_filters.cpp)
target_link_libraries (iccv2011_test_filters ${PCL_LIBRARIES})

add_executable (iccv2011_test_segmentation src/test_segmentation.cpp)
target_link_libraries (iccv2011_test_segmentation ${PCL_LIBRARIES})

add_executable (iccv2011_test_feature_estimation src/test_feature_estimation.cpp)
target_link_libraries (iccv2011_test_feature_estimation ${PCL_LIBRARIES})

add_executable (iccv2011_test_registration src/test_registration.cpp)
target_link_libraries (iccv2011_test_registration ${PCL_LIBRARIES})

#add_executable (iccv2011_test_surface src/test_surface.cpp)
#target_link_libraries (iccv2011_test_surface ${PCL_LIBRARIES})

add_executable (iccv2011_test_object_recognition src/test_object_recognition.cpp)
target_link_libraries (iccv2011_test_object_recognition ${PCL_LIBRARIES})

add_executable (iccv2011_build_object_model src/build_object_model.cpp)
target_link_libraries (iccv2011_build_object_model ${PCL_LIBRARIES})

add_executable (iccv2011_build_all_object_models src/build_all_object_models.cpp)
target_link_libraries (iccv2011_build_all_object_models ${PCL_LIBRARIES})

add_executable (iccv2011_tutorial src/tutorial.cpp)
target_link_libraries (iccv2011_tutorial ${PCL_LIBRARIES})
