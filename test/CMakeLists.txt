set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global unit tests")

if(BUILD_visualization)
  include("${VTK_USE_FILE}")
  set(SUBSYS_DEPS common sample_consensus io kdtree features filters geometry keypoints search surface registration segmentation octree recognition people outofcore visualization)
  set(OPT_DEPS vtk)
else()
  set(SUBSYS_DEPS common sample_consensus io kdtree features filters geometry keypoints search surface registration segmentation octree recognition people outofcore)
endif()

set(DEFAULT OFF)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(build)

    find_package(Gtest REQUIRED)
    include_directories(SYSTEM ${GTEST_INCLUDE_DIRS} ${GTEST_SRC_DIR})

    add_library(pcl_gtest STATIC ${GTEST_SRC_DIR}/src/gtest-all.cc)
	
    if( MSVC11 ) # VS2012 doesn't correctly support variadic templates yet
        add_definitions("-D_VARIADIC_MAX=10")
    endif()

    enable_testing()
    include_directories(${PCL_INCLUDE_DIRS})

    add_custom_target(tests "${CMAKE_CTEST_COMMAND}" "-V" VERBATIM)

    add_subdirectory(common)
    add_subdirectory(features)
    add_subdirectory(filters)
    add_subdirectory(geometry)
    add_subdirectory(io)
    add_subdirectory(kdtree)
    add_subdirectory(octree)
    add_subdirectory(outofcore)
    add_subdirectory(registration)
    add_subdirectory(search)
    add_subdirectory(keypoints)
    add_subdirectory(surface)
    add_subdirectory(sample_consensus)

    PCL_ADD_TEST(a_bearing_angle_image_test test_bearing_angle_image
                 FILES test_bearing_angle_image.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io)

	  PCL_ADD_TEST(a_recognition_ism_test test_recognition_ism
                 FILES test_recognition_ism.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_features
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/ism_train.pcd" "${PCL_SOURCE_DIR}/test/ism_test.pcd")

    PCL_ADD_TEST(search test_search
                 FILES test_search.cpp
                 LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd")

    PCL_ADD_TEST(a_transforms_test test_transforms
                 FILES test_transforms.cpp
                 LINK_WITH pcl_gtest pcl_io
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
    
    PCL_ADD_TEST(a_segmentation_test test_segmentation
                 FILES test_segmentation.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_common
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/car6.pcd" "${PCL_SOURCE_DIR}/test/colored_cloud.pcd")
    
    PCL_ADD_TEST(test_non_linear test_non_linear
                 FILES test_non_linear.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io pcl_sample_consensus pcl_segmentation pcl_kdtree pcl_search 
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/noisy_slice_displaced.pcd")

    PCL_ADD_TEST(a_recognition_cg_test test_recognition_cg
                 FILES test_recognition_cg.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_features pcl_recognition pcl_keypoints
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/milk.pcd" "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

    PCL_ADD_TEST(a_people_detection_test test_people_detection
                 FILES test_people_groundBasedPeopleDetectionApp.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_search pcl_features pcl_sample_consensus pcl_filters pcl_io pcl_segmentation pcl_people
                 ARGUMENTS "${PCL_SOURCE_DIR}/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml" "${PCL_SOURCE_DIR}/test/five_people.pcd")

    if(BUILD_visualization AND (NOT UNIX OR (UNIX AND DEFINED ENV{DISPLAY})))
        PCL_ADD_TEST(a_visualization_test test_visualization
                 FILES test_visualization.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_visualization pcl_features
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/bunny.pcd")
    endif()

endif(build)
