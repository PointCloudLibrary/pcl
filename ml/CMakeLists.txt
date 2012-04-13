set(SUBSYS_NAME ml)
set(SUBSYS_DESC "Point cloud machine learning library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(incs 
        include/pcl/${SUBSYS_NAME}/dt/decision_tree.h
        include/pcl/${SUBSYS_NAME}/dt/decision_tree_evaluator.h
        include/pcl/${SUBSYS_NAME}/dt/decision_tree_trainer.h
        include/pcl/${SUBSYS_NAME}/feature_handler.h
        include/pcl/${SUBSYS_NAME}/multiple_data_2d_example_index.h
        include/pcl/${SUBSYS_NAME}/point_xy_32i.h
        include/pcl/${SUBSYS_NAME}/regression_variance_stats_estimator.h
        include/pcl/${SUBSYS_NAME}/multi_channel_2d_comparison_feature.h
        include/pcl/${SUBSYS_NAME}/multi_channel_2d_comparison_feature_handler.h
        include/pcl/${SUBSYS_NAME}/multi_channel_2d_data_set.h
        include/pcl/${SUBSYS_NAME}/stats_estimator.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/dt/decision_tree_evaluator.hpp
        include/pcl/${SUBSYS_NAME}/impl/dt/decision_tree_trainer.hpp
        )

    set(srcs
		src/point_xy_32i.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
	SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${LIB_NAME} pcl_common)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

endif(build)
