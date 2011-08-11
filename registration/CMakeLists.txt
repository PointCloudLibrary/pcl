set(SUBSYS_NAME registration)
set(SUBSYS_DESC "Point cloud registration library")
set(SUBSYS_DEPS common io kdtree sample_consensus features)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(incs 
        include/pcl/${SUBSYS_NAME}/correspondence_estimation.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_distance.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_features.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_one_to_one.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_sample_consensus.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_trimmed.h
        include/pcl/${SUBSYS_NAME}/correspondence_sorting.h
        include/pcl/${SUBSYS_NAME}/correspondence_types.h
        include/pcl/${SUBSYS_NAME}/ia_ransac.h
        include/pcl/${SUBSYS_NAME}/icp.h
        include/pcl/${SUBSYS_NAME}/icp_nl.h
        #include/pcl/${SUBSYS_NAME}/incremental_registration.h
        include/pcl/${SUBSYS_NAME}/ppf_registration.h
        include/pcl/${SUBSYS_NAME}/pyramid_feature_matching.h
        include/pcl/${SUBSYS_NAME}/registration.h
        include/pcl/${SUBSYS_NAME}/transformation_estimation.h
        include/pcl/${SUBSYS_NAME}/transformation_estimation_svd.h
        include/pcl/${SUBSYS_NAME}/transforms.h
        include/pcl/${SUBSYS_NAME}/gicp.h
        )

#    set(stanford_gicp_incs
#        include/pcl/${SUBSYS_NAME}/stanford_gicp/gicp.h
#        include/pcl/${SUBSYS_NAME}/stanford_gicp/optimize.h
#        include/pcl/${SUBSYS_NAME}/stanford_gicp/scan.h
#        include/pcl/${SUBSYS_NAME}/stanford_gicp/transform.h
#        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_distance.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_features.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_one_to_one.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_sample_consensus.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_trimmed.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_types.hpp
        include/pcl/${SUBSYS_NAME}/impl/ia_ransac.hpp
        include/pcl/${SUBSYS_NAME}/impl/icp.hpp
        include/pcl/${SUBSYS_NAME}/impl/icp_nl.hpp
        include/pcl/${SUBSYS_NAME}/impl/ppf_registration.hpp
        include/pcl/${SUBSYS_NAME}/impl/pyramid_feature_matching.hpp
        include/pcl/${SUBSYS_NAME}/impl/registration.hpp
        #include/pcl/${SUBSYS_NAME}/impl/stanford_gicp.hpp
        include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_svd.hpp
        include/pcl/${SUBSYS_NAME}/impl/transforms.hpp
        include/pcl/${SUBSYS_NAME}/impl/gicp.hpp
        )

    set(srcs
      src/registration.cpp
      #src/stanford_gicp/bfgs_funcs.cpp
      #src/stanford_gicp/gicp.cpp
      #src/stanford_gicp/optimize.cpp
      #src/stanford_gicp/scan.cpp
      #src/stanford_gicp/transform.cpp
      src/ppf_registration.cpp
      src/pyramid_feature_matching.cpp)

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_kdtree pcl_sample_consensus pcl_features)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/stanford_gicp ${stanford_gicp_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(tools)
endif(build)
