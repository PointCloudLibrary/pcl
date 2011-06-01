set(SUBSYS_NAME registration)
set(SUBSYS_DESC "Point cloud registration library")
set(SUBSYS_DEPS common io kdtree sample_consensus)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(incs 
        include/pcl/${SUBSYS_NAME}/correspondence_estimation.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_distance.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_one_to_one.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_reciprocal.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_sample_consensus.h
        include/pcl/${SUBSYS_NAME}/correspondence_rejection_trimmed.h
        include/pcl/${SUBSYS_NAME}/correspondence_sorting.h
        include/pcl/${SUBSYS_NAME}/correspondence_types.h
        include/pcl/${SUBSYS_NAME}/ia_ransac.h
        include/pcl/${SUBSYS_NAME}/icp.h
        include/pcl/${SUBSYS_NAME}/icp_nl.h
        include/pcl/${SUBSYS_NAME}/registration.h
        include/pcl/${SUBSYS_NAME}/transformation_estimation.h
        include/pcl/${SUBSYS_NAME}/transformation_estimation_svd.h
        include/pcl/${SUBSYS_NAME}/transforms.h
        )


    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/correspondence_estimation.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_distance.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_one_to_one.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_reciprocal.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_sample_consensus.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_rejection_trimmed.hpp
        include/pcl/${SUBSYS_NAME}/impl/correspondence_types.hpp
        include/pcl/${SUBSYS_NAME}/impl/ia_ransac.hpp
        include/pcl/${SUBSYS_NAME}/impl/icp.hpp
        include/pcl/${SUBSYS_NAME}/impl/icp_nl.hpp
        include/pcl/${SUBSYS_NAME}/impl/registration.hpp
        include/pcl/${SUBSYS_NAME}/impl/transformation_estimation_svd.hpp
        include/pcl/${SUBSYS_NAME}/impl/transforms.hpp
        )

    set(srcs src/registration.cpp)

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_kdtree pcl_sample_consensus)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
                       "${EXT_DEPS}" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)
