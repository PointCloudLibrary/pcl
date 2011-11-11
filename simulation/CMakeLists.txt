set(SUBSYS_NAME simulation)
set(SUBSYS_DESC "Point Cloud Library Simulation")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS openni vtk)

if(build)


    set(srcs 
        src/pcd_io.cpp
        src/vtk_io.cpp
        src/ply_io.cpp
        src/compression.cpp
        src/lzf.cpp
        src/obj_io.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/file_io.h
        include/pcl/${SUBSYS_NAME}/lzf.h
        include/pcl/${SUBSYS_NAME}/io.h
        include/pcl/${SUBSYS_NAME}/pcd_io.h
        include/pcl/${SUBSYS_NAME}/pcl_io_exception.h
        include/pcl/${SUBSYS_NAME}/vtk_io.h
        include/pcl/${SUBSYS_NAME}/ply_io.h
        include/pcl/${SUBSYS_NAME}/ply.h
        include/pcl/${SUBSYS_NAME}/obj_io.h 
        ${VTK_IO_INCLUDES}
        ${OPENNI_GRABBER_INCLUDES}
        )


    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/pcd_io.hpp
       )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRECTORIES})
    link_directories(${VTK_LINK_DIRECTORIES})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${compression_incs} ${impl_incs} ${OPENNI_INCLUDES})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common ${VTK_IO_TARGET_LINK_LIBRARIES})
    if(OPENNI_FOUND)
      target_link_libraries(${LIB_NAME} ${OPENNI_LIBRARIES})
    endif(OPENNI_FOUND)

    set(EXT_DEPS eigen3)
    if(OPENNI_FOUND)
      list(APPEND EXT_DEPS openni-dev)
    endif(OPENNI_FOUND)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} compression ${compression_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/openni_camera ${OPENNI_INCLUDES})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(tools)
    add_subdirectory(test)
endif(build)

