set(SUBSYS_NAME 2d)
set(SUBSYS_DESC "Point cloud 2d")
set(SUBSYS_DEPS common io filters)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS vtk)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)

    set(srcs
        src/convolution_2d.cpp 
        )

    set(incs
        "include/pcl/${SUBSYS_NAME}/convolution.h"
        "include/pcl/${SUBSYS_NAME}/kernel.h"
        "include/pcl/${SUBSYS_NAME}/edge.h"
        "include/pcl/${SUBSYS_NAME}/morphology.h"
        )

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/convolution.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/edge.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/morphology.hpp"
        )

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    
    if(${VTK_FOUND})
        set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
        include("${VTK_USE_FILE}")
        set(VTK_IO_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets vtkIO vtkImaging)
    endif(${VTK_FOUND})

    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" ${VTK_INCLUDE_DIRECTORIES})
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    link_directories(${VTK_LINK_DIRECTORIES})
    target_link_libraries("${LIB_NAME}" ${VTK_LIBRARIES} pcl_io)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    #Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
    
endif(build)
