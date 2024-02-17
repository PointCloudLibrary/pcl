#Helper library
set(SUBSYS_NAME io_ply)
set(SUBSYS_DESC "Point cloud IO ply library")
set(SUBSYS_DEPS common)
set(SUBSYS_EXT_DEPS boost eigen3)

set(PLY_SOURCES src/ply_parser.cpp)
set(PLY_INCLUDES
  "include/pcl/io_ply/byte_order.h"
  "include/pcl/io_ply/io_operators.h"
  "include/pcl/io_ply/ply.h"
  "include/pcl/io_ply/ply_parser.h"
)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

if(NOT build)
  return()
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${PLY_SOURCES} ${PLY_INCLUDES})
if(MINGW)
  # libws2_32 isn't added by default for MinGW
  target_link_libraries(${LIB_NAME} ws2_32)
endif()
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ply" ${PLY_INCLUDES})
  
target_link_libraries(${LIB_NAME} pcl_common Boost::boost)
