set(SUBSYS_NAME people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common segmentation search kdtree)

set(build OFF)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

set(CUDA_FIND_QUIETLY TRUE)
  
if(build)
  find_package(CUDA 4.1)

  if(CUDA_FOUND)

    FILE(GLOB incs  trees/include/pcl/${SUBSYS_NAME}/trees/*.h 
                    display/include/pcl/${SUBSYS_NAME}/display/*.h 
                    conversion/include/pcl/${SUBSYS_NAME}/conversion/*.h)    
    FIlE(GLOB srcs  conversion/src/*.cpp 
                    display/src/*.cpp 
                    trees/src/*.cpp)  
    FILE(GLOB srcs_cuda trees/src/kernels/*.cu 
                        trees/src/kernels/*.h)

    set(inc_folders trees/include
                    display/include
                    conversion/include)

    source_group("Source Files\\cuda" FILES ${srcs_cuda} )

    include_directories( ${CUDA_CUT_INCLUDE_DIR} )
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDA_INCLUDE_DIRS} ${inc_folders})

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G -O3 --ptxas-options=-v -arch=sm_11 -Xcompiler -fPIC)
    CUDA_COMPILE(objs_cuda STATIC ${srcs_cuda} ${incs} )

    set(LIB_NAME pcl_${SUBSYS_NAME})

    #add_definitions("-DVERBOSE")    
    #set(impl_incs include/pcl/${SUBSYS_NAME}/conversions/conversions.hpp)

    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${srcs_cuda} ${objs_cuda})
    target_link_libraries(${LIB_NAME} pcl_search pcl_sample_consensus ${CUDA_CUDART_LIBRARY})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
  endif(CUDA_FOUND)

add_subdirectory(apps)

endif(build)
