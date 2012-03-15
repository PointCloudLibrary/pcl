set(SUBSYS_NAME people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common gpu segmentation search kdtree)

set(build OFF)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

set(CUDA_FIND_QUIETLY TRUE)
FIND_PACKAGE( CUDA REQUIRED )
  
if(build)
  if(CUDA_FOUND)

    #############################
    # The Kernel sources
    message(STATUS ${CUDA_CUT_INCLUDE_DIR})
    set(srcs_kernel trees/src/kernels/CUDA_run_tree.cu 
                    trees/src/kernels/CUDA_run_multi_tree.cu)
    INCLUDE_DIRECTORIES( ${CUDA_CUT_INCLUDE_DIR} )
    SET( CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G -O3 --ptxas-options=-v -arch=sm_11 -Xcompiler -fPIC)
    CUDA_COMPILE(COMPILED_KERNELS STATIC ${srcs_kernel} )

    #add_definitions("-DVERBOSE")

    set(srcs  conversion/src/conversions.cpp
              display/src/colorLMap.cpp
              trees/src/CUDA_tree.cpp
              trees/src/tree_live_proc.cpp
              trees/src/multi_tree_live_proc.cpp
        )

    set(incs  conversion/include/pcl/${SUBSYS_NAME}/conversion/conversions.h
              display/include/pcl/${SUBSYS_NAME}/display/colormap.h
              trees/include/pcl/${SUBSYS_NAME}/trees/handle_error.h
              trees/include/pcl/${SUBSYS_NAME}/trees/option_parser.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tex_fetch.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_io.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_live.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_run.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_train.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_train_histogram.h
              trees/include/pcl/${SUBSYS_NAME}/trees/tree_train_io.h
        )

    #set(impl_incs include/pcl/${SUBSYS_NAME}/conversions/conversions.hpp
    #    )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDA_INCLUDE_DIRS})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${COMPILED_KERNELS})
    target_link_libraries(${LIB_NAME} pcl_search pcl_sample_consensus ${CUDA_CUDART_LIBRARY})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
  endif(CUDA_FOUND)
endif(build)
