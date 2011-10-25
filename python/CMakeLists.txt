set(SUBSYS_NAME python)
set(SUBSYS_DESC "Python PCL")
set(SUBSYS_DEPS common filters)

if(PYTHONLIBS_FOUND)
    find_python_module(cheetah REQUIRED)
    if(PY_CHEETAH_FOUND)
        set(DEFAULT TRUE)
        set(REASON "")
        include_directories(${PYTHON_INCLUDE_PATH})
    else(PY_CHEETAH_FOUND)
        set(DEFAULT FALSE)
        set(REASON "Python Cheetah module was not found")
    endif(PY_CHEETAH_FOUND)
else(PYTHONLIBS_FOUND)
    set(DEFAULT FALSE)
    set(REASON "Python libraries were not found")
endif(PYTHONLIBS_FOUND)

set(DEFAULT FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

add_custom_command(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/pygen.i
   COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/gencode.py" "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/gencode.py
   )
if(build)
    set(srcs pcl.cpp)
    set(incs pygen.i)

    set(LIB_NAME pcl)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common ${common_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common/impl ${common_incs_impl})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})
    set_target_properties(pcl PROPERTIES PREFIX "")
    # set_target_properties(pcl PROPERTIES OUTPUT_NAME "cv2")

    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "" "")
endif(build)

