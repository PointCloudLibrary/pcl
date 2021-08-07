set(SUBSYS_NAME cuda)
set(SUBSYS_DESC "Point cloud CUDA libraries")
set(SUBSYS_DEPS)

option(BUILD_CUDA "Build the CUDA-related subsystems" ${DEFAULT})

if(NOT (BUILD_CUDA AND CUDA_FOUND))
  return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  string(REPLACE "-Wold-style-cast" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-Wno-invalid-offsetof" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(APPEND CMAKE_CXX_FLAGS " -Wno-conversion -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable")
  # allow deprecation warnings in Eigen(3.3.7)/Core, see here: https://gitlab.kitware.com/vtk/vtk/-/issues/17661
  string(APPEND CMAKE_CXX_FLAGS " -Wno-error=cpp")
endif()

collect_subproject_directory_names("${CMAKE_CURRENT_SOURCE_DIR}" "CMakeLists.txt" PCL_CUDA_MODULES_NAMES PCL_CUDA_MODULES_DIRS)
set(PCL_CUDA_MODULES_NAMES_UNSORTED ${PCL_CUDA_MODULES_NAMES})
topological_sort(PCL_CUDA_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_CUDA_MODULES_NAMES_UNSORTED PCL_CUDA_MODULES_NAMES PCL_CUDA_MODULES_DIRS)
foreach(subdir ${PCL_CUDA_MODULES_DIRS})
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/${subdir}")
endforeach()
