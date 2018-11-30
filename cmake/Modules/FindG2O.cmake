
# Macro to unify finding both the debug and release versions of the libraries
MACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

  find_library("${LIBRARY}_DEBUG"
    NAMES "g2o_${LIBRARYNAME}_d"
    PATHS
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library("${LIBRARY}_DEBUG"
    NAMES "g2o_${LIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  find_library("${LIBRARY}_RELEASE"
    NAMES "g2o_${LIBRARYNAME}"
    PATHS
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library("${LIBRARY}_RELEASE"
    NAMES "g2o_${LIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  if(NOT ${LIBRARY}_DEBUG)
    if(${LIBRARY}_RELEASE)
      set(${LIBRARY}_DEBUG ${${LIBRARY}_RELEASE})
    endif()
  endif()

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(${LIBRARY} ${${LIBRARY}_DEBUG})
  else()
    set(${LIBRARY} ${${LIBRARY}_RELEASE})
  endif()

  mark_as_advanced(FORCE ${LIBRARY}_DEBUG ${LIBRARY}_RELEASE ${LIBRARY})

ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the header files
find_path(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_ROOT}
  $ENV{G2O_ROOT}
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
)

set(G2O_SOLVERS_FOUND FALSE)
set(G2O_FOUND FALSE)
find_package(Eigen 3)
if(Eigen_FOUND AND G2O_INCLUDE_DIR)
  set(G2O_INCLUDE_DIR ${G2O_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
  set(G2O_EXTRA_LIB)
  find_package(SuiteSparse)

  if(CHOLMOD_FOUND)
    set(G2O_INCLUDE_DIR ${G2O_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})
    set(G2O_EXTRA_LIB ${G2O_EXTRA_LIB} ${CHOLMOD_LIBRARIES})
  endif()
  if(CSPARSE_FOUND)
    set(G2O_INCLUDE_DIR ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
    set(G2O_EXTRA_LIB ${G2O_EXTRA_LIB} ${CSPARSE_LIBRARY})
  endif()
  mark_as_advanced(FORCE G2O_INCLUDE_DIR G2O_EXTRA_LIB)

  # Find the core elements
  FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
  FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

  # Find the CLI library
  FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

  # Find the pluggable solvers
  FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
  FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
  FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
  FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
  FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
  FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
  FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)

  # Find the predefined types
  FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
  FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
  FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
  FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
  FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
  FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
  FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

  # G2O solvers declared found if we found at least one solver
  if((G2O_SOLVER_CHOLMOD AND CHOLMOD_FOUND) OR (G2O_SOLVER_CSPARSE AND CSPARSE_FOUND) OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY)
    set(G2O_SOLVERS_FOUND TRUE)
  endif()

  # G2O itself declared found if we found the core libraries and at least one solver
  if(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
    set(G2O_FOUND TRUE)
  endif()
endif()

if(G2O_FOUND)
  if(NOT G2O_FIND_QUIETLY)
    message(STATUS "Found G2O")
  endif()
else()
  if(G2O_FIND_REQUIRED)
    message(FATAL_ERROR "G2O NOT FOUND")
  else()
    if(NOT G2O_FIND_QUIETLY)
      message(STATUS "G2O NOT FOUND")
    endif()
  endif()
endif()
