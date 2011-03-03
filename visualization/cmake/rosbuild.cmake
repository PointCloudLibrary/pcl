# standalone PCL custom rosbuild.cmake

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

include(CheckFunctionExists)
include(AddFileDependencies)

macro(rosbuild_init)
    # Do nothing
endmacro(rosbuild_init)

macro(rosbuild_genmsg)
    # Do nothing
endmacro(rosbuild_genmsg)

macro(rosbuild_add_library lib)
  if (WIN32)
    add_library(${lib} STATIC ${ARGN})
  else (WIN32)
    add_library(${lib} SHARED ${ARGN})
  endif (WIN32)
endmacro(rosbuild_add_library)

macro(rosbuild_add_boost_directories)
    if(WIN32)
        find_package(Boost REQUIRED)
    else(WIN32)
        find_package(Boost COMPONENTS system filesystem)
    endif(WIN32)
    if(NOT Boost_FOUND)
        message(FATAL_ERROR "Could not find Boost libraries.")
    else(NOT Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
        link_directories(${Boost_LIBRARY_DIRS})
    endif(NOT Boost_FOUND)
endmacro(rosbuild_add_boost_directories)

macro(rosbuild_link_boost _target)
    target_link_libraries(${_target} ${Boost_LIBRARIES})
endmacro(rosbuild_link_boost)

macro(rosbuild_check_for_sse)
  # check for SSE extensions
  include(CheckCXXSourceRuns)
  if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
   set(SSE_FLAGS)
  
   set(CMAKE_REQUIRED_FLAGS "-msse3")
   check_cxx_source_runs("
    #include <pmmintrin.h>
  
    int main()
    {
       __m128d a, b;
       double vals[2] = {0};
       a = _mm_loadu_pd(vals);
       b = _mm_hadd_pd(a,a);
       _mm_storeu_pd(vals, b);
       return 0;
    }"
    HAS_SSE3_EXTENSIONS)
  
   set(CMAKE_REQUIRED_FLAGS "-msse2")
   check_cxx_source_runs("
    #include <emmintrin.h>
  
    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)
  
   set(CMAKE_REQUIRED_FLAGS "-msse")
   check_cxx_source_runs("
    #include <xmmintrin.h>
    int main()
    {
        __m128 a, b;
        float vals[4] = {0};
        a = _mm_loadu_ps(vals);
        b = a;
        b = _mm_add_ps(a,b);
        _mm_storeu_ps(vals,b);
        return 0;
    }"
    HAS_SSE_EXTENSIONS)
  
   set(CMAKE_REQUIRED_FLAGS)
  
   if(HAS_SSE3_EXTENSIONS)
    set(SSE_FLAGS "-msse3 -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE3 extensions, using flags: ${SSE_FLAGS}")
   elseif(HAS_SSE2_EXTENSIONS)
    set(SSE_FLAGS "-msse2 -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE2 extensions, using flags: ${SSE_FLAGS}")
   elseif(HAS_SSE_EXTENSIONS)
    set(SSE_FLAGS "-msse -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE extensions, using flags: ${SSE_FLAGS}")
   endif()
  elseif(MSVC)
   check_cxx_source_runs("
    #include <emmintrin.h>
  
    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)
   if( HAS_SSE2_EXTENSIONS )
    message(STATUS "[rosbuild] Found SSE2 extensions")
    set(SSE_FLAGS "/arch:SSE2 /fp:fast -D__SSE__ -D__SSE2__" )
   endif()
  endif()
endmacro(rosbuild_check_for_sse)

macro(rosbuild_add_executable exe)
  add_executable(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  # The SOURCES property seems to be available only since 2.6.  Yar.
  #get_target_property(_srclist ${exe} SOURCES) 
  set(_srclist ${ARGN})
  foreach(_src ${_srclist}) 
    # Handle the case where the second argument is EXCLUDE_FROM_ALL, not a
    # source file.  Only have to do this because we can't get the SOURCES
    # property.
    if(NOT _src STREQUAL EXCLUDE_FROM_ALL)
      set(_file_name _file_name-NOTFOUND)
      find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
      if(NOT _file_name)
        message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
        set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
      endif(NOT _file_name)
      add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST}) 
    endif(NOT _src STREQUAL EXCLUDE_FROM_ALL)
  endforeach(_src)

  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rosbuild_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rosbuild_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rosbuild_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure to do any prebuild work (e.g., msg/srv generation) before
  # building this target.
  #add_dependencies(${exe} rosbuild_precompile)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rosbuild_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")

endmacro(rosbuild_add_executable)

macro(rosbuild_add_compile_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} COMPILE_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        COMPILE_FLAGS "${_flags_str}")
endmacro(rosbuild_add_compile_flags)

macro(rosbuild_add_link_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} LINK_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        LINK_FLAGS "${_flags_str}")
endmacro(rosbuild_add_link_flags)

macro(rosbuild_add_openmp_flags target)
  # Bullseye's wrappers appear to choke on OpenMP pragmas.  So if
  # ROS_TEST_COVERAGE is set (which indicates that we're doing a coverage
  # build with Bullseye), we make this macro a no-op.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    _rosbuild_warn("because ROS_TEST_COVERAGE is set, OpenMP support is disabled")
  else("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
  
  # list of OpenMP flags to check
    set(_rospack_check_openmp_flags
      "-fopenmp" # gcc
      "-openmp" # icc
      "-mp" # SGI & PGI
      "-xopenmp" # Sun
      "-omp" # Tru64
      "-qsmp=omp" # AIX
      )
  
  # backup for a variable we will change
    set(_rospack_openmp_flags_backup ${CMAKE_REQUIRED_FLAGS})
  
  # mark the fact we do not yet know the flag
    set(_rospack_openmp_flag_found FALSE)
    set(_rospack_openmp_flag_value)
  
  # find an OpenMP flag that works
    foreach(_rospack_openmp_test_flag ${_rospack_check_openmp_flags})
      if(NOT _rospack_openmp_flag_found)      
        set(CMAKE_REQUIRED_FLAGS ${_rospack_openmp_test_flag})
        check_function_exists(omp_set_num_threads _rospack_openmp_function_found${_rospack_openmp_test_flag})
  	   
        if(_rospack_openmp_function_found${_rospack_openmp_test_flag})
  	set(_rospack_openmp_flag_value ${_rospack_openmp_test_flag})
  	set(_rospack_openmp_flag_found TRUE)
        endif(_rospack_openmp_function_found${_rospack_openmp_test_flag})
      endif(NOT _rospack_openmp_flag_found)
    endforeach(_rospack_openmp_test_flag ${_rospack_check_openmp_flags})
  
  # restore the CMake variable
    set(CMAKE_REQUIRED_FLAGS ${_rospack_openmp_flags_backup})
    
  # add the flags or warn
    if(_rospack_openmp_flag_found)
      rosbuild_add_compile_flags(${target} ${_rospack_openmp_flag_value})
      rosbuild_add_link_flags(${target} ${_rospack_openmp_flag_value})
    else(_rospack_openmp_flag_found)
      message("WARNING: OpenMP compile flag not found")
    endif(_rospack_openmp_flag_found)

  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
endmacro(rosbuild_add_openmp_flags)

macro(_rosbuild_list_to_string _string _list)
    set(${_string})
    foreach(_item ${_list})
        string(LENGTH "${${_string}}" _len)
        if(${_len} GREATER 0)
          set(${_string} "${${_string}} ${_item}")
        else(${_len} GREATER 0)
          set(${_string} "${_item}")
        endif(${_len} GREATER 0)
    endforeach(_item)
endmacro(_rosbuild_list_to_string)

