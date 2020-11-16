set(SUBSYS_NAME benchmarks)
set(SUBSYS_DESC "Point cloud library benchmarks")
set(SUBSYS_DEPS common features search kdtree io)
set(DEFAULT OFF)
set(build TRUE)
set(REASON "Disabled by default")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
if(NOT build)
  return()
endif()

find_package(benchmark REQUIRED)
add_custom_target(run_benchmarks)

macro(PCL_ADD_BENCHMARK _name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs ARGUMENTS LINK_WITH)
  cmake_parse_arguments(PCL_ADD_BENCHMARK "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_executable(benchmark_${_name} ${_name}.cpp)
  set_target_properties(benchmark_${_name} PROPERTIES FOLDER "Benchmarks")
  target_link_libraries(benchmark_${_name} benchmark::benchmark ${PCL_ADD_BENCHMARK_LINK_WITH})
  set_target_properties(benchmark_${_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  
  #Only applies to MSVC
  if(MSVC)
    #Requires CMAKE version 3.13.0
    if(CMAKE_VERSION VERSION_LESS "3.13.0" AND (NOT ArgumentWarningShown))
      message(WARNING "Arguments for unit test projects are not added - this requires at least CMake 3.13. Can be added manually in \"Project settings -> Debugging -> Command arguments\"")
      SET (ArgumentWarningShown TRUE PARENT_SCOPE)
    else()
      #Only add if there are arguments to test
      if(PCL_ADD_BENCHMARK_ARGUMENTS)
        string (REPLACE ";" " " PCL_ADD_BENCHMARK_ARGUMENTS_STR "${PCL_ADD_BENCHMARK_ARGUMENTS}")
        set_target_properties(benchmark_${_name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS ${PCL_ADD_BENCHMARK_ARGUMENTS_STR})
      endif()
    endif()
  endif()
  
  add_custom_target(run_benchmark_${_name} benchmark_${_name} ${PCL_ADD_BENCHMARK_ARGUMENTS})
  set_target_properties(run_benchmark_${_name} PROPERTIES FOLDER "Benchmarks")
  
  add_dependencies(run_benchmarks run_benchmark_${_name})
endmacro()

PCL_ADD_BENCHMARK(features LINK_WITH pcl_io pcl_search pcl_features ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd")
