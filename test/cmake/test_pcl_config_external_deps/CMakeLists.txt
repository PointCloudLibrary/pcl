# Consumer project used by the cmake_pcl_config_surface_external_deps test
# (see ../CMakeLists.txt). It reproduces issue #6522: linking only
# pcl_surface must still expose QHULL::QHULL and FLANN::FLANN on the link
# interface, otherwise $<TARGET_RUNTIME_DLLS:app> omits qhull_r.dll and
# flann_cpp.dll and a deployed app exits 0xC0000135 (STATUS_DLL_NOT_FOUND).
cmake_minimum_required(VERSION 3.16.3)
project(pcl_external_deps_repro LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED COMPONENTS surface)

add_executable(app main.cpp)
target_link_libraries(app PRIVATE pcl_surface)

get_target_property(_surface_libs pcl_surface INTERFACE_LINK_LIBRARIES)
message(STATUS "pcl_surface INTERFACE_LINK_LIBRARIES = ${_surface_libs}")
get_target_property(_kdtree_libs pcl_kdtree INTERFACE_LINK_LIBRARIES)
message(STATUS "pcl_kdtree INTERFACE_LINK_LIBRARIES = ${_kdtree_libs}")
get_target_property(_search_libs pcl_search INTERFACE_LINK_LIBRARIES)
message(STATUS "pcl_search INTERFACE_LINK_LIBRARIES = ${_search_libs}")

# QHULL is an optional dependency of surface itself, so it must be on
# pcl_surface's own link interface.
if(NOT TARGET QHULL::QHULL)
  message(FATAL_ERROR "QHULL::QHULL target not found in consumer build (see #6522)")
endif()
if(NOT _surface_libs MATCHES "QHULL::QHULL")
  message(FATAL_ERROR "QHULL::QHULL missing from pcl_surface INTERFACE_LINK_LIBRARIES (see #6522)")
endif()
# FLANN reaches surface transitively through pcl_kdtree and pcl_search, so
# it must be on their link interfaces.
if(NOT TARGET FLANN::FLANN)
  message(FATAL_ERROR "FLANN::FLANN target not found in consumer build (see #6522)")
endif()
if(NOT _kdtree_libs MATCHES "FLANN::FLANN")
  message(FATAL_ERROR "FLANN::FLANN missing from pcl_kdtree INTERFACE_LINK_LIBRARIES (see #6522)")
endif()
if(NOT TARGET FLANN::FLANN)
  message(FATAL_ERROR "FLANN::FLANN target not found in consumer build (see #6522)")
endif()
if(NOT _search_libs MATCHES "FLANN::FLANN")
  message(FATAL_ERROR "FLANN::FLANN missing from pcl_search INTERFACE_LINK_LIBRARIES (see #6522)")
endif()

message(STATUS "OK: QHULL::QHULL and FLANN::FLANN present on the component link interfaces")
