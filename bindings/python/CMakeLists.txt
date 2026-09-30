cmake_minimum_required(VERSION 3.5)
project(bindings)

# find_package(PCL REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)

# We can replace `find_package` with `add_subdirectory`, depending on usage.
# https://pybind11.readthedocs.io/en/stable/compiling.html#find-package-vs-add-subdirectory 
find_package(pybind11)

pybind11_add_module(pcl ${CMAKE_CURRENT_SOURCE_DIR}/pybind11-gen/common/include/pcl/impl/point_types.cpp)

target_link_libraries(pcl PRIVATE ${PCL_LIBRARIES})
# add_dependencies(pcl_demo some_other_target)
