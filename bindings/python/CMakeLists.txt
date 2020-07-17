cmake_minimum_required(VERSION 3.5)
project(bindings)

# find_package(PCL REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pybind11)

pybind11_add_module(pcl /home/divyanshu/Projects/active/pcl/bindings/python/pybind11/common/include/pcl/impl/point_types.cpp)

target_link_libraries(pcl PRIVATE ${PCL_LIBRARIES})
# add_dependencies(pcl_demo some_other_target)
