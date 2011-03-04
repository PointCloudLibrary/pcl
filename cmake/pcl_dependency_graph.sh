#!/bin/bash
pwd=`pwd`
cd ../pcl && mkdir -p build && cd build && cmake --graphviz=/tmp/pcl.dot ..
cd $pwd
./graphviz-cleaner.py -u -p pcd -p hdf5 -p topic_tools -p cminpack -p roslib -p cpp -p test -p sensor_msgs -p qhull -p ros -p boost -p XmlRpc -p log4cxx -f /tmp/pcl_cleaned.dot /tmp/pcl.dot 2> /dev/null
dot /tmp/pcl_cleaned.dot -Tpng -o pcl_dependency_graph.png

