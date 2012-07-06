#!/bin/bash

set -x

appDir=$(pwd)
superbuildDir=$(cd $(pwd)/../../../../../.. && echo $(pwd))
buildDir=${superbuildDir}/CMakeExternals/Build/HelloPCL
target=device

mkdir -p ${buildDir}
cd ${buildDir}

cmake \
-G Xcode \
-DSUPERBUILD_DIR=${superbuildDir} \
-DVES_DIR=${superbuildDir}/CMakeExternals/Build/ves-ios-$target \
-DVTK_DIR=${superbuildDir}/CMakeExternals/Build/vtk-ios-$target \
-DPCL_DIR=${superbuildDir}/CMakeExternals/Install/pcl-ios-$target/share/pcl-1.6 \
-DEIGEN_INCLUDE_DIRS:PATH=${superbuildDir}/CMakeExternals/Install/eigen  \
${appDir}
