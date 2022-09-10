# TODO maybe also rolling and latest?
FROM "ubuntu:devel"

# TODO PCL_INDEX_SIZE and PCL_INDEX_SIGNED
# TODO test more versions of cmake, boost, vtk, eigen, qt, maybe flann, maybe other compilers?
ARG VTK_VERSION
ARG CMAKE_CXX_STANDARD
ARG CMAKE_BUILD_TYPE
ARG COMPILER_PACKAGE
ARG CMAKE_C_COMPILER
ARG CMAKE_CXX_COMPILER
RUN echo VTK_VERSION=${VTK_VERSION} CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} COMPILER_PACKAGE=${COMPILER_PACKAGE} CMAKE_C_COMPILER=${CMAKE_C_COMPILER} CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y git cmake ${COMPILER_PACKAGE}
RUN apt install -y libeigen3-dev libflann-dev
RUN apt install -y libboost-filesystem-dev libboost-date-time-dev libboost-iostreams-dev
RUN apt install -y libgtest-dev libbenchmark-dev
RUN apt install -y qtbase5-dev libvtk${VTK_VERSION}-dev libvtk${VTK_VERSION}-qt-dev

RUN cd \
 && git clone --depth=1 https://github.com/PointCloudLibrary/pcl \
 && mkdir pcl/build \
 && cd pcl/build \
 && cmake .. -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DBUILD_simulation=ON -DBUILD_surface_on_nurbs=ON -DBUILD_global_tests=ON -DBUILD_benchmarks=ON -DBUILD_examples=ON -DBUILD_tools=ON -DBUILD_apps=ON -DBUILD_apps_3d_rec_framework=ON -DBUILD_apps_cloud_composer=ON -DBUILD_apps_in_hand_scanner=ON -DBUILD_apps_modeler=ON -DBUILD_apps_point_cloud_editor=ON \
 && cmake --build . -- -j2 -k \
 && cmake --build . -- tests
# TODO maybe also build tutorials?
