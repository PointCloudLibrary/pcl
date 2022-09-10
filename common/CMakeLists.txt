set(SUBSYS_NAME common)
set(SUBSYS_DESC "Point cloud common library")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS eigen boost)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(range_image_incs
  include/pcl/range_image/bearing_angle_image.h
  include/pcl/range_image/range_image.h
  include/pcl/range_image/range_image_planar.h
  include/pcl/range_image/range_image_spherical.h
)

set(range_image_incs_impl
  include/pcl/range_image/impl/range_image.hpp
  include/pcl/range_image/impl/range_image_planar.hpp
  include/pcl/range_image/impl/range_image_spherical.hpp
)

set(range_image_srcs
  src/bearing_angle_image.cpp
  src/range_image.cpp
  src/range_image_planar.cpp
)

set(srcs
  src/point_types.cpp
  src/pcl_base.cpp
  src/PCLPointCloud2.cpp
  src/io.cpp
  src/common.cpp
  src/correspondence.cpp
  src/distances.cpp
  src/parse.cpp
  src/poses_from_matches.cpp
  src/print.cpp
  src/projection_matrix.cpp
  src/time_trigger.cpp
  src/gaussian.cpp
  src/colors.cpp
  src/feature_histogram.cpp
  ${range_image_srcs}
)

set(incs
  include/pcl/correspondence.h
  include/pcl/memory.h
  include/pcl/exceptions.h
  include/pcl/pcl_base.h
  include/pcl/pcl_exports.h
  include/pcl/pcl_macros.h
  include/pcl/types.h
  include/pcl/point_cloud.h
  include/pcl/point_struct_traits.h
  include/pcl/point_traits.h
  include/pcl/type_traits.h
  include/pcl/point_types_conversion.h
  include/pcl/point_representation.h
  include/pcl/point_types.h
  include/pcl/for_each_type.h
  include/pcl/pcl_tests.h
  include/pcl/cloud_iterator.h
  include/pcl/TextureMesh.h
  include/pcl/sse.h
  include/pcl/PCLPointField.h
  include/pcl/PCLPointCloud2.h
  include/pcl/PCLImage.h
  include/pcl/PCLHeader.h
  include/pcl/ModelCoefficients.h
  include/pcl/PolygonMesh.h
  include/pcl/Vertices.h
  include/pcl/PointIndices.h
  include/pcl/register_point_struct.h
  include/pcl/conversions.h
  include/pcl/make_shared.h
)

set(common_incs
  include/pcl/common/boost.h
  include/pcl/common/angles.h
  include/pcl/common/bivariate_polynomial.h
  include/pcl/common/centroid.h
  include/pcl/common/concatenate.h
  include/pcl/common/common.h
  include/pcl/common/common_headers.h
  include/pcl/common/distances.h
  include/pcl/common/eigen.h
  include/pcl/common/copy_point.h
  include/pcl/common/io.h
  include/pcl/common/file_io.h
  include/pcl/common/intersections.h
  include/pcl/common/norms.h
  include/pcl/common/piecewise_linear_function.h
  include/pcl/common/polynomial_calculations.h
  include/pcl/common/poses_from_matches.h
  include/pcl/common/time.h
  include/pcl/common/time_trigger.h
  include/pcl/common/transforms.h
  include/pcl/common/transformation_from_correspondences.h
  include/pcl/common/vector_average.h
  include/pcl/common/pca.h
  include/pcl/common/point_tests.h
  include/pcl/common/synchronizer.h
  include/pcl/common/utils.h
  include/pcl/common/geometry.h
  include/pcl/common/gaussian.h
  include/pcl/common/spring.h
  include/pcl/common/intensity.h
  include/pcl/common/random.h
  include/pcl/common/generate.h
  include/pcl/common/projection_matrix.h
  include/pcl/common/colors.h
  include/pcl/common/feature_histogram.h
)

set(common_incs_impl
  include/pcl/common/impl/angles.hpp
  include/pcl/common/impl/bivariate_polynomial.hpp
  include/pcl/common/impl/centroid.hpp
  include/pcl/common/impl/common.hpp
  include/pcl/common/impl/eigen.hpp
  include/pcl/common/impl/intersections.hpp
  include/pcl/common/impl/copy_point.hpp
  include/pcl/common/impl/io.hpp
  include/pcl/common/impl/file_io.hpp
  include/pcl/common/impl/norms.hpp
  include/pcl/common/impl/piecewise_linear_function.hpp
  include/pcl/common/impl/polynomial_calculations.hpp
  include/pcl/common/impl/pca.hpp
  include/pcl/common/impl/transforms.hpp
  include/pcl/common/impl/transformation_from_correspondences.hpp
  include/pcl/common/impl/vector_average.hpp
  include/pcl/common/impl/gaussian.hpp
  include/pcl/common/impl/spring.hpp
  include/pcl/common/impl/intensity.hpp
  include/pcl/common/impl/random.hpp
  include/pcl/common/impl/generate.hpp
  include/pcl/common/impl/projection_matrix.hpp
  include/pcl/common/impl/accumulators.hpp
)

set(impl_incs
  include/pcl/impl/pcl_base.hpp
  include/pcl/impl/instantiate.hpp
  include/pcl/impl/point_types.hpp
  include/pcl/impl/cloud_iterator.hpp
)

set(tools_incs
  include/pcl/console/parse.h
  include/pcl/console/print.h
  include/pcl/console/time.h
)

set(kissfft_incs
  include/pcl/common/fft/_kiss_fft_guts.h
  include/pcl/common/fft/kiss_fft.h
  include/pcl/common/fft/kiss_fftr.h
)

set(kissfft_srcs
  src/fft/kiss_fft.c
  src/fft/kiss_fftr.c
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${kissfft_srcs} ${incs} ${common_incs} ${impl_incs} ${tools_incs} ${kissfft_incs} ${common_incs_impl} ${range_image_incs} ${range_image_incs_impl})

target_link_libraries(${LIB_NAME} Boost::boost)

if(MSVC AND NOT (MSVC_VERSION LESS 1915))
  # MSVC resolved a byte alignment issue in compiler version 15.9
  # We get this due to using Eigen objects and allocating those objects with make_shared
  # It has to be public so that everything linking with common has this definition too
  target_compile_definitions(${LIB_NAME} PUBLIC _ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

set(EXT_DEPS eigen3)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" common ${common_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" common/fft ${kissfft_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" common/impl ${common_incs_impl})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" impl ${impl_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" console ${tools_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" range_image ${range_image_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" range_image/impl ${range_image_incs_impl})
