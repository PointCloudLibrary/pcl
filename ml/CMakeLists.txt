set(SUBSYS_NAME ml)
set(SUBSYS_DESC "Point cloud machine learning library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/branch_estimator.h"
  "include/pcl/${SUBSYS_NAME}/feature_handler.h"
  "include/pcl/${SUBSYS_NAME}/multi_channel_2d_comparison_feature.h"
  "include/pcl/${SUBSYS_NAME}/multi_channel_2d_comparison_feature_handler.h"
  "include/pcl/${SUBSYS_NAME}/multi_channel_2d_data_set.h"
  "include/pcl/${SUBSYS_NAME}/multiple_data_2d_example_index.h"
  "include/pcl/${SUBSYS_NAME}/point_xy_32i.h"
  "include/pcl/${SUBSYS_NAME}/point_xy_32f.h"
  "include/pcl/${SUBSYS_NAME}/regression_variance_stats_estimator.h"
  "include/pcl/${SUBSYS_NAME}/stats_estimator.h"
  "include/pcl/${SUBSYS_NAME}/densecrf.h"
  "include/pcl/${SUBSYS_NAME}/pairwise_potential.h"
  "include/pcl/${SUBSYS_NAME}/permutohedral.h"
  "include/pcl/${SUBSYS_NAME}/svm_wrapper.h"
  "include/pcl/${SUBSYS_NAME}/svm.h"
  "include/pcl/${SUBSYS_NAME}/kmeans.h"
)

set(dt_incs
  "include/pcl/${SUBSYS_NAME}/dt/decision_forest.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_forest_evaluator.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_forest_trainer.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_tree.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_tree_evaluator.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_tree_trainer.h"
  "include/pcl/${SUBSYS_NAME}/dt/decision_tree_data_provider.h"
)

set(ferns_incs
  "include/pcl/${SUBSYS_NAME}/ferns/fern.h"
  "include/pcl/${SUBSYS_NAME}/ferns/fern_evaluator.h"
  "include/pcl/${SUBSYS_NAME}/ferns/fern_trainer.h"
)

set(dt_impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/dt/decision_forest_evaluator.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/dt/decision_forest_trainer.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/dt/decision_tree_evaluator.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/dt/decision_tree_trainer.hpp"
)

set(ferns_impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/ferns/fern_evaluator.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/ferns/fern_trainer.hpp"
)

set(svm_impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/svm/svm_wrapper.hpp"
)

set(srcs
  src/point_xy_32i.cpp
  src/point_xy_32f.cpp
  src/densecrf.cpp
  src/pairwise_potential.cpp
  src/permutohedral.cpp
  src/svm_wrapper.cpp
  src/svm.cpp
  src/kmeans.cpp
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
SET_TARGET_PROPERTIES("${LIB_NAME}" PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries("${LIB_NAME}" pcl_common)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})
# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/dt" ${dt_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ferns" ${ferns_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/dt" ${dt_impl_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/ferns" ${ferns_impl_incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl/svm" ${svm_impl_incs})
