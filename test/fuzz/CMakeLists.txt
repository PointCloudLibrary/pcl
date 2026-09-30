#
# SPDX-License-Identifier: BSD-3-Clause
#
#  Point Cloud Library (PCL) - www.pointclouds.org
#  Copyright (c) 2020-, Open Perception
#
#  All rights reserved
#
set(SUBSYS_NAME tests_fuzz)
set(SUBSYS_DESC "Fuzzer to check for regression in point cloud library io module")

PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS io)

set(DEFAULT OFF)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(fuzz_ply test_fuzz_ply
             FILES regression_fuzzer.cpp io_wrappers.cpp
             LINK_WITH pcl_gtest pcl_io
             ARGUMENTS "${PCL_SOURCE_DIR}/test/fuzz/corpus/")
