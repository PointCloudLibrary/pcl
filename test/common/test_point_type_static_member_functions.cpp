/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_macros.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/type_traits.h>

using namespace pcl;
using namespace pcl::test;

template <typename T> class PointTypeStaticMemberFunctionsTest : public testing::Test { };
using FeaturePointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM (PCL_DESCRIPTOR_FEATURE_POINT_TYPES), Histogram<1>>;
TYPED_TEST_SUITE (PointTypeStaticMemberFunctionsTest, FeaturePointTypes);
TYPED_TEST (PointTypeStaticMemberFunctionsTest, DescriptorSizeTests)
{
  static_assert
  (
    TypeParam::descriptorSize() == pcl::detail::traits::descriptorSize_v<TypeParam>,
    "incorrect descriptorSize"
  );
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
