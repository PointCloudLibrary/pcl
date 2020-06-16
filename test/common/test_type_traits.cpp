/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2019-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/memory.h>  // for pcl::has_custom_allocator, PCL_MAKE_ALIGNED_OPERATOR_NEW
#include <pcl/type_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h> // for pcl::isXYFinite, pcl::isXYZFinite, pcl::isNormalFinite

#include <pcl/test/gtest.h>

TEST (TypeTraits, HasCustomAllocatorTrait)
{
  struct Foo
  {
  public:
    // Manually ignore warnings here because of an issue in Eigen which
    // results in a local typedef being unused inside the new and delete
    // operators added by Eigen for C++14 or lower standards
    /** \todo Remove for C++17 (or future standards)
     */
    #ifdef __clang__
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunused-local-typedef"
    #endif
    PCL_MAKE_ALIGNED_OPERATOR_NEW
    #ifdef __clang__
    #pragma clang diagnostic pop
    #endif
  };

  struct Bar
  {
  };

  EXPECT_TRUE(pcl::has_custom_allocator<Foo>::value);
  EXPECT_FALSE(pcl::has_custom_allocator<Bar>::value);
}

TEST (TypeTraits, HasXY)
{
  static_assert(!pcl::traits::has_xy_v<pcl::Normal>,
                "has_xy<> should detect lack of x and y fields");
  static_assert(pcl::traits::has_xy_v<pcl::PointXYZ>,
                "has_xy<> should detect x and y fields");
}

TEST (TypeTraits, HasXYZ)
{
  static_assert(!pcl::traits::has_xyz_v<pcl::Normal>,
                "has_xyz<> should detect lack of x or y or z fields");
  static_assert(pcl::traits::has_xyz_v<pcl::PointXYZ>,
                "has_xyz<> should detect x, y and z fields");
}

TEST (TypeTraits, HasNormal)
{
  static_assert(!pcl::traits::has_normal_v<pcl::PointXYZ>,
                "has_normal<> should detect lack of normal_{x or y or z} fields");
  static_assert(pcl::traits::has_normal_v<pcl::Axis>,
                "has_normal<> should detect normal_{x, y and z} fields");
}

TEST (TypeTraits, HasCurvature)
{
  static_assert(!pcl::traits::has_curvature_v<pcl::PointXYZ>,
                "has_curvature<> should detect lack of curvature field");
  static_assert(pcl::traits::has_curvature_v<pcl::Normal>,
                "has_curvature<> should detect curvature field");
}

TEST (TypeTraits, HasIntensity)
{
  static_assert(!pcl::traits::has_intensity_v<pcl::InterestPoint>,
                "has_intensity<> should detect lack of intensity field");
  static_assert(pcl::traits::has_intensity_v<pcl::PointXYZI>,
                "has_intensity<> should detect intensity field");
}

TEST (TypeTraits, HasColor)
{
  static_assert(!pcl::traits::has_color_v<pcl::PointXYZ>,
                "has_color<> should detect lack of rgb field");
  static_assert(pcl::traits::has_color_v<pcl::PointXYZRGB>,
                "has_color<> should detect rgb field");
  static_assert(pcl::traits::has_color_v<pcl::PointXYZRGBA>,
                "has_color<> should detect rgb field");
}

TEST (TypeTraits, HasLabel)
{
  static_assert(!pcl::traits::has_label_v<pcl::PointXYZRGB>,
                "has_label<> should detect lack of label field");
  static_assert(pcl::traits::has_label_v<pcl::PointXYZRGBL>,
                "has_label<> should detect label field");
}

TEST (TypeTests, IsXYFinite)
{
  EXPECT_TRUE(pcl::isXYFinite(pcl::RGB {}));
  EXPECT_TRUE(pcl::isXYFinite(pcl::PointXYZ {2,3,4}));

  EXPECT_TRUE(pcl::isXYFinite(pcl::PointXYZ {std::numeric_limits<float>::max(), 3, 4}));
  EXPECT_TRUE(pcl::isXYFinite(pcl::PointXYZ {std::numeric_limits<float>::min(), 3, 4}));

  EXPECT_FALSE(pcl::isXYFinite(pcl::PointXYZ {std::numeric_limits<float>::infinity(), 3, 4}));
  EXPECT_FALSE(pcl::isXYFinite(pcl::PointXYZ {-std::numeric_limits<float>::infinity(), 3, 4}));

  EXPECT_FALSE(pcl::isXYFinite(pcl::PointXYZ {std::numeric_limits<float>::quiet_NaN(), 3, 4}));
  EXPECT_FALSE(pcl::isXYFinite(pcl::PointXYZ {-std::numeric_limits<float>::signaling_NaN(), 3, 4}));
}


TEST (TypeTests, IsXYZFinite)
{
  EXPECT_TRUE(pcl::isXYZFinite(pcl::RGB {}));
  EXPECT_TRUE(pcl::isXYZFinite(pcl::PointXYZ {2,3,4}));

  EXPECT_TRUE(pcl::isXYZFinite(pcl::PointXYZ {std::numeric_limits<float>::max(), 3, 4}));
  EXPECT_TRUE(pcl::isXYZFinite(pcl::PointXYZ {std::numeric_limits<float>::min(), 3, 4}));

  EXPECT_FALSE(pcl::isXYZFinite(pcl::PointXYZ {std::numeric_limits<float>::infinity(), 3, 4}));
  EXPECT_FALSE(pcl::isXYZFinite(pcl::PointXYZ {-std::numeric_limits<float>::infinity(), 3, 4}));

  EXPECT_FALSE(pcl::isXYZFinite(pcl::PointXYZ {std::numeric_limits<float>::quiet_NaN(), 3, 4}));
  EXPECT_FALSE(pcl::isXYZFinite(pcl::PointXYZ {-std::numeric_limits<float>::signaling_NaN(), 3, 4}));
}

TEST (TypeTests, IsNormalFinite)
{
  EXPECT_TRUE(pcl::isNormalFinite(pcl::RGB {}));
  EXPECT_TRUE(pcl::isNormalFinite(pcl::Normal {2,3,4}));

  EXPECT_TRUE(pcl::isNormalFinite(pcl::Normal {std::numeric_limits<float>::max(), 3, 4}));
  EXPECT_TRUE(pcl::isNormalFinite(pcl::Normal {std::numeric_limits<float>::min(), 3, 4}));

  EXPECT_FALSE(pcl::isNormalFinite(pcl::Normal {std::numeric_limits<float>::infinity(), 3, 4}));
  EXPECT_FALSE(pcl::isNormalFinite(pcl::Normal {-std::numeric_limits<float>::infinity(), 3, 4}));

  EXPECT_FALSE(pcl::isNormalFinite(pcl::Normal {std::numeric_limits<float>::quiet_NaN(), 3, 4}));
  EXPECT_FALSE(pcl::isNormalFinite(pcl::Normal {-std::numeric_limits<float>::signaling_NaN(), 3, 4}));
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
