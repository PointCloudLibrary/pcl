/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: test_filters.cpp 7683 2012-10-23 02:49:03Z rusu $
 *
 */

#include <random>

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>

#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>


namespace
{

bool
getRandomBool ()
{
  static std::default_random_engine gen;
  static std::uniform_int_distribution<> int_distr(0, 1);
  return int_distr(gen);
}

struct TestData
{
  TestData(pcl::Indices const & insideIndices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
    : input_cloud_(input_cloud),
      inside_mask_(input_cloud_->size(), false),
      inside_indices_(insideIndices),
      inside_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      outside_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    pcl::copyPointCloud(*input_cloud_, inside_indices_, *inside_cloud_);
    for (pcl::index_t idx : inside_indices_) {
      inside_mask_[idx] = true;
    }
    for (size_t i = 0; i < input_cloud_->size(); ++i) {
      if (!inside_mask_[i]) {
        outside_indices_.push_back(i);
      }
    }
    pcl::copyPointCloud(*input_cloud_, outside_indices_, *outside_cloud_);
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_;
  std::vector<bool> inside_mask_;
  pcl::Indices inside_indices_, outside_indices_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inside_cloud_, outside_cloud_;
};


std::vector<TestData>
createTestDataSuite(
    std::function<pcl::PointXYZ()> inside_point_generator,
    std::function<pcl::PointXYZ()> outside_point_generator)
{
  std::vector<TestData> test_data_suite;
  size_t const chunk_size = 1000;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inside_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outside_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mixed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Indices inside_indices_for_inside_cloud;
  pcl::Indices inside_indices_for_outside_cloud; // empty indices, cause outside_cloud don't contains any inside point
  pcl::Indices inside_indices_for_mixed_cloud;
  for (size_t i = 0; i < chunk_size; ++i)
  {
    inside_indices_for_inside_cloud.push_back(i);
    inside_cloud->push_back(inside_point_generator());
    outside_cloud->push_back(outside_point_generator());
    if (getRandomBool()) {
      inside_indices_for_mixed_cloud.push_back(i);
      mixed_cloud->push_back(inside_point_generator());
    }
    else {
      mixed_cloud->push_back(outside_point_generator());
    }
  }
  test_data_suite.emplace_back(std::move(inside_indices_for_inside_cloud), inside_cloud);
  test_data_suite.emplace_back(std::move(inside_indices_for_outside_cloud), outside_cloud);
  test_data_suite.emplace_back(std::move(inside_indices_for_mixed_cloud), mixed_cloud);
  return test_data_suite;
}


template <class CropHullTestTraits>
class PCLCropHullTestFixture : public ::testing::Test
{
  public:
    PCLCropHullTestFixture()
      : gen_(12345u),
        rd_(0.0f, 1.0f)
    {
      baseOffsetList_.emplace_back(0, 0, 0);
      baseOffsetList_.emplace_back(5, 1, 10);
      baseOffsetList_.emplace_back(1, 5, 10);
      baseOffsetList_.emplace_back(1, 10, 5);
      baseOffsetList_.emplace_back(10, 1, 5);
      baseOffsetList_.emplace_back(10, 5, 1);
    }
  protected:

    void
    SetUp () override
    {
      data_.clear();
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      for (pcl::PointXYZ const & baseOffset : baseOffsetList_)
      {
        pcl::copyPointCloud(*CropHullTestTraits::getInputCloud(), *input_cloud);
        for (pcl::PointXYZ & p : *input_cloud) {
          p.getVector3fMap() += baseOffset.getVector3fMap();
        }
        auto inside_point_generator = [this, &baseOffset] () {
          pcl::PointXYZ p(rd_(gen_), rd_(gen_), rd_(gen_));
          p.getVector3fMap() += baseOffset.getVector3fMap();
          return p;
        };
        auto outside_point_generator = [this, &baseOffset] () {
          pcl::PointXYZ p(rd_(gen_) + 2., rd_(gen_) + 2., rd_(gen_) + 2.);
          p.getVector3fMap() += baseOffset.getVector3fMap();
          return p;
        };
        pcl::CropHull<pcl::PointXYZ> crop_hull_filter = createDefaultCropHull(input_cloud);
        std::vector<TestData> test_data_suite = createTestDataSuite(inside_point_generator, outside_point_generator);
        data_.emplace_back(crop_hull_filter, test_data_suite);
      }
    }

    std::vector<std::pair<pcl::CropHull<pcl::PointXYZ>, std::vector<TestData>>> data_;

  private:
    pcl::CropHull<pcl::PointXYZ>
    createDefaultCropHull (pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) const
    {
      //pcl::CropHull<pcl::PointXYZ> crop_hull_filter(true);
      pcl::CropHull<pcl::PointXYZ> crop_hull_filter;
      pcl::ConvexHull<pcl::PointXYZ> convex_hull;
      convex_hull.setDimension(3);
      convex_hull.setInputCloud(input_cloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::Vertices> hull_polygons;
      convex_hull.reconstruct(*hull_cloud_ptr, hull_polygons);
      crop_hull_filter.setHullIndices(hull_polygons);
      crop_hull_filter.setHullCloud(hull_cloud_ptr);
      crop_hull_filter.setDim(CropHullTestTraits::getDim());
      return crop_hull_filter;
    }

    mutable std::mt19937 gen_;
    mutable std::uniform_real_distribution<float> rd_;
    pcl::PointCloud<pcl::PointXYZ> baseOffsetList_;
};


struct CropHullTestTraits2d
{
  static pcl::PointCloud<pcl::PointXYZ>::ConstPtr
  getInputCloud();

  static int
  getDim();
};


struct CropHullTestTraits3d
{
  static pcl::PointCloud<pcl::PointXYZ>::ConstPtr
  getInputCloud();

  static int
  getDim();
};


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
CropHullTestTraits2d::getInputCloud ()
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (input_cloud->empty()) {
    for (const float i: {0.f, 1.f})
      for (const float j: {0.f, 1.f})
        for (const float k: {0.f, -0.1f})
          input_cloud->emplace_back(i, j, k);
  }
  return input_cloud;
}

int
CropHullTestTraits2d::getDim ()
{
  return 2;
}


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
CropHullTestTraits3d::getInputCloud ()
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (input_cloud->empty()) {
    for (const float i: {0.f, 1.f})
      for (const float j: {0.f, 1.f})
        for (const float k: {0.f, 1.f})
          input_cloud->emplace_back(i, j, k);
  }
  return input_cloud;
}

int
CropHullTestTraits3d::getDim ()
{
  return 3;
}

} // end of anonymous namespace
using CropHullTestTypes = ::testing::Types<CropHullTestTraits2d, CropHullTestTraits3d>;
TYPED_TEST_SUITE(PCLCropHullTestFixture, CropHullTestTypes);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// since test input cloud has same distribution for all dimensions, this test also check problem from issue #3960 //
TYPED_TEST (PCLCropHullTestFixture, simple_test)
{
  for (auto & entry : this->data_)
  {
    auto & crop_hull_filter = entry.first;
    for (TestData const & test_data : entry.second)
    {
      crop_hull_filter.setInputCloud(test_data.input_cloud_);
      pcl::Indices filtered_indices;
      crop_hull_filter.filter(filtered_indices);
      pcl::test::EXPECT_EQ_VECTORS(test_data.inside_indices_, filtered_indices);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCropHullTestFixture, test_cloud_filtering)
{
  for (auto & entry : this->data_)
  {
    auto & crop_hull_filter = entry.first;
    for (TestData const & test_data : entry.second)
    {
      crop_hull_filter.setInputCloud(test_data.input_cloud_);
      pcl::PointCloud<pcl::PointXYZ> filteredCloud;
      crop_hull_filter.filter(filteredCloud);
      ASSERT_EQ (test_data.inside_cloud_->size(), filteredCloud.size());
      for (pcl::index_t i = 0; i < test_data.inside_cloud_->size(); ++i)
      {
        EXPECT_XYZ_NEAR(test_data.inside_cloud_->at(i), filteredCloud[i], 1e-5);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this test will pass only for 2d case //
using PCLCropHullTestFixture2d = PCLCropHullTestFixture<CropHullTestTraits2d>;
TEST_F (PCLCropHullTestFixture2d, test_crop_inside)
{
  for (auto & entry : this->data_)
  {
    auto & crop_hull_filter = entry.first;
    for (TestData const & test_data : entry.second)
    {
      crop_hull_filter.setInputCloud(test_data.input_cloud_);
      crop_hull_filter.setCropOutside(false);
      pcl::Indices filtered_indices;
      crop_hull_filter.filter(filtered_indices);
      pcl::test::EXPECT_EQ_VECTORS(test_data.outside_indices_, filtered_indices);
    }
  }
}



/* ---[ */
int
main (int argc, char** argv)
{
  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
