/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 */

#include <random>
#include <algorithm>
#include <array>
#include <tuple>

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>


namespace
{


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
    if (i % 2) {
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


template <class TupleType>
class PCLCropHullTestFixture : public ::testing::Test
{
  public:
    using CropHullTestTraits = std::tuple_element_t<0, TupleType>;
    using RandomGeneratorType = std::tuple_element_t<1, TupleType>;

    PCLCropHullTestFixture()
    {
      baseOffsetList_.emplace_back(0, 0, 0);
      baseOffsetList_.emplace_back(5, 1, 10);
      baseOffsetList_.emplace_back(1, 5, 10);
      baseOffsetList_.emplace_back(1, 10, 5);
      baseOffsetList_.emplace_back(10, 1, 5);
      baseOffsetList_.emplace_back(10, 5, 1);
    }
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  protected:

    void
    SetUp () override
    {
      data_.clear();
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      for (pcl::PointXYZ const & baseOffset : baseOffsetList_)
      {
        pcl::copyPointCloud(*CropHullTestTraits::getHullCloud(), *input_cloud);
        for (pcl::PointXYZ & p : *input_cloud) {
          p.getVector3fMap() += baseOffset.getVector3fMap();
        }
        auto inside_point_generator = [this, &baseOffset] () {
          pcl::PointXYZ p(rg_(), rg_(), rg_());
          p.getVector3fMap() += baseOffset.getVector3fMap();
          return p;
        };
        auto outside_point_generator = [this, &baseOffset] () {
          std::array<float, 3> pt;
          std::generate(pt.begin(), pt.end(),
              [this] {
                float v = rg_();
                return v + std::copysign(0.51, v);
              });
          pcl::PointXYZ p(pt[0], pt[1], pt[2]);
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
      crop_hull_filter.setHullCloud(input_cloud->makeShared());
      crop_hull_filter.setHullIndices(CropHullTestTraits::getHullPolygons());
      crop_hull_filter.setDim(CropHullTestTraits::getDim());
      return crop_hull_filter;
    }

    RandomGeneratorType rg_;
    pcl::PointCloud<pcl::PointXYZ> baseOffsetList_;
};


struct CropHullTestTraits2d
{
  static pcl::PointCloud<pcl::PointXYZ>::ConstPtr
  getHullCloud();

  static std::vector<pcl::Vertices>
  getHullPolygons();

  static int
  getDim();
};


struct CropHullTestTraits3d
{
  static pcl::PointCloud<pcl::PointXYZ>::ConstPtr
  getHullCloud();

  static std::vector<pcl::Vertices>
  getHullPolygons();

  static int
  getDim();
};


template <size_t seed> struct RandomGenerator
{
  public:
    RandomGenerator()
      : gen_(seed), rd_(-0.5f, 0.5f)
    {}

    float operator()() {
      return rd_(gen_);
    }

  private:
    std::mt19937 gen_;
    std::uniform_real_distribution<float> rd_;
};


static std::vector<std::vector<pcl::index_t>> cube_elements = {
  {0, 2, 1}, // l
  {1, 2, 3}, // l
  {3, 2, 6}, // f
  {6, 2, 4}, // bt
  {4, 2, 0}, // bt
  {3, 7, 1}, // t
  {1, 7, 5}, // t
  {5, 7, 4}, // r
  {4, 7, 6}, // r
  {6, 7, 3}, // f
  {5, 1, 4}, // back
  {4, 1, 0}  // back
};


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
CropHullTestTraits2d::getHullCloud ()
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (input_cloud->empty()) {
    for (const float i: {-0.5f, 0.5f})
      for (const float j: {-0.5f, .5f})
        for (const float k: {0.f, -0.1f})
          input_cloud->emplace_back(i, j, k);
  }
  return input_cloud;
}

std::vector<pcl::Vertices>
CropHullTestTraits2d::getHullPolygons ()
{
  std::vector<pcl::Vertices> polygons(12);
  for (size_t i = 0; i < 12; ++i) {
    polygons[i].vertices = cube_elements[i];
  }
  return polygons;
}

int
CropHullTestTraits2d::getDim ()
{
  return 2;
}


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
CropHullTestTraits3d::getHullCloud ()
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (input_cloud->empty()) {
    for (const float i: {-0.5f, 0.5f})
      for (const float j: {-0.5f, 0.5f})
        for (const float k: {-0.5f, 0.5f})
          input_cloud->emplace_back(i, j, k);
  }
  return input_cloud;
}

std::vector<pcl::Vertices>
CropHullTestTraits3d::getHullPolygons ()
{
  std::vector<pcl::Vertices> polygons(12);
  for (size_t i = 0; i < 12; ++i) {
    polygons[i].vertices = cube_elements[i];
  }
  return polygons;
}

int
CropHullTestTraits3d::getDim ()
{
  return 3;
}

} // end of anonymous namespace
using CropHullTestTraits2dList = std::tuple<
  std::tuple<CropHullTestTraits2d, RandomGenerator<0>>,
  std::tuple<CropHullTestTraits2d, RandomGenerator<123>>,
  std::tuple<CropHullTestTraits2d, RandomGenerator<456>>
  >;
using CropHullTestTraits3dList = std::tuple<
  std::tuple<CropHullTestTraits3d, RandomGenerator<0>>,
  std::tuple<CropHullTestTraits3d, RandomGenerator<123>>,
  std::tuple<CropHullTestTraits3d, RandomGenerator<456>>
  >;
using CropHullTestTypes = ::testing::Types<
  std::tuple_element_t<0, CropHullTestTraits2dList>,
  std::tuple_element_t<1, CropHullTestTraits2dList>,
  std::tuple_element_t<2, CropHullTestTraits2dList>,
  std::tuple_element_t<0, CropHullTestTraits3dList>,
  std::tuple_element_t<1, CropHullTestTraits3dList>,
  std::tuple_element_t<2, CropHullTestTraits3dList>
  >;
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
      ASSERT_EQ(test_data.inside_indices_.size(), filtered_indices.size());
      pcl::test::EXPECT_EQ_VECTORS(test_data.inside_indices_, filtered_indices);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// checking that the result is independent of the original state of the output_indices
TYPED_TEST (PCLCropHullTestFixture, non_empty_output_indices)
{
  for (auto & entry : this->data_)
  {
    auto & crop_hull_filter = entry.first;
    for (TestData const & test_data : entry.second)
    {
      crop_hull_filter.setInputCloud(test_data.input_cloud_);
      // the size of indices array does not matter. only that it is not empty
      pcl::Indices filtered_indices(42);
      crop_hull_filter.filter(filtered_indices);
      ASSERT_EQ(test_data.inside_indices_.size(), filtered_indices.size());
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
      pcl::index_t cloud_size = test_data.inside_cloud_->size();
      for (pcl::index_t i = 0; i < cloud_size; ++i)
      {
        EXPECT_XYZ_NEAR(test_data.inside_cloud_->at(i), filteredCloud[i], 1e-5);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCropHullTestFixture, test_keep_organized)
{
  for (auto & entry : this->data_)
  {
    auto & crop_hull_filter = entry.first;
    crop_hull_filter.setKeepOrganized(true);
    crop_hull_filter.setUserFilterValue(-10.);
    const pcl::PointXYZ defaultPoint(-10., -10., -10.);
    for (TestData const & test_data : entry.second)
    {
      crop_hull_filter.setInputCloud(test_data.input_cloud_);
      pcl::PointCloud<pcl::PointXYZ> filteredCloud;
      crop_hull_filter.filter(filteredCloud);
      ASSERT_EQ (test_data.input_cloud_->size(), filteredCloud.size());
      for (size_t i = 0; i < test_data.input_cloud_->size(); ++i)
      {
        pcl::PointXYZ expectedPoint = test_data.inside_mask_[i] ? test_data.input_cloud_->at(i) : defaultPoint;
        ASSERT_XYZ_NEAR(expectedPoint, filteredCloud[i], 1e-5);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCropHullTestFixture, test_crop_inside)
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
