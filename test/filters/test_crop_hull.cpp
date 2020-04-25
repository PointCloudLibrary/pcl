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

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>

#include <random>

#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>


namespace
{
  bool randomBool()
  {
    static std::default_random_engine gen;
    static std::uniform_int_distribution<> int_distr(0, 1);
    return int_distr(gen);
  }

  struct TestData
  {
    TestData(pcl::Indices const & insideIndices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud)
      : inputCloud(inputCloud),
        insideMask(inputCloud->size(), false),
        insideIndices(insideIndices),
        insideCloud(new pcl::PointCloud<pcl::PointXYZ>),
        outsideCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
      pcl::copyPointCloud(*inputCloud, insideIndices, *insideCloud);
      for (pcl::index_t idx : insideIndices) {
        insideMask[idx] = true;
      }
      for (size_t i = 0; i < inputCloud->size(); ++i) {
        if (!insideMask[i]) {
          outsideIndices.push_back(i);
        }
      }
      pcl::copyPointCloud(*inputCloud, outsideIndices, *outsideCloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;
    std::vector<bool> insideMask;
    pcl::Indices insideIndices, outsideIndices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr insideCloud, outsideCloud;
  };


  std::vector<TestData>
  createTestDataSuite(
      std::function<pcl::PointXYZ()> insidePointGenerator,
      std::function<pcl::PointXYZ()> outsidePointGenerator)
  {
    std::vector<TestData> testDataSuite;
    size_t const chunkSize = 1000;
    pcl::PointCloud<pcl::PointXYZ>::Ptr insideCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outsideCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mixedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::Indices insideIndicesForInsideCloud;
    pcl::Indices insideIndicesForOutsideCloud; // empty indices, cause outsideCloud don't contains any inside point
    pcl::Indices insideIndicesForMixedCloud;
    for (size_t i = 0; i < chunkSize; ++i)
    {
      insideIndicesForInsideCloud.push_back(i);
      insideCloud->push_back(insidePointGenerator());
      outsideCloud->push_back(outsidePointGenerator());
      if (randomBool()) {
        insideIndicesForMixedCloud.push_back(i);
        mixedCloud->push_back(insidePointGenerator());
      }
      else {
        mixedCloud->push_back(outsidePointGenerator());
      }
    }
    testDataSuite.emplace_back(std::move(insideIndicesForInsideCloud), insideCloud);
    testDataSuite.emplace_back(std::move(insideIndicesForOutsideCloud), outsideCloud);
    testDataSuite.emplace_back(std::move(insideIndicesForMixedCloud), mixedCloud);
    return testDataSuite;
  }


  template <class CropHullTestTraits>
  class PCLCropHullTestFixture : public ::testing::Test
  {
    public:
      PCLCropHullTestFixture()
        : gen(12345u),
          rd(0.0f, 1.0f)
      {
        baseOffsetList.emplace_back(0, 0, 0);
        baseOffsetList.emplace_back(5, 1, 10);
        baseOffsetList.emplace_back(1, 5, 10);
        baseOffsetList.emplace_back(1, 10, 5);
        baseOffsetList.emplace_back(10, 1, 5);
        baseOffsetList.emplace_back(10, 5, 1);
      }
    protected:

      void SetUp() override
      {
        data.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
        for (pcl::PointXYZ const & baseOffset : baseOffsetList)
        {
          pcl::copyPointCloud(*CropHullTestTraits::getInputCloud(), *inputCloud);
          for (pcl::PointXYZ & p : *inputCloud) {
            p.getVector3fMap() += baseOffset.getVector3fMap();
          }
          auto insidePointGenerator = [this, &baseOffset] () {
            pcl::PointXYZ p(rd(gen), rd(gen), rd(gen));
            p.getVector3fMap() += baseOffset.getVector3fMap();
            return p;
          };
          auto outsidePointGenerator = [this, &baseOffset] () {
            pcl::PointXYZ p(rd(gen) + 2., rd(gen) + 2., rd(gen) + 2.);
            p.getVector3fMap() += baseOffset.getVector3fMap();
            return p;
          };
          pcl::CropHull<pcl::PointXYZ> cropHullFilter = createDefaultCropHull(inputCloud);
          std::vector<TestData> testDataSuite = createTestDataSuite(insidePointGenerator, outsidePointGenerator);
          data.emplace_back(cropHullFilter, testDataSuite);
        }
      }

      std::vector<std::pair<pcl::CropHull<pcl::PointXYZ>, std::vector<TestData>>> data;

    private:
      pcl::CropHull<pcl::PointXYZ> createDefaultCropHull (pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud) const
      {
        //pcl::CropHull<pcl::PointXYZ> cropHullFilter(true);
        pcl::CropHull<pcl::PointXYZ> cropHullFilter;
        pcl::ConvexHull<pcl::PointXYZ> convexHull;
        convexHull.setDimension(3);
        convexHull.setInputCloud(inputCloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::Vertices> hullPolygons;
        convexHull.reconstruct(*hullCloudPtr, hullPolygons);
        cropHullFilter.setHullIndices(hullPolygons);
        cropHullFilter.setHullCloud(hullCloudPtr);
        cropHullFilter.setDim(CropHullTestTraits::getDim());
        return cropHullFilter;
      }

      mutable std::mt19937 gen;
      mutable std::uniform_real_distribution<float> rd;
      pcl::PointCloud<pcl::PointXYZ> baseOffsetList;
  };


  struct CropHullTestTraits2d
  {
    static pcl::PointCloud<pcl::PointXYZ>::ConstPtr getInputCloud()
    {
      static pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
      if (inputCloud->empty()) {
        for (const float i: {0.f, 1.f})
          for (const float j: {0.f, 1.f})
            for (const float k: {0.f, -0.1f})
              inputCloud->emplace_back(i, j, k);
      }
      return inputCloud;
    }
    static int getDim() {
      return 2;
    }
  };


  struct CropHullTestTraits3d
  {
    static pcl::PointCloud<pcl::PointXYZ>::ConstPtr getInputCloud()
    {
      static pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
      if (inputCloud->empty()) {
        for (const float i: {0.f, 1.f})
          for (const float j: {0.f, 1.f})
            for (const float k: {0.f, 1.f})
              inputCloud->emplace_back(i, j, k);
      }
      return inputCloud;
    }
    static int getDim() {
      return 3;
    }
  };
}
using CropHullTestTypes = ::testing::Types<CropHullTestTraits2d, CropHullTestTraits3d>;
TYPED_TEST_SUITE(PCLCropHullTestFixture, CropHullTestTypes);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// since test input cloud has same distribution for all dimensions, this test also check problem from issue #3960 //
TYPED_TEST (PCLCropHullTestFixture, simple_test)
{
  for (auto & entry : this->data)
  {
    auto & cropHullFilter = entry.first;
    for (TestData const & testData : entry.second)
    {
      cropHullFilter.setInputCloud(testData.inputCloud);
      pcl::Indices filteredIndices;
      cropHullFilter.filter(filteredIndices);
      pcl::test::EXPECT_EQ_VECTORS(testData.insideIndices, filteredIndices);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this test will pass only for 2d case //
using PCLCropHullTestFixture2d = PCLCropHullTestFixture<CropHullTestTraits2d>;
TEST_F (PCLCropHullTestFixture2d, test_crop_inside)
{
  for (auto & entry : this->data)
  {
    auto & cropHullFilter = entry.first;
    for (TestData const & testData : entry.second)
    {
      cropHullFilter.setInputCloud(testData.inputCloud);
      cropHullFilter.setCropOutside(false);
      pcl::Indices filteredIndices;
      cropHullFilter.filter(filteredIndices);
      pcl::test::EXPECT_EQ_VECTORS(testData.outsideIndices, filteredIndices);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCropHullTestFixture, test_cloud_filtering)
{
  for (auto & entry : this->data)
  {
    auto & cropHullFilter = entry.first;
    for (TestData const & testData : entry.second)
    {
      cropHullFilter.setInputCloud(testData.inputCloud);
      pcl::PointCloud<pcl::PointXYZ> filteredCloud;
      cropHullFilter.filter(filteredCloud);
      ASSERT_EQ (testData.insideCloud->size(), filteredCloud.size());
      for (pcl::index_t i = 0; i < testData.insideCloud->size(); ++i)
      {
        EXPECT_XYZ_NEAR(testData.insideCloud->at(i), filteredCloud[i], 1e-5);
      }
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
