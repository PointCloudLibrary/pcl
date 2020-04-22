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
  struct Checker
  {
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;

    Checker(pcl::ConvexHull<pcl::PointXYZ> & hull, int dim)
      : cropHullFilter(true)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::Vertices> hullPolygons;
      hull.reconstruct(*hullCloudPtr, hullPolygons);

      cropHullFilter.setHullIndices(hullPolygons);
      cropHullFilter.setHullCloud(hullCloudPtr);
      cropHullFilter.setDim(dim);
      cropHullFilter.setCropOutside(true);
    }

    void check(
        pcl::Indices const & expectedFilteredIndices,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud)
    {
      std::vector<bool> expectedFilteredMask(inputCloud->size(), false);
      pcl::Indices expectedRemovedIndices;
      pcl::PointCloud<pcl::PointXYZ> expectedCloud;
      pcl::copyPointCloud(*inputCloud, expectedFilteredIndices, expectedCloud);
      for (pcl::index_t idx : expectedFilteredIndices) {
        expectedFilteredMask[idx] = true;
      }
      for (size_t i = 0; i < inputCloud->size(); ++i) {
        if (!expectedFilteredMask[i]) {
          expectedRemovedIndices.push_back(i);
        }
      }

      cropHullFilter.setInputCloud(inputCloud);

      pcl::Indices filteredIndices;
      cropHullFilter.filter(filteredIndices);
      pcl::test::EXPECT_EQ_VECTORS(expectedFilteredIndices, filteredIndices);
      pcl::test::EXPECT_EQ_VECTORS(expectedRemovedIndices, *cropHullFilter.getRemovedIndices());
      // check negative filter functionality
      {
        cropHullFilter.setNegative(true);
        cropHullFilter.filter(filteredIndices);
        pcl::test::EXPECT_EQ_VECTORS(expectedRemovedIndices, filteredIndices);
        pcl::test::EXPECT_EQ_VECTORS(expectedFilteredIndices, *cropHullFilter.getRemovedIndices());
        cropHullFilter.setNegative(false);
      }
      // check cropOutside functionality
      {
        cropHullFilter.setCropOutside(false);
        cropHullFilter.filter(filteredIndices);
        pcl::test::EXPECT_EQ_VECTORS(expectedRemovedIndices, filteredIndices);
        pcl::test::EXPECT_EQ_VECTORS(expectedFilteredIndices, *cropHullFilter.getRemovedIndices());
        cropHullFilter.setCropOutside(true);
      }

      pcl::PointCloud<pcl::PointXYZ> filteredCloud;
      cropHullFilter.filter(filteredCloud);
      ASSERT_EQ (expectedCloud.size(), filteredCloud.size());
      for (pcl::index_t i = 0; i < expectedCloud.size(); ++i)
      {
        EXPECT_XYZ_NEAR(expectedCloud[i], filteredCloud[i], 1e-5);
      }
      // check non empty out cloud filtering
      cropHullFilter.filter(filteredCloud);
      EXPECT_EQ (expectedCloud.size(), filteredCloud.size());

      // check keep organized
      {
        cropHullFilter.setKeepOrganized(true);
        cropHullFilter.setUserFilterValue(-10.);
        pcl::PointXYZ defaultPoint(-10., -10., -10.);
        cropHullFilter.filter(filteredCloud);
        ASSERT_EQ (inputCloud->size(), filteredCloud.size());
        for (pcl::index_t i = 0; i < inputCloud->size(); ++i)
        {
          pcl::PointXYZ expectedPoint = expectedFilteredMask[i] ? inputCloud->at(i) : defaultPoint;
          EXPECT_XYZ_NEAR(expectedPoint, filteredCloud[i], 1e-5);
        }
        cropHullFilter.setKeepOrganized(false);
      }
    }
  };

  bool randomBool() {
    static auto gen = std::bind(
        std::uniform_int_distribution<>(0,1),
        std::default_random_engine());
    return gen();
  }

  std::vector<std::pair<pcl::Indices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>>
  createTestData(
      std::function<pcl::PointXYZ()> insidePointGenerator,
      std::function<pcl::PointXYZ()> outsidePointGenerator)
  {
    std::vector<std::pair<pcl::Indices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> testData;
    size_t const chunkSize = 1000;
    pcl::PointCloud<pcl::PointXYZ>::Ptr insideCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outsideCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mixedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::Indices insideIndicesForInsideCloud;
    pcl::Indices insideIndicesForOutsideCloud;
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
    testData.emplace_back(insideIndicesForInsideCloud, insideCloud);
    testData.emplace_back(insideIndicesForOutsideCloud, outsideCloud);
    testData.emplace_back(insideIndicesForMixedCloud, mixedCloud);
    return testData;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_2dsquare)
{
  pcl::PointCloud<pcl::PointXYZ> baseOffsetList;
  baseOffsetList.emplace_back(5, 1, -3);
  baseOffsetList.emplace_back(1, 5, -3);
  for (pcl::PointXYZ const & baseOffset : baseOffsetList)
  {
    pcl::ConvexHull<pcl::PointXYZ> convexHull;
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ> ());
      inputCloud->emplace_back(0.0f, 0.0f, 0.0f);
      inputCloud->emplace_back(0.0f, 1.0f, 0.0f);
      inputCloud->emplace_back(1.0f, 0.0f, 0.0f);
      inputCloud->emplace_back(1.0f, 1.0f, 0.0f);
      inputCloud->emplace_back(0.0f, 0.0f, 0.1f);
      inputCloud->emplace_back(0.0f, 1.0f, 0.1f);
      inputCloud->emplace_back(1.0f, 0.0f, 0.1f);
      inputCloud->emplace_back(1.0f, 1.0f, 0.1f);
      for (pcl::PointXYZ & p : *inputCloud) {
        p.getVector3fMap() += baseOffset.getVector3fMap();
      }

      convexHull.setDimension(3);
      convexHull.setInputCloud(inputCloud);
    }
    Checker checker(convexHull, 2);

    std::mt19937 gen(12345u);
    std::uniform_real_distribution<float> rd (0.0f, 1.0f);
    auto insidePointGenerator = [&rd, &gen, baseOffset] () {
      pcl::PointXYZ p(rd(gen), rd(gen), 1.0);
      p.getVector3fMap() += baseOffset.getVector3fMap();
      return p;
    };
    auto outsidePointGenerator = [&rd, &gen, baseOffset] () {
      pcl::PointXYZ p(rd(gen) + 2., rd(gen) + 2., rd(gen) + 2.);
      p.getVector3fMap() += baseOffset.getVector3fMap();
      return p;
    };
    std::vector<std::pair<pcl::Indices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> testData =
      createTestData(insidePointGenerator, outsidePointGenerator);

    for (auto const & entry : testData)
    {
      checker.check(entry.first, entry.second);
    }
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, issue_1657_CropHull3d_not_cropping_inside)
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  pcl::CropHull<pcl::PointXYZ> cropHullFilter(true);
  PointCloud::Ptr hullCloud(new PointCloud());
  PointCloud::Ptr hullPoints(new PointCloud());
  std::vector<pcl::Vertices> hullPolygons;

  hullCloud->clear();
  {
    pcl::PointXYZ p;
    p.x = -1;
    p.y = -1;
    p.z = -1;
    hullCloud->push_back(p);
    p.z = 1;
    hullCloud->push_back(p);
  }
  {
    pcl::PointXYZ p;
    p.x = -1;
    p.y = 1;
    p.z = -1;
    hullCloud->push_back(p);
    p.z = 1;
    hullCloud->push_back(p);
  }
  {
    pcl::PointXYZ p;
    p.x = 1;
    p.y = 1;
    p.z = -1;
    hullCloud->push_back(p);
    p.z = 1;
    hullCloud->push_back(p);
  }
  {
    pcl::PointXYZ p;
    p.x = 1;
    p.y = -1;
    p.z = -1;
    hullCloud->push_back(p);
    p.z = 1;
    hullCloud->push_back(p);
  }

  // setup hull filter
  pcl::ConvexHull<pcl::PointXYZ> cHull;
  cHull.setInputCloud(hullCloud);
  cHull.reconstruct(*hullPoints, hullPolygons);

  cropHullFilter.setHullIndices(hullPolygons);
  cropHullFilter.setHullCloud(hullPoints);
  //cropHullFilter.setDim(2); // if you uncomment this, it will work
  cropHullFilter.setCropOutside(false); // this will remove points inside the hull

  // create point cloud
  PointCloud::Ptr pc(new PointCloud());

  // a point inside the hull
  {
    pcl::PointXYZ p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    pc->push_back(p);
  }

  // and a point outside the hull
  {
    pcl::PointXYZ p;
    p.x = 10;
    p.y = 10;
    p.z = 10;
    pc->push_back(p);
  }

  //filter points
  cropHullFilter.setInputCloud(pc);
  PointCloud::Ptr filtered(new PointCloud());
  cropHullFilter.filter(*filtered);

  ASSERT_EQ(1, filtered->size());

  EXPECT_NEAR(
      (filtered->front().getVector3fMap() - Eigen::Vector3f(10.0, 10.0, 10.0)).norm(),
      0.0,
      1e-5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_3dcube)
{
  pcl::PointCloud<pcl::PointXYZ> baseOffsetList;
  baseOffsetList.emplace_back(5, 1, -3);
  baseOffsetList.emplace_back(1, 5, -3);
  for (pcl::PointXYZ const & baseOffset : baseOffsetList)
  {
    pcl::ConvexHull<pcl::PointXYZ> convexHull;
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ> ());
      inputCloud->emplace_back(0.0f, 0.0f, 0.0f);
      inputCloud->emplace_back(0.0f, 0.0f, 1.0f);
      inputCloud->emplace_back(0.0f, 1.0f, 0.0f);
      inputCloud->emplace_back(0.0f, 1.0f, 1.0f);
      inputCloud->emplace_back(1.0f, 0.0f, 0.0f);
      inputCloud->emplace_back(1.0f, 0.0f, 1.0f);
      inputCloud->emplace_back(1.0f, 1.0f, 0.0f);
      inputCloud->emplace_back(1.0f, 1.0f, 1.0f);
      for (pcl::PointXYZ & p : *inputCloud) {
        p.getVector3fMap() += baseOffset.getVector3fMap();
      }

      convexHull.setDimension(3);
      convexHull.setInputCloud(inputCloud);
    }
    Checker checker(convexHull, 3);

    std::mt19937 gen(12345u);
    std::uniform_real_distribution<float> rd (0.0f, 1.0f);
    auto insidePointGenerator = [&rd, &gen, baseOffset] () {
      pcl::PointXYZ p(rd(gen), rd(gen), rd(gen));
      p.getVector3fMap() += baseOffset.getVector3fMap();
      return p;
    };
    auto outsidePointGenerator = [&rd, &gen, baseOffset] () {
      pcl::PointXYZ p(rd(gen) + 2., rd(gen) + 2., rd(gen) + 2.);
      p.getVector3fMap() += baseOffset.getVector3fMap();
      return p;
    };
    std::vector<std::pair<pcl::Indices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> testData =
      createTestData(insidePointGenerator, outsidePointGenerator);

    for (auto const & entry : testData)
    {
      checker.check(entry.first, entry.second);
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
