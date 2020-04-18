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

#include <random>

#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>


namespace
{
  template <class PointGenerator1, class PointGenerator2>
  void complexTest(
      pcl::CropHull<pcl::PointXYZ> cropHullFilter,
      PointGenerator1 insidePointGenerator,
      PointGenerator2 outsidePointGenerator)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr insideCubeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outsideCubeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mixedCubeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    size_t const chunkSize = 100;
    {
      for (size_t i = 0; i < chunkSize; ++i)
      {
        insideCubeCloud->push_back(insidePointGenerator());
        outsideCubeCloud->push_back(outsidePointGenerator());
      }
      *mixedCubeCloud = (*insideCubeCloud) + (*outsideCubeCloud);
    }
    std::vector<int> emptyIndices;
    std::vector<int> lowerIndices(insideCubeCloud->size());
    std::vector<int> upperIndices(insideCubeCloud->size());
    std::iota(lowerIndices.begin(), lowerIndices.end(), 0);
    std::iota(upperIndices.begin(), upperIndices.end(), lowerIndices.size());

    struct CheckEquals {
      pcl::CropHull<pcl::PointXYZ> cropHullFilter;
      CheckEquals(pcl::CropHull<pcl::PointXYZ> const & cropHullFilter)
        : cropHullFilter(cropHullFilter)
      {}
      void check(
          std::vector<int> const & expectedFilteredIndices,
          pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud)
      {
        std::vector<bool> expectedFilteredMask(inputCloud->size(), false);
        std::vector<int> expectedRemovedIndices;
        pcl::PointCloud<pcl::PointXYZ> expectedCloud;
        pcl::copyPointCloud(*inputCloud, expectedFilteredIndices, expectedCloud);
        for (int idx : expectedFilteredIndices) {
          expectedFilteredMask[idx] = true;
        }
        for (size_t i = 0; i < inputCloud->size(); ++i) {
          if (!expectedFilteredMask[i]) {
            expectedRemovedIndices.push_back(i);
          }
        }

        cropHullFilter.setInputCloud(inputCloud);

        std::vector<int> filteredIndices;
        cropHullFilter.filter(filteredIndices);
        ASSERT_EQ (expectedFilteredIndices.size(), filteredIndices.size());
        for (int i = 0; i < filteredIndices.size(); ++i)
        {
          ASSERT_EQ (expectedFilteredIndices[i], filteredIndices[i]);
        }
        //expected extract_removed_indices_ is true
        ASSERT_EQ (expectedRemovedIndices.size(), cropHullFilter.getRemovedIndices()->size());
        for (int i = 0; i < cropHullFilter.getRemovedIndices()->size(); ++i)
        {
          ASSERT_EQ (expectedRemovedIndices[i], cropHullFilter.getRemovedIndices()->at(i));
        }
        // check negative filter functionality
        {
          cropHullFilter.setNegative(true);
          cropHullFilter.filter(filteredIndices);
          //expected extract_removed_indices_ is true
          ASSERT_EQ (expectedRemovedIndices.size(), filteredIndices.size());
          for (int i = 0; i < filteredIndices.size(); ++i)
          {
            ASSERT_EQ (expectedRemovedIndices[i], filteredIndices[i]);
          }
          ASSERT_EQ (expectedFilteredIndices.size(), cropHullFilter.getRemovedIndices()->size());
          for (int i = 0; i < cropHullFilter.getRemovedIndices()->size(); ++i)
          {
            ASSERT_EQ (expectedFilteredIndices[i], cropHullFilter.getRemovedIndices()->at(i));
          }
          cropHullFilter.setNegative(false);
        }

        pcl::PointCloud<pcl::PointXYZ> filteredCloud;
        cropHullFilter.filter(filteredCloud);
        ASSERT_EQ (expectedCloud.size(), filteredCloud.size());
        for (int i = 0; i < expectedCloud.size(); ++i)
        {
          Eigen::Vector3f expectedPoint = expectedCloud[i].getVector3fMap();
          Eigen::Vector3f actualPoint = filteredCloud[i].getVector3fMap();
          ASSERT_NEAR((expectedPoint - actualPoint).norm(), 0.0, 1e-5);
        }
        // check non empty cloud filtering
        cropHullFilter.filter(filteredCloud);
        ASSERT_EQ (expectedCloud.size(), filteredCloud.size());

        // check keep organized
        {
          cropHullFilter.setKeepOrganized(true);
          cropHullFilter.setUserFilterValue(-10.);
          pcl::PointXYZ defaultPoint(-10., -10., -10.);
          cropHullFilter.filter(filteredCloud);
          ASSERT_EQ (inputCloud->size(), filteredCloud.size());
          for (int i = 0; i < inputCloud->size(); ++i)
          {
            Eigen::Vector3f actualPoint = filteredCloud[i].getVector3fMap();
            Eigen::Vector3f expectedPoint = defaultPoint.getVector3fMap();
            if (expectedFilteredMask[i])
            {
              expectedPoint = inputCloud->at(i).getVector3fMap();
            }
            ASSERT_NEAR((expectedPoint - actualPoint).norm(), 0.0, 1e-5);
          }
          cropHullFilter.setKeepOrganized(false);
        }
      }
    };

    {
      CheckEquals checker(cropHullFilter);
      checker.check(lowerIndices, insideCubeCloud);
      checker.check(emptyIndices, outsideCubeCloud);
      checker.check(lowerIndices, mixedCubeCloud);
    }

    {
      cropHullFilter.setCropOutside(false);
      CheckEquals checker(cropHullFilter);
      checker.check(emptyIndices, insideCubeCloud);
      checker.check(lowerIndices, outsideCubeCloud);
      checker.check(upperIndices, mixedCubeCloud);
    }
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
    pcl::CropHull<pcl::PointXYZ> cropHullFilter(true);
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

      pcl::ConvexHull<pcl::PointXYZ> convexHull;
      convexHull.setDimension(3);
      convexHull.setInputCloud(inputCloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::Vertices> hullPolygons;
      convexHull.reconstruct(*hullCloudPtr, hullPolygons);

      cropHullFilter.setHullIndices(hullPolygons);
      cropHullFilter.setHullCloud(hullCloudPtr);
      cropHullFilter.setDim(2);
      cropHullFilter.setCropOutside(true);
    }

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

    complexTest(cropHullFilter, insidePointGenerator, outsidePointGenerator);
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
    pcl::CropHull<pcl::PointXYZ> cropHullFilter(true);
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

      pcl::ConvexHull<pcl::PointXYZ> convexHull;
      convexHull.setDimension(3);
      convexHull.setInputCloud(inputCloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::Vertices> hullPolygons;
      convexHull.reconstruct(*hullCloudPtr, hullPolygons);

      cropHullFilter.setHullIndices(hullPolygons);
      cropHullFilter.setHullCloud(hullCloudPtr);
      cropHullFilter.setDim(3);
      cropHullFilter.setCropOutside(true);
    }

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

    complexTest(cropHullFilter, insidePointGenerator, outsidePointGenerator);
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
