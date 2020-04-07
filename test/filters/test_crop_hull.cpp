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
    pcl::PointCloud<pcl::PointXYZ>::Ptr emptyCloud(new pcl::PointCloud<pcl::PointXYZ>);
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

    struct CheckEquals {
      pcl::CropHull<pcl::PointXYZ> cropHullFilter;
      CheckEquals(pcl::CropHull<pcl::PointXYZ> const & cropHullFilter)
        : cropHullFilter(cropHullFilter)
      {}
      void check(
          pcl::PointCloud<pcl::PointXYZ>::ConstPtr expectedCloud,
          pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
          int idxOffset = 0)
      {
        cropHullFilter.setInputCloud(inputCloud);

        std::vector<int> filteredIndices;
        cropHullFilter.filter(filteredIndices);
        ASSERT_EQ (expectedCloud->size(), filteredIndices.size());
        std::sort(filteredIndices.begin(), filteredIndices.end());
        for (int i = 0; i < filteredIndices.size(); ++i)
        {
          ASSERT_EQ (idxOffset + i, filteredIndices[i]);
        }

        pcl::PointCloud<pcl::PointXYZ> filteredCloud;
        cropHullFilter.filter(filteredCloud);
        ASSERT_EQ (expectedCloud->size(), filteredCloud.size());
        for (int i = 0; i < expectedCloud->size(); ++i)
        {
          Eigen::Vector3f expectedPoint = expectedCloud->at(i).getVector3fMap();
          Eigen::Vector3f actualPoint = filteredCloud.at(i).getVector3fMap();
          EXPECT_NEAR((expectedPoint - actualPoint).norm(), 0.0, 1e-5);
        }
      }
    };

    {
      CheckEquals checker(cropHullFilter);
      checker.check(insideCubeCloud, insideCubeCloud);
      checker.check(emptyCloud, outsideCubeCloud);
      checker.check(insideCubeCloud, mixedCubeCloud);
    }

    {
      cropHullFilter.setCropOutside(false);
      CheckEquals checker(cropHullFilter);
      checker.check(emptyCloud, insideCubeCloud);
      checker.check(outsideCubeCloud, outsideCubeCloud);
      checker.check(outsideCubeCloud, mixedCubeCloud, insideCubeCloud->size());
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_2dsquare)
{
  pcl::CropHull<pcl::PointXYZ> cropHullFilter;
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    inputCloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 0.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 1.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.1f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 0.1f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 0.0f, 0.1f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 1.0f, 0.1f));

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
  auto insidePointGenerator = [&rd, &gen] () {
    return pcl::PointXYZ(rd(gen), rd(gen), 1.0);
  };
  auto outsidePointGenerator = [&rd, &gen] () {
    return pcl::PointXYZ(rd(gen) + 2., rd(gen) + 2., rd(gen) + 2.);
  };

  complexTest(cropHullFilter, insidePointGenerator, outsidePointGenerator);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, issue_1657_CropHull3d_not_cropping_inside)
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  pcl::CropHull<pcl::PointXYZ> cropHullFilter;
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
  pcl::CropHull<pcl::PointXYZ> cropHullFilter;
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    inputCloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 1.0f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 1.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 0.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 0.0f, 1.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 1.0f, 0.0f));
    inputCloud->push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));

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
  auto insidePointGenerator = [&rd, &gen] () {
    return pcl::PointXYZ(rd(gen), rd(gen), rd(gen));
  };
  auto outsidePointGenerator = [&rd, &gen] () {
    return pcl::PointXYZ(rd(gen) + 2., rd(gen) + 2., rd(gen) + 2.);
  };

  complexTest(cropHullFilter, insidePointGenerator, outsidePointGenerator);
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
