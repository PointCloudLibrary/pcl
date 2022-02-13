/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <random>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ>::Ptr cloud_;
PointCloud<PointXYZ>::Ptr cloud_t_;
KdTree<PointXYZ>::Ptr tree_;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr another_cloud_;
pcl::PointCloud<pcl::Normal>::Ptr normals_;
pcl::PointCloud<pcl::Normal>::Ptr another_normals_;

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingRGBTest, Segment)
{
  RegionGrowingRGB<pcl::PointXYZRGB> rg;

  rg.setInputCloud (colored_cloud);
  rg.setDistanceThreshold (10);
  rg.setRegionColorThreshold (5);
  rg.setPointColorThreshold (6);
  rg.setMinClusterSize (20);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_NE (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, Segment)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputCloud (cloud_);
  rg.setInputNormals (normals_);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_NE (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithoutCloud)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputNormals (normals_);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithoutNormals)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputCloud (cloud_);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentEmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr empty_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputCloud (empty_cloud);
  rg.setInputNormals (empty_normals);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithDifferentNormalAndCloudSize)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputCloud (another_cloud_);
  rg.setInputNormals (normals_);

  const auto first_cloud_size = cloud_->size ();
  const auto second_cloud_size = another_cloud_->size ();
  ASSERT_NE (first_cloud_size, second_cloud_size);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);

  rg.setInputCloud (cloud_);
  rg.setInputNormals (another_normals_);

  rg.extract (clusters);
  num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithWrongThresholdParameters)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setInputCloud (cloud_);
  rg.setInputNormals (normals_);

  rg.setNumberOfNeighbours (0);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);
  auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);

  rg.setNumberOfNeighbours (30);
  rg.setResidualTestFlag (true);
  rg.setResidualThreshold (-10.0);

  rg.extract (clusters);
  num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);

  rg.setCurvatureTestFlag (true);
  rg.setCurvatureThreshold (-10.0f);

  rg.extract (clusters);
  num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentFromPoint)
{
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;

  pcl::PointIndices cluster;
  rg.getSegmentFromPoint (0, cluster);
  EXPECT_EQ (0, cluster.indices.size ());

  rg.setInputCloud (cloud_);
  rg.setInputNormals (normals_);
  rg.getSegmentFromPoint(0, cluster);
  EXPECT_NE (0, cluster.indices.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, Segment)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;

  pcl::PointXYZ object_center;
  double radius = 0.0;
  double sigma = 0.0;
  double source_weight = 0.0;
  unsigned int neighbor_number = 0;

  object_center.x = -36.01f;
  object_center.y = -64.73f;
  object_center.z = -6.18f;
  radius = 3.8003856;
  sigma = 0.25;
  source_weight = 0.8;
  neighbor_number = 14;

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  foreground_points->points.push_back (object_center);

  mcSeg.setForegroundPoints (foreground_points);
  mcSeg.setInputCloud (another_cloud_);
  mcSeg.setRadius (radius);
  mcSeg.setSigma (sigma);
  mcSeg.setSourceWeight (source_weight);
  mcSeg.setNumberOfNeighbours (neighbor_number);

  std::vector <pcl::PointIndices> clusters;
  mcSeg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (2, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithoutForegroundPoints)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (another_cloud_);
  mcSeg.setRadius (3.8003856);

  std::vector <pcl::PointIndices> clusters;
  mcSeg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithoutCloud)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;

  std::vector <pcl::PointIndices> clusters;
  mcSeg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentEmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (empty_cloud);

  std::vector <pcl::PointIndices> clusters;
  mcSeg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (0, num_of_segments);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithWrongParameters)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (another_cloud_);
  pcl::PointXYZ object_center;
  object_center.x = -36.01f;
  object_center.y = -64.73f;
  object_center.z = -6.18f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  foreground_points->points.push_back (object_center);
  mcSeg.setForegroundPoints (foreground_points);

  unsigned int prev_neighbor_number = mcSeg.getNumberOfNeighbours ();
  EXPECT_LT (0, prev_neighbor_number);

  mcSeg.setNumberOfNeighbours (0);
  unsigned int curr_neighbor_number = mcSeg.getNumberOfNeighbours ();
  EXPECT_EQ (prev_neighbor_number, curr_neighbor_number);

  double prev_radius = mcSeg.getRadius ();
  EXPECT_LT (0.0, prev_radius);

  mcSeg.setRadius (0.0);
  double curr_radius = mcSeg.getRadius ();
  EXPECT_EQ (prev_radius, curr_radius);

  mcSeg.setRadius (-10.0);
  curr_radius = mcSeg.getRadius ();
  EXPECT_EQ (prev_radius, curr_radius);

  double prev_sigma = mcSeg.getSigma ();
  EXPECT_LT (0.0, prev_sigma);

  mcSeg.setSigma (0.0);
  double curr_sigma = mcSeg.getSigma ();
  EXPECT_EQ (prev_sigma, curr_sigma);

  mcSeg.setSigma (-10.0);
  curr_sigma = mcSeg.getSigma ();
  EXPECT_EQ (prev_sigma, curr_sigma);

  double prev_source_weight = mcSeg.getSourceWeight ();
  EXPECT_LT (0.0, prev_source_weight);

  mcSeg.setSourceWeight (0.0);
  double curr_source_weight = mcSeg.getSourceWeight ();
  EXPECT_EQ (prev_source_weight, curr_source_weight);

  mcSeg.setSourceWeight (-10.0);
  curr_source_weight = mcSeg.getSourceWeight ();
  EXPECT_EQ (prev_source_weight, curr_source_weight);

  mcSeg.setRadius (3.8003856);

  std::vector <pcl::PointIndices> clusters;
  mcSeg.extract (clusters);
  const auto num_of_segments = clusters.size ();
  EXPECT_EQ (2, num_of_segments);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (SegmentDifferences, Segmentation)
{
  SegmentDifferences<PointXYZ> sd;
  sd.setInputCloud (cloud_);
  sd.setDistanceThreshold (0.00005);

  // Set the target as itself
  sd.setTargetCloud (cloud_);

  PointCloud<PointXYZ> output;
  sd.segment (output);

  EXPECT_EQ (output.size (), 0);
  
  // Set a different target
  sd.setTargetCloud (cloud_t_);
  sd.segment (output);
  EXPECT_EQ (output.size (), 126);
  //savePCDFile ("./test/0-t.pcd", output);

  // Reverse
  sd.setInputCloud (cloud_t_);
  sd.setTargetCloud (cloud_);
  sd.segment (output);
  EXPECT_EQ (output.size (), 127);
  //savePCDFile ("./test/t-0.pcd", output);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractPolygonalPrism, Segmentation)
{
  PointCloud<PointXYZ>::Ptr hull (new PointCloud<PointXYZ>);
  hull->points.resize (5);

  for (std::size_t i = 0; i < hull->size (); ++i)
  {
    (*hull)[i].x = (*hull)[i].y = static_cast<float> (i);
    (*hull)[i].z = 0.0f;
  }

  ExtractPolygonalPrismData<PointXYZ> ex;
  ex.setInputCloud (cloud_);
  ex.setInputPlanarHull (hull);

  PointIndices output;
  ex.segment (output);

  EXPECT_EQ (output.indices.size (), 0);
}
#include <pcl/io/ply_io.h>
TEST (ExtractPolygonalPrism, two_rings)
{
  // prepare 2 rings
  PointCloud<PointXYZ>::Ptr ring(new PointCloud<PointXYZ>);
  { // use random
    std::random_device rd;
    std::mt19937 gen(rd());
    float rMin = 0.1, rMax = 0.25f;
    std::uniform_real_distribution<float> radiusDist(rMin, rMax);
    std::uniform_real_distribution<float> radianDist(-M_PI, M_PI);
    std::uniform_real_distribution<float> zDist(-0.01f, 0.01f);
    for (size_t i = 0; i < 1000; i++) {
      float radius = radiusDist(gen);
      float angle = radianDist(gen);
      float z = zDist(gen);
      PointXYZ point(cos(angle) * radius, sin(angle) * radius, z);
      ring->push_back(point);
    }
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  for (auto& point : ring->points) {
    auto left = point;
    auto rght = point;
    left.x -= 0.5f;
    rght.x += 0.5f;
    cloud->points.push_back(left);
    cloud->points.push_back(rght);
  }

  // find hull
  PointCloud<PointXYZ>::Ptr hullCloud(new PointCloud<PointXYZ>);
  std::vector<pcl::Vertices> rings;
  {
    pcl::ConcaveHull<PointXYZ> concaveHull;
    concaveHull.setInputCloud(cloud);
    concaveHull.setAlpha(0.05);
    concaveHull.reconstruct(*hullCloud, rings);
  }
  EXPECT_EQ(rings.size(), 4);

  // add more points before using prism
  size_t ringsPointCount = cloud->size();
  cloud->points.push_back({0, 0, 0});
  for (float a = -M_PI; a < M_PI; a += 0.05f) {
    float r = 1.f;
    cloud->points.push_back({r * cos(a), r * sin(a), 0});
  }

  // do prism
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ExtractPolygonalPrismData<PointXYZ> ex;
  {
    ex.setInputCloud(cloud);
    ex.setInputPlanarHull(hullCloud);
    ex.setHeightLimits(-1, 1);
    ex.setRings(rings);
    ex.segment(*inliers);
  }

  // check that almost all of the rings are in the prism.
  // *almost* because the points on the border is not well defined,
  // and may be inside or outside.
  EXPECT_NEAR(inliers->indices.size(), ringsPointCount, 100);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "This test requires three point clouds. The first one must be 'bun0.pcd'." << std::endl;
    std::cerr << "The second must be 'car6.pcd'. The last one must be 'colored_cloud.pcd'." << std::endl;
    std::cerr << "Please download and pass them in the specified order(including the path to them)." << std::endl;
    return (-1);
  }

  // Load a standard PCD file from disk
  PointCloud<PointXYZ> cloud, cloud_t, another_cloud;
  PointCloud<PointXYZRGB> colored_cloud_1;
  if (loadPCDFile (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (argv[2], another_cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `car6.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (argv[3], colored_cloud_1) < 0)
  {
    std::cerr << "Failed to read test file. Please download `colored_cloud.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  colored_cloud = colored_cloud_1.makeShared();

  // Tranpose the cloud
  cloud_t = cloud;
  for (auto& point: cloud_t)
    point.x += 0.01f;

  cloud_   = cloud.makeShared ();
  cloud_t_ = cloud_t.makeShared ();

  another_cloud_ = another_cloud.makeShared();
  normals_ = (new pcl::PointCloud<pcl::Normal>)->makeShared();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud_);
  normal_estimator.setKSearch(30);
  normal_estimator.compute(*normals_);

  another_normals_ = (new pcl::PointCloud<pcl::Normal>)->makeShared();
  normal_estimator.setInputCloud(another_cloud_);
  normal_estimator.setKSearch(30);
  normal_estimator.compute(*another_normals_);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
