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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <gtest/gtest.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>

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
/*TEST (RegionGrowingRGBTest, Segment)
{
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> rg;

  rg.setCloud (colored_cloud);
  rg.setDistanceThreshold (10);
  rg.setRegionColorThreshold (5);
  rg.setPointColorThreshold (6);
  rg.setMinPointNumber (20);

  rg.segmentPoints ();

  std::vector< std::vector<int> > segments;
  segments = rg.getSegments ();

  EXPECT_NE(0, segments.size());
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, Segment)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setCloud(cloud_);
  rg.setNormals(normals_);

  int num_of_segments = rg.segmentPoints();
  EXPECT_NE(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_NE(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithoutCloud)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setNormals(normals_);

  int num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithoutNormals)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setCloud(cloud_);

  int num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentEmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr empty_normals(new pcl::PointCloud<pcl::Normal>);

  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setCloud(empty_cloud);
  rg.setNormals(empty_normals);

  int num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithDifferentNormalAndCloudSize)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setCloud(another_cloud_);
  rg.setNormals(normals_);

  int first_cloud_size = static_cast<int> (cloud_->points.size());
  int second_cloud_size = static_cast<int> (another_cloud_->points.size ());
  ASSERT_NE(first_cloud_size, second_cloud_size);

  int num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());

  rg.setCloud(cloud_);
  rg.setNormals(another_normals_);

  num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentWithWrongThresholdParameters)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;
  rg.setCloud(cloud_);
  rg.setNormals(normals_);

  rg.setNumberOfNeighbours(0);

  int num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  std::vector<std::vector<int> > segments;
  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());

  rg.setNumberOfNeighbours(30);
  rg.setResidualTest(true);
  rg.setResidualThreshold(-10.0);

  num_of_segments = rg.segmentPoints();
  EXPECT_EQ(0, num_of_segments);

  rg.setCurvatureTest(true);
  rg.setCurvatureThreshold(-10.0f);

  segments = rg.getSegments();
  EXPECT_EQ(0, segments.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RegionGrowingTest, SegmentFromPoint)
{
  pcl::RegionGrowing<pcl::PointXYZ> rg;

  std::vector<int> segment = rg.getSegmentFromPoint(0);
  EXPECT_EQ(0, segment.size());

  rg.setCloud(cloud_);
  rg.setNormals(normals_);
  segment = rg.getSegmentFromPoint(0);
  EXPECT_NE(0, segment.size());
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

  object_center.x = -36.01;
  object_center.y = -64.73;
  object_center.z = -6.18;
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
  mcSeg.setNeighbourNumber (neighbor_number);

  bool success = mcSeg.segmentPoints ();
  EXPECT_EQ (true, success);

  std::vector<int> labels = mcSeg.getSegments ();
  int number_of_points = static_cast<int> (another_cloud_->points.size ());
  int number_of_labels = static_cast<int> (labels.size ());
  EXPECT_EQ (number_of_points, number_of_labels);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithoutForegroundPoints)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (another_cloud_);
  mcSeg.setRadius (3.8003856);

  bool success = mcSeg.segmentPoints ();
  EXPECT_EQ (false, success);

  std::vector<int> labels = mcSeg.getSegments ();
  int number_of_labels = static_cast<int> (labels.size ());
  EXPECT_EQ (0, number_of_labels);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithoutCloud)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;

  bool success = mcSeg.segmentPoints ();
  EXPECT_EQ (false, success);

  std::vector<int> labels = mcSeg.getSegments ();
  int number_of_labels = static_cast<int> (labels.size ());
  EXPECT_EQ (0, number_of_labels);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentEmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (empty_cloud);

  bool success = mcSeg.segmentPoints ();
  EXPECT_EQ (false, success);

  std::vector<int> labels = mcSeg.getSegments ();
  int number_of_labels = static_cast<int> (labels.size ());
  EXPECT_EQ (0, number_of_labels);
}

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MinCutSegmentationTest, SegmentWithWrongParameters)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> mcSeg;
  mcSeg.setInputCloud (another_cloud_);
  pcl::PointXYZ object_center;
  object_center.x = -36.01;
  object_center.y = -64.73;
  object_center.z = -6.18;
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  foreground_points->points.push_back (object_center);
  mcSeg.setForegroundPoints (foreground_points);

  unsigned int prev_neighbor_number = mcSeg.getNeighbourNumber ();
  EXPECT_LT (0, prev_neighbor_number);

  mcSeg.setNeighbourNumber (0);
  unsigned int curr_neighbor_number = mcSeg.getNeighbourNumber ();
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
  bool success = mcSeg.segmentPoints ();
  EXPECT_EQ (true, success);
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

  EXPECT_EQ (static_cast<int> (output.points.size ()), 0);
  
  // Set a different target
  sd.setTargetCloud (cloud_t_);
  sd.segment (output);
  EXPECT_EQ (static_cast<int> (output.points.size ()), 126);
  //savePCDFile ("./test/0-t.pcd", output);

  // Reverse
  sd.setInputCloud (cloud_t_);
  sd.setTargetCloud (cloud_);
  sd.segment (output);
  EXPECT_EQ (static_cast<int> (output.points.size ()), 127);
  //savePCDFile ("./test/t-0.pcd", output);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractPolygonalPrism, Segmentation)
{
  PointCloud<PointXYZ>::Ptr hull (new PointCloud<PointXYZ>);
  hull->points.resize (5);

  for (size_t i = 0; i < hull->points.size (); ++i)
  {
    hull->points[i].x = hull->points[i].y = static_cast<float> (i);
    hull->points[i].z = 0.0f;
  }

  ExtractPolygonalPrismData<PointXYZ> ex;
  ex.setInputCloud (cloud_);
  ex.setInputPlanarHull (hull);

  PointIndices output;
  ex.segment (output);

  EXPECT_EQ (static_cast<int> (output.indices.size ()), 0);
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
  for (size_t i = 0; i < cloud.points.size (); ++i)
    cloud_t.points[i].x += 0.01f;

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
