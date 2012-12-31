/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;


PointCloud2::Ptr cloud_blob (new PointCloud2);
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
vector<int> indices_;

//pcl::IndicesConstPtr indices;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractIndicesSelf, Filters)
{
  // Test the PointCloud<PointT> method
  ExtractIndices<PointXYZ> ei;
  boost::shared_ptr<vector<int> > indices (new vector<int> (2));
  (*indices)[0] = 0;
  (*indices)[1] = static_cast<int> (cloud->points.size ()) - 1;

  PointCloud<PointXYZ>::Ptr output (new PointCloud<PointXYZ>);
  ei.setInputCloud (cloud);
  ei.setIndices (indices);
  ei.filter (*output);

  EXPECT_EQ (int (output->points.size ()), 2);
  EXPECT_EQ (int (output->width), 2);
  EXPECT_EQ (int (output->height), 1);

  EXPECT_EQ (cloud->points[0].x, output->points[0].x);
  EXPECT_EQ (cloud->points[0].y, output->points[0].y);
  EXPECT_EQ (cloud->points[0].z, output->points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output->points[1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output->points[1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output->points[1].z);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractIndices, Filters)
{
  // Test the PointCloud<PointT> method
  ExtractIndices<PointXYZ> ei;
  boost::shared_ptr<vector<int> > indices (new vector<int> (2));
  (*indices)[0] = 0;
  (*indices)[1] = static_cast<int> (cloud->points.size ()) - 1;

  PointCloud<PointXYZ> output;
  ei.setInputCloud (cloud);
  ei.setIndices (indices);
  ei.filter (output);

  EXPECT_EQ (int (output.points.size ()), 2);
  EXPECT_EQ (int (output.width), 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[1].z);

  ei.setNegative (true);
  ei.filter (output);

  EXPECT_EQ (output.points.size (), cloud->points.size () - 2);
  EXPECT_EQ (output.width, cloud->points.size () - 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[1].x, output.points[0].x);
  EXPECT_EQ (cloud->points[1].y, output.points[0].y);
  EXPECT_EQ (cloud->points[1].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 2].x, output.points[output.points.size () - 1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 2].y, output.points[output.points.size () - 1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 2].z, output.points[output.points.size () - 1].z);

  // Test the sensor_msgs::PointCloud2 method
  ExtractIndices<PointCloud2> ei2;

  PointCloud2 output_blob;
  ei2.setInputCloud (cloud_blob);
  ei2.setIndices (indices);
  ei2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 2);
  EXPECT_EQ (int (output.width), 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[1].z);

  ei2.setNegative (true);
  ei2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (output.points.size (), cloud->points.size () - 2);
  EXPECT_EQ (output.width, cloud->points.size () - 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[1].x, output.points[0].x);
  EXPECT_EQ (cloud->points[1].y, output.points[0].y);
  EXPECT_EQ (cloud->points[1].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 2].x, output.points[output.points.size () - 1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 2].y, output.points[output.points.size () - 1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 2].z, output.points[output.points.size () - 1].z);

  // Test setNegative on empty datasets
  PointCloud<PointXYZ> empty, result;
  ExtractIndices<PointXYZ> eie;
  eie.setInputCloud (empty.makeShared ());
  eie.setNegative (false);
  eie.filter (result);

  EXPECT_EQ (int (result.points.size ()), 0);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (int (result.points.size ()), 0);

  boost::shared_ptr<vector<int> > idx (new vector<int> (10));
  eie.setIndices (idx);
  eie.setNegative (false);
  eie.filter (result);
  EXPECT_EQ (int (result.points.size ()), 0);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (int (result.points.size ()), 0);

  empty.points.resize (10);
  empty.width = 10; empty.height = 1;
  eie.setInputCloud (empty.makeShared ());
  for (int i = 0; i < 10; ++i)
    (*idx)[i] = i;
  eie.setIndices (idx);
  eie.setNegative (false);
  eie.filter (result);
  EXPECT_EQ (int (result.points.size ()), 10);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (int (result.points.size ()), 0);

  /*
  PointCloud<PointXYZ> sc, scf;
  sc.points.resize (5); sc.width = 5; sc.height = 1; sc.is_dense = true;
  for (int i = 0; i < 5; i++)
  {
    sc.points[i].x = sc.points[i].z = 0;
    sc.points[i].y = i;
  }
  PassThrough<PointXYZ> ps;
  ps.setInputCloud (sc.makeShared ());
  ps.setFilterFieldName ("y");
  ps.setFilterLimits (0.99, 2.01);
  for (int i = 0; i < 2; i++)
  {
    ps.setFilterLimitsNegative ((bool)i);
    ps.filter (scf);
    std::cerr << scf.points.size () << std::endl;
    for (size_t j = 0; j < scf.points.size (); ++j)
      std::cerr << scf.points[j] << std::endl;
  }
  */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PassThrough, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;
  PassThrough<PointXYZ> pt;

  pt.setInputCloud (cloud);
  pt.filter (output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0.05f, 0.1f);
  pt.filter (output);

  EXPECT_EQ (int (output.points.size ()), 42);
  EXPECT_EQ (int (output.width), 42);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output.points[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output.points[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output.points[41].z, 0.052133, 1e-5);

  pt.setFilterLimitsNegative (true);
  pt.filter (output);

  EXPECT_EQ (int (output.points.size ()), 355);
  EXPECT_EQ (int (output.width), 355);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output.points[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output.points[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output.points[354].z, -0.0444, 1e-5);

  PassThrough<PointXYZ> pt_(true);

  pt_.setInputCloud (cloud);
  pt_.filter (output);

  EXPECT_EQ (pt_.getRemovedIndices()->size(), 0);
  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt_.setFilterFieldName ("z");
  pt_.setFilterLimits (0.05f, 0.1f);
  pt_.filter (output);

  EXPECT_EQ (int (output.points.size ()), 42);
  EXPECT_EQ (int (output.width), 42);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size ()-pt_.getRemovedIndices()->size());

  EXPECT_NEAR (output.points[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output.points[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output.points[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output.points[41].z, 0.052133, 1e-5);

  pt_.setFilterLimitsNegative (true);
  pt_.filter (output);

  EXPECT_EQ (int (output.points.size ()), 355);
  EXPECT_EQ (int (output.width), 355);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size ()-pt_.getRemovedIndices()->size());

  EXPECT_NEAR (output.points[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output.points[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output.points[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output.points[354].z, -0.0444, 1e-5);

  // Test the keep organized structure
  pt.setUserFilterValue (std::numeric_limits<float>::quiet_NaN ());
  pt.setFilterFieldName ("");
  pt.filter (output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (output.is_dense, cloud->is_dense);
  EXPECT_NEAR (output.points[0].x, cloud->points[0].x, 1e-5);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, cloud->points[cloud->points.size () - 1].x, 1e-5);

  pt.setFilterFieldName ("z");
  pt.setFilterLimitsNegative (false);
  pt.setKeepOrganized (true);
  pt.filter (output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (bool (output.is_dense), false); // NaN was set as a user filter value

  if (!pcl_isnan (output.points[0].x)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[0].y)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[0].z)) EXPECT_EQ (1, 0);

  if (!pcl_isnan (output.points[41].x)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[41].y)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[41].z)) EXPECT_EQ (1, 0);

  pt.setFilterLimitsNegative (true);
  pt.filter (output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (bool (output.is_dense), false); // NaN was set as a user filter value

  EXPECT_NEAR (output.points[0].x, cloud->points[0].x, 1e-5);
  EXPECT_NEAR (output.points[0].y, cloud->points[0].y, 1e-5);
  EXPECT_NEAR (output.points[0].z, cloud->points[0].z, 1e-5);

  EXPECT_NEAR (output.points[41].x, cloud->points[41].x, 1e-5);
  EXPECT_NEAR (output.points[41].y, cloud->points[41].y, 1e-5);
  EXPECT_NEAR (output.points[41].z, cloud->points[41].z, 1e-5);

  // Test the PointCloud2 method
  PassThrough<PointCloud2> pt2;

  PointCloud2 output_blob;
  pt2.setInputCloud (cloud_blob);
  pt2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt2.setFilterFieldName ("z");
  pt2.setFilterLimits (0.05, 0.1);
  pt2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 42);
  EXPECT_EQ (int (output.width), 42);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output.points[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output.points[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output.points[41].z, 0.052133, 1e-5);

  pt2.setFilterLimitsNegative (true);
  pt2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 355);
  EXPECT_EQ (int (output.width), 355);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output.points[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output.points[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output.points[354].z, -0.0444, 1e-5);

  PassThrough<PointCloud2> pt2_(true);
  pt2_.setInputCloud (cloud_blob);
  pt2_.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (pt2_.getRemovedIndices()->size(), 0);
  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt2_.setFilterFieldName ("z");
  pt2_.setFilterLimits (0.05, 0.1);
  pt2_.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 42);
  EXPECT_EQ (int (output.width), 42);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size ()-pt2_.getRemovedIndices()->size());

  EXPECT_NEAR (output.points[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output.points[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output.points[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output.points[41].z, 0.052133, 1e-5);

  pt2_.setFilterLimitsNegative (true);
  pt2_.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 355);
  EXPECT_EQ (int (output.width), 355);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size ()-pt2_.getRemovedIndices()->size());

  EXPECT_NEAR (output.points[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output.points[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output.points[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output.points[354].z, -0.0444, 1e-5);

  // Test the keep organized structure
  pt2.setUserFilterValue (std::numeric_limits<float>::quiet_NaN ());
  pt2.setFilterFieldName ("");
  pt2.filter (output_blob);
  fromROSMsg (output_blob, output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (output.is_dense, cloud->is_dense);
  EXPECT_NEAR (output.points[0].x, cloud->points[0].x, 1e-5);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, cloud->points[cloud->points.size () - 1].x, 1e-5);

  pt2.setFilterFieldName ("z");
  pt2.setFilterLimitsNegative (false);
  pt2.setKeepOrganized (true);
  pt2.filter (output_blob);
  fromROSMsg (output_blob, output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (bool (output.is_dense), false); // NaN was set as a user filter value

  if (!pcl_isnan (output.points[0].x)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[0].y)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[0].z)) EXPECT_EQ (1, 0);

  if (!pcl_isnan (output.points[41].x)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[41].y)) EXPECT_EQ (1, 0);
  if (!pcl_isnan (output.points[41].z)) EXPECT_EQ (1, 0);

  pt2.setFilterLimitsNegative (true);
  pt2.filter (output_blob);
  fromROSMsg (output_blob, output);

  EXPECT_EQ (output.points.size (), cloud->points.size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (bool (output.is_dense), false); // NaN was set as a user filter value

  EXPECT_NEAR (output.points[0].x, cloud->points[0].x, 1e-5);
  EXPECT_NEAR (output.points[0].y, cloud->points[0].y, 1e-5);
  EXPECT_NEAR (output.points[0].z, cloud->points[0].z, 1e-5);

  EXPECT_NEAR (output.points[41].x, cloud->points[41].x, 1e-5);
  EXPECT_NEAR (output.points[41].y, cloud->points[41].y, 1e-5);
  EXPECT_NEAR (output.points[41].z, cloud->points[41].z, 1e-5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;
  VoxelGrid<PointXYZ> grid;

  grid.setLeafSize (0.02f, 0.02f, 0.02f);
  grid.setInputCloud (cloud);
  grid.filter (output);

  EXPECT_EQ (int (output.points.size ()), 103);
  EXPECT_EQ (int (output.width), 103);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  grid.setFilterFieldName ("z");
  grid.setFilterLimits (0.05, 0.1);
  grid.filter (output);

  EXPECT_EQ (int (output.points.size ()), 14);
  EXPECT_EQ (int (output.width), 14);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output.points[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output.points[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output.points[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output.points[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output.points[13].z, 0.051333, 1e-4);

  grid.setFilterLimitsNegative (true);
  grid.setSaveLeafLayout(true);
  grid.filter (output);

  EXPECT_EQ (int (output.points.size ()), 100);
  EXPECT_EQ (int (output.width), 100);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  //EXPECT_NEAR (output.points[0].x, -0.070192, 1e-4);
  //EXPECT_NEAR (output.points[0].y, 0.17653, 1e-4);
  //EXPECT_NEAR (output.points[0].z, -0.048774, 1e-4);

  //EXPECT_NEAR (output.points[99].x, -0.068948, 1e-4);
  //EXPECT_NEAR (output.points[99].y, 0.1447, 1e-4);
  //EXPECT_NEAR (output.points[99].z, 0.042178, 1e-4);

  // centroids should be identified correctly
  EXPECT_EQ (grid.getCentroidIndex (output.points[0]), 0);
  EXPECT_EQ (grid.getCentroidIndex (output.points[99]), 99);
  EXPECT_EQ (grid.getCentroidIndexAt (grid.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx = grid.getCentroidIndex (cloud->points[195]);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (fabs (output.points[centroidIdx].x - cloud->points[195].x), 0.02);
  EXPECT_LE (fabs (output.points[centroidIdx].y - cloud->points[195].y), 0.02);
  EXPECT_LE (fabs (output.points[centroidIdx].z - cloud->points[195].z), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid.getNeighborCentroidIndices (output.points[0], Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid.getNeighborCentroidIndices (output.points[99], Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions = Eigen::Vector3i (0, 0, 1);
  vector<int> neighbors = grid.getNeighborCentroidIndices (cloud->points[195], directions);
  EXPECT_EQ (neighbors.size (), size_t (directions.cols ()));
  EXPECT_NE (neighbors.at (0), -1);
  EXPECT_LE (fabs (output.points[neighbors.at (0)].x - output.points[centroidIdx].x), 0.02);
  EXPECT_LE (fabs (output.points[neighbors.at (0)].y - output.points[centroidIdx].y), 0.02);
  EXPECT_LE ( output.points[neighbors.at (0)].z - output.points[centroidIdx].z, 0.02 * 2);

  // Test the sensor_msgs::PointCloud2 method
  VoxelGrid<PointCloud2> grid2;

  PointCloud2 output_blob;

  grid2.setLeafSize (0.02f, 0.02f, 0.02f);
  grid2.setInputCloud (cloud_blob);
  grid2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 103);
  EXPECT_EQ (int (output.width), 103);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  grid2.setFilterFieldName ("z");
  grid2.setFilterLimits (0.05, 0.1);
  grid2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 14);
  EXPECT_EQ (int (output.width), 14);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output.points[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output.points[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output.points[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output.points[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output.points[13].z, 0.051333, 1e-4);

  grid2.setFilterLimitsNegative (true);
  grid2.setSaveLeafLayout(true);
  grid2.filter (output_blob);

  fromROSMsg (output_blob, output);

  EXPECT_EQ (int (output.points.size ()), 100);
  EXPECT_EQ (int (output.width), 100);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  //EXPECT_NEAR (output.points[0].x, -0.070192, 1e-4);
  //EXPECT_NEAR (output.points[0].y, 0.17653, 1e-4);
  //EXPECT_NEAR (output.points[0].z, -0.048774, 1e-4);

  //EXPECT_NEAR (output.points[99].x, -0.068948, 1e-4);
  //EXPECT_NEAR (output.points[99].y, 0.1447, 1e-4);
  //EXPECT_NEAR (output.points[99].z, 0.042178, 1e-4);

  // centroids should be identified correctly
  EXPECT_EQ (grid2.getCentroidIndex (output.points[0].x, output.points[0].y, output.points[0].z), 0);
  EXPECT_EQ (grid2.getCentroidIndex (output.points[99].x, output.points[99].y, output.points[99].z), 99);
  EXPECT_EQ (grid2.getCentroidIndexAt (grid2.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx2 = grid2.getCentroidIndex (0.048722f, 0.073760f, 0.017434f);
  EXPECT_NE (centroidIdx2, -1);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (fabs (output.points[centroidIdx2].x - 0.048722), 0.02);
  EXPECT_LE (fabs (output.points[centroidIdx2].y - 0.073760), 0.02);
  EXPECT_LE (fabs (output.points[centroidIdx2].z - 0.017434), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output.points[0].x, output.points[0].y, output.points[0].z, Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output.points[99].x, output.points[99].y, output.points[99].z, Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions2 = Eigen::Vector3i (0, 0, 1);
  vector<int> neighbors2 = grid2.getNeighborCentroidIndices (0.048722f, 0.073760f, 0.017434f, directions2);
  EXPECT_EQ (neighbors2.size (), size_t (directions2.cols ()));
  EXPECT_NE (neighbors2.at (0), -1);
  EXPECT_LE (fabs (output.points[neighbors2.at (0)].x - output.points[centroidIdx2].x), 0.02);
  EXPECT_LE (fabs (output.points[neighbors2.at (0)].y - output.points[centroidIdx2].y), 0.02);
  EXPECT_LE (output.points[neighbors2.at (0)].z - output.points[centroidIdx2].z, 0.02 * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid_RGB, Filters)
{
  PointCloud2 cloud_rgb_blob_;
  PointCloud2::Ptr cloud_rgb_blob_ptr_;
  PointCloud<PointXYZRGB> cloud_rgb_;
  PointCloud<PointXYZRGB>::Ptr cloud_rgb_ptr_;

  int col_r[] = {214, 193, 180, 164, 133, 119, 158, 179, 178, 212};
  int col_g[] = {10, 39, 219, 231, 142, 169, 84, 158, 139, 214};
  int col_b[] = {101, 26, 46, 189, 211, 154, 246, 16, 139, 153};
  float ave_r = 0.0f;
  float ave_b = 0.0f;
  float ave_g = 0.0f;
  for (int i = 0; i < 10; ++i)
  {
    ave_r += static_cast<float> (col_r[i]);
    ave_g += static_cast<float> (col_g[i]);
    ave_b += static_cast<float> (col_b[i]);
  }
  ave_r /= 10.0f;
  ave_g /= 10.0f;
  ave_b /= 10.0f;

  for (int i = 0; i < 10; ++i)
  {
    PointXYZRGB pt;
    int rgb = (col_r[i] << 16) | (col_g[i] << 8) | col_b[i];
    pt.x = 0.0f;
    pt.y = 0.0f;
    pt.z = 0.0f;
    pt.rgb = *reinterpret_cast<float*> (&rgb);
    cloud_rgb_.points.push_back (pt);
  }

  toROSMsg (cloud_rgb_, cloud_rgb_blob_);
  cloud_rgb_blob_ptr_.reset (new PointCloud2 (cloud_rgb_blob_));
  cloud_rgb_ptr_.reset (new PointCloud<PointXYZRGB> (cloud_rgb_));

  PointCloud<PointXYZRGB> output_rgb;
  VoxelGrid<PointXYZRGB> grid_rgb;

  grid_rgb.setLeafSize (0.03f, 0.03f, 0.03f);
  grid_rgb.setInputCloud (cloud_rgb_ptr_);
  grid_rgb.filter (output_rgb);

  EXPECT_EQ (int (output_rgb.points.size ()), 1);
  EXPECT_EQ (int (output_rgb.width), 1);
  EXPECT_EQ (int (output_rgb.height), 1);
  EXPECT_EQ (bool (output_rgb.is_dense), true);
  {
    int rgb;
    int r,g,b;
    memcpy (&rgb, &(output_rgb.points[0].rgb), sizeof(int));
    r = (rgb >> 16) & 0xFF; g = (rgb >> 8 ) & 0xFF; b = (rgb >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgb.points[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb.points[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb.points[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
  }

  VoxelGrid<PointCloud2> grid2;
  PointCloud2 output_rgb_blob;

  grid2.setLeafSize (0.03f, 0.03f, 0.03f);
  grid2.setInputCloud (cloud_rgb_blob_ptr_);
  grid2.filter (output_rgb_blob);

  fromROSMsg (output_rgb_blob, output_rgb);

  EXPECT_EQ (int (output_rgb.points.size ()), 1);
  EXPECT_EQ (int (output_rgb.width), 1);
  EXPECT_EQ (int (output_rgb.height), 1);
  EXPECT_EQ (bool (output_rgb.is_dense), true);
  {
    int rgb;
    int r,g,b;
    memcpy (&rgb, &(output_rgb.points[0].rgb), sizeof(int));
    r = (rgb >> 16) & 0xFF; g = (rgb >> 8 ) & 0xFF; b = (rgb >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgb.points[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb.points[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb.points[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
  }
}

#if 0
////////////////////////////////////////////////////////////////////////////////
float getRandomNumber (float max = 1.0, float min = 0.0)
{
  return (max - min) * float(rand ()) / float (RAND_MAX) + min;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid_XYZNormal, Filters)
{
  PointCloud2 cloud_blob_;
  PointCloud2::Ptr cloud_blob_ptr_;

  PointCloud<PointNormal>::Ptr input (new PointCloud<PointNormal>);
  PointCloud<PointNormal> output;
  input->reserve (16);
  input->is_dense = false;
  
  PointNormal point;
  PointNormal ground_truth[2][2][2];
  for (unsigned zIdx = 0; zIdx < 2; ++zIdx)
  {
    // x = 0 -> same two positions
    for (unsigned yIdx = 0; yIdx < 2; ++yIdx)
    {
      for (unsigned xIdx = 0; xIdx < 2; ++xIdx)
      {
        // y = 0, z = 0 -> parallel normals opposite direction
        // y = 0, z = 1 -> one normal is NaN
        // y = 1, z = 0 -> orthogonal normals
        // y = 1, z = 1 -> random normals
        PointNormal& voxel = ground_truth [xIdx][yIdx][zIdx];
        
        point.x = xIdx * 1.99;
        point.y = yIdx * 1.99;
        point.z = zIdx * 1.99;
        point.normal_x = getRandomNumber (1.0, -1.0);
        point.normal_y = getRandomNumber (1.0, -1.0);
        point.normal_z = getRandomNumber (1.0, -1.0);
        
        float norm = 1.0f / sqrt (point.normal_x * point.normal_x + point.normal_y * point.normal_y + point.normal_z * point.normal_z );
        point.normal_x *= norm;
        point.normal_y *= norm;
        point.normal_z *= norm;
        
//        std::cout << "adding point: " << point.x << " , " << point.y << " , " << point.z
//                  << " -- " << point.normal_x << " , " << point.normal_y << " , " << point.normal_z << std::endl;
        input->push_back (point);
        
        voxel = point;
        
        if (xIdx != 0)
        {
          point.x = getRandomNumber (0.99) + float (xIdx);
          point.y = getRandomNumber (0.99) + float (yIdx);
          point.z = getRandomNumber (0.99) + float (zIdx);
        }
        if (yIdx == 0 && zIdx == 0) // opposite normals
        {
          point.normal_x *= -1.0;
          point.normal_y *= -1.0;
          point.normal_z *= -1.0;
        }
        else if (yIdx == 0 && zIdx == 1) // second normal is nan
        {
          point.normal_x = std::numeric_limits<float>::quiet_NaN ();
          point.normal_y = std::numeric_limits<float>::quiet_NaN ();
          point.normal_z = std::numeric_limits<float>::quiet_NaN ();
        }
        else if (yIdx == 1 && zIdx == 0) // orthogonal
        {
          point.normal_x = voxel.normal_y - voxel.normal_z;
          point.normal_y = voxel.normal_z - voxel.normal_x;
          point.normal_z = voxel.normal_x - voxel.normal_y;
        }
        else if (yIdx == 1 && zIdx == 1) // random
        {
          point.normal_x = getRandomNumber (1.0, -1.0);
          point.normal_y = getRandomNumber (1.0, -1.0);
          point.normal_z = getRandomNumber (1.0, -1.0);
        }

        voxel.x += point.x;
        voxel.y += point.y;
        voxel.z += point.z;
        
        voxel.x *= 0.5;
        voxel.y *= 0.5;
        voxel.z *= 0.5;
        
        if (yIdx == 0 && zIdx == 0)
        {
          voxel.normal_x = std::numeric_limits<float>::quiet_NaN ();
          voxel.normal_y = std::numeric_limits<float>::quiet_NaN ();
          voxel.normal_z = std::numeric_limits<float>::quiet_NaN ();
        }
        else if (pcl_isfinite (point.normal_x))
        {
          float norm = 1.0f / sqrt (point.normal_x * point.normal_x + point.normal_y * point.normal_y + point.normal_z * point.normal_z );
          point.normal_x *= norm;
          point.normal_y *= norm;
          point.normal_z *= norm;
          
          voxel.normal_x += point.normal_x;
          voxel.normal_y += point.normal_y;
          voxel.normal_z += point.normal_z;
          
          norm = 1.0f / sqrt (voxel.normal_x * voxel.normal_x + voxel.normal_y * voxel.normal_y + voxel.normal_z * voxel.normal_z );
          
          voxel.normal_x *= norm;
          voxel.normal_y *= norm;
          voxel.normal_z *= norm;
        }
//        std::cout << "adding point: " << point.x << " , " << point.y << " , " << point.z
//                  << " -- " << point.normal_x << " , " << point.normal_y << " , " << point.normal_z << std::endl;
        input->push_back (point);
//        std::cout << "voxel: " << voxel.x << " , " << voxel.y << " , " << voxel.z
//                  << " -- " << voxel.normal_x << " , " << voxel.normal_y << " , " << voxel.normal_z << std::endl;
        
      }
    }
  }
    
  VoxelGrid<PointNormal> grid;
  grid.setLeafSize (1.0f, 1.0f, 1.0f);
  grid.setFilterLimits (0.0, 2.0);
  grid.setInputCloud (input);
  grid.filter (output);
  
  // check the output
  for (unsigned idx = 0, zIdx = 0; zIdx < 2; ++zIdx)
  {
    for (unsigned yIdx = 0; yIdx < 2; ++yIdx)
    {
      for (unsigned xIdx = 0; xIdx < 2; ++xIdx, ++idx)
      {
        PointNormal& voxel = ground_truth [xIdx][yIdx][zIdx];
        PointNormal& point = output.points [idx];
        // check for point equalities
        EXPECT_EQ (voxel.x, point.x);
        EXPECT_EQ (voxel.y, point.y);
        EXPECT_EQ (voxel.z, point.z);
        
        if (pcl_isfinite(voxel.normal_x) || pcl_isfinite (point.normal_x))
        {
          EXPECT_EQ (voxel.normal_x, point.normal_x);
          EXPECT_EQ (voxel.normal_y, point.normal_y);
          EXPECT_EQ (voxel.normal_z, point.normal_z);
        }
      }
    }
  }
  
  toROSMsg (*input, cloud_blob_);
  cloud_blob_ptr_.reset (new PointCloud2 (cloud_blob_));
  
  VoxelGrid<PointCloud2> grid2;
  PointCloud2 output_blob;

  grid2.setLeafSize (1.0f, 1.0f, 1.0f);
  grid2.setFilterLimits (0.0f, 2.0f);
  grid2.setInputCloud (cloud_blob_ptr_);
  grid2.filter (output_blob);

  fromROSMsg (output_blob, output);
  // check the output
  for (unsigned idx = 0, zIdx = 0; zIdx < 2; ++zIdx)
  {
    for (unsigned yIdx = 0; yIdx < 2; ++yIdx)
    {
      for (unsigned xIdx = 0; xIdx < 2; ++xIdx, ++idx)
      {
        PointNormal& voxel = ground_truth [xIdx][yIdx][zIdx];
        PointNormal& point = output.points [idx];
        // check for point equalities
        EXPECT_EQ (voxel.x, point.x);
        EXPECT_EQ (voxel.y, point.y);
        EXPECT_EQ (voxel.z, point.z);
        
        if (pcl_isfinite(voxel.normal_x) || pcl_isfinite (point.normal_x))
        {
          EXPECT_EQ (voxel.normal_x, point.normal_x);
          EXPECT_EQ (voxel.normal_y, point.normal_y);
          EXPECT_EQ (voxel.normal_z, point.normal_z);
        }
      }
    }
  }
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGridCovariance, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;
  VoxelGridCovariance<PointXYZ> grid;

  grid.setLeafSize (0.02f, 0.02f, 0.02f);
  grid.setInputCloud (cloud);
  grid.filter (output);

  EXPECT_EQ (int (output.points.size ()), 23);
  EXPECT_EQ (int (output.width), 23);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.073619894683361053, 1e-4);
  EXPECT_NEAR (output.points[0].y,  0.16789889335632324,  1e-4);
  EXPECT_NEAR (output.points[0].z, -0.03018110990524292,  1e-4);

  EXPECT_NEAR (output.points[13].x, -0.06865914911031723, 1e-4);
  EXPECT_NEAR (output.points[13].y,  0.15243285894393921, 1e-4);
  EXPECT_NEAR (output.points[13].z,  0.03266800194978714, 1e-4);

  grid.setSaveLeafLayout (true);
  grid.filter (output);

  // centroids should be identified correctly
  EXPECT_EQ (grid.getCentroidIndex (output.points[0]), 0);
  EXPECT_EQ (grid.getCentroidIndex (output.points[17]), 17);
  EXPECT_EQ (grid.getCentroidIndexAt (grid.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 38 [-0.066091, 0.11973, 0.050881]
  int centroidIdx = grid.getCentroidIndex (cloud->points[38]);
  EXPECT_NE (centroidIdx, -1);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid.getNeighborCentroidIndices (output.points[0], Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid.getNeighborCentroidIndices (output.points[17], Eigen::MatrixXi::Zero(3,1))[0], 17);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions = Eigen::Vector3i (0, 1, 0);
  vector<int> neighbors = grid.getNeighborCentroidIndices (cloud->points[38], directions);
  EXPECT_EQ (neighbors.size (), size_t (directions.cols ()));
  EXPECT_NE (neighbors.at (0), -1);
  EXPECT_LE (fabs (output.points[neighbors.at (0)].x - output.points[centroidIdx].x), 0.02);
  EXPECT_LE (fabs (output.points[neighbors.at (0)].y - output.points[centroidIdx].y), 0.02);
  EXPECT_LE (output.points[neighbors.at (0)].z - output.points[centroidIdx].z, 0.02 * 2);

  // testing seach functions
  grid.setSaveLeafLayout (false);
  grid.filter (output, true);

  // testing k nearest neighbors search
  vector<VoxelGridCovariance<pcl::PointXYZ>::LeafConstPtr> leaves;
  vector<float> distances;
  grid.nearestKSearch (PointXYZ(0,1,0), 1, leaves, distances);

  EXPECT_EQ (int (leaves.size ()), 1);

  EXPECT_NEAR (leaves[0]->getMean ()[0], -0.0284687, 1e-4);
  EXPECT_NEAR (leaves[0]->getMean ()[1], 0.170919, 1e-4);
  EXPECT_NEAR (leaves[0]->getMean ()[2], -0.00765753, 1e-4);

  // testing radius search
  grid.radiusSearch (PointXYZ (0,0,0), 0.075, leaves, distances);

  EXPECT_EQ (leaves.size (), 3);

  EXPECT_NEAR (leaves[0]->getMean ()[0], 0.0322579, 1e-4);
  EXPECT_NEAR (leaves[0]->getMean ()[1], 0.0469001, 1e-4);
  EXPECT_NEAR (leaves[0]->getMean ()[2], 0.0328501, 1e-4);

  EXPECT_NEAR (leaves[1]->getMean ()[0], 0.0124421, 1e-4);
  EXPECT_NEAR (leaves[1]->getMean ()[1], 0.0524267, 1e-4);
  EXPECT_NEAR (leaves[1]->getMean ()[2], 0.0488767, 1e-4);

  EXPECT_NEAR (leaves[2]->getMean ()[0], -0.00936106, 1e-4);
  EXPECT_NEAR (leaves[2]->getMean ()[1], 0.0516725, 1e-4);
  EXPECT_NEAR (leaves[2]->getMean ()[2], 0.0508024, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ProjectInliers, Filters)
{
  // Test the PointCloud<PointT> method
  ProjectInliers<PointXYZ> proj;
  PointCloud<PointXYZ> output;

  ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (output);

  for (size_t i = 0; i < output.points.size (); ++i)
    EXPECT_NEAR (output.points[i].z, 0.0, 1e-4);

    // Test the sensor_msgs::PointCloud2 method
    ProjectInliers<PointCloud2> proj2;

    PointCloud2 output_blob;

    proj2.setModelType (SACMODEL_PLANE);
    proj2.setInputCloud (cloud_blob);
    proj2.setModelCoefficients (coefficients);
    proj2.filter (output_blob);

    fromROSMsg (output_blob, output);

    for (size_t i = 0; i < output.points.size (); ++i)
    EXPECT_NEAR (output.points[i].z, 0.0, 1e-4);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RadiusOutlierRemoval, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> cloud_out;
  // Remove outliers using a spherical density criterion
  RadiusOutlierRemoval<PointXYZ> outrem;
  outrem.setInputCloud (cloud);
  outrem.setRadiusSearch (0.02);
  outrem.setMinNeighborsInRadius (14);
  outrem.filter (cloud_out);

  EXPECT_EQ (int (cloud_out.points.size ()), 307);
  EXPECT_EQ (int (cloud_out.width), 307);
  EXPECT_EQ (bool (cloud_out.is_dense), true);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].z, -0.021299, 1e-4);

  // Test the sensor_msgs::PointCloud2 method
  PointCloud2 cloud_out2;
  RadiusOutlierRemoval<PointCloud2> outrem2;
  outrem2.setInputCloud (cloud_blob);
  outrem2.setRadiusSearch (0.02);
  outrem2.setMinNeighborsInRadius (15);
  outrem2.filter (cloud_out2);

  fromROSMsg (cloud_out2, cloud_out);
  EXPECT_EQ (int (cloud_out.points.size ()), 307);
  EXPECT_EQ (int (cloud_out.width), 307);
  EXPECT_EQ (bool (cloud_out.is_dense), true);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].z, -0.021299, 1e-4);

  // Remove outliers using a spherical density criterion
  RadiusOutlierRemoval<PointXYZ> outrem_(true);
  outrem_.setInputCloud (cloud);
  outrem_.setRadiusSearch (0.02);
  outrem_.setMinNeighborsInRadius (14);
  outrem_.filter (cloud_out);

  EXPECT_EQ (int (cloud_out.points.size ()), 307);
  EXPECT_EQ (int (cloud_out.width), 307);
  EXPECT_EQ (bool (cloud_out.is_dense), true);
  EXPECT_EQ (int (cloud_out.points.size ()), cloud->points.size ()-outrem_.getRemovedIndices()->size());

  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].z, -0.021299, 1e-4);

  // Test the sensor_msgs::PointCloud2 method
  RadiusOutlierRemoval<PointCloud2> outrem2_(true);
  outrem2_.setInputCloud (cloud_blob);
  outrem2_.setRadiusSearch (0.02);
  outrem2_.setMinNeighborsInRadius (15);
  outrem2_.filter (cloud_out2);

  fromROSMsg (cloud_out2, cloud_out);
  EXPECT_EQ (int (cloud_out.points.size ()), 307);
  EXPECT_EQ (int (cloud_out.width), 307);
  EXPECT_EQ (bool (cloud_out.is_dense), true);
  EXPECT_EQ (int (cloud_out.points.size ()), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());

  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out.points[cloud_out.points.size () - 1].z, -0.021299, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RandomSample, Filters)
{
  // Test the PointCloud<PointT> method
  // Randomly sample 10 points from cloud
  RandomSample<PointXYZ> sample;
  sample.setInputCloud (cloud);
  sample.setSample (10);

  // Indices
  vector<int> indices;
  sample.filter (indices);

  EXPECT_EQ (int (indices.size ()), 10);

  // Cloud
  PointCloud<PointXYZ> cloud_out;
  sample.filter(cloud_out);

  EXPECT_EQ (int (cloud_out.width), 10);
  EXPECT_EQ (int (indices.size ()), int (cloud_out.size ()));

  for (size_t i = 0; i < indices.size () - 1; ++i)
  {
    // Check that indices are sorted
    EXPECT_LT (indices[i], indices[i+1]);
    // Compare original points with sampled indices against sampled points
    EXPECT_NEAR (cloud->points[indices[i]].x, cloud_out.points[i].x, 1e-4);
    EXPECT_NEAR (cloud->points[indices[i]].y, cloud_out.points[i].y, 1e-4);
    EXPECT_NEAR (cloud->points[indices[i]].z, cloud_out.points[i].z, 1e-4);
  }


  // Test the sensor_msgs::PointCloud2 method
  // Randomly sample 10 points from cloud
  RandomSample<PointCloud2> sample2;
  sample2.setInputCloud (cloud_blob);
  sample2.setSample (10);

  // Indices
  vector<int> indices2;
  sample2.filter (indices2);

  EXPECT_EQ (int (indices2.size ()), 10);

  // Cloud
  PointCloud2 output_blob;
  sample2.filter (output_blob);

  fromROSMsg (output_blob, cloud_out);

  EXPECT_EQ (int (cloud_out.width), 10);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out.size ()));

  for (size_t i = 0; i < indices2.size () - 1; ++i)
  {
    // Check that indices are sorted
    EXPECT_LT (indices2[i], indices2[i+1]);
    // Compare original points with sampled indices against sampled points
    EXPECT_NEAR (cloud->points[indices2[i]].x, cloud_out.points[i].x, 1e-4);
    EXPECT_NEAR (cloud->points[indices2[i]].y, cloud_out.points[i].y, 1e-4);
    EXPECT_NEAR (cloud->points[indices2[i]].z, cloud_out.points[i].z, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CropBox, Filters)
{

  // PointT
  // -------------------------------------------------------------------------

  // Create cloud with center point and corner points
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());

  input->push_back (PointXYZ (0.0f, 0.0f, 0.0f));
  input->push_back (PointXYZ (0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, -0.9f));

  // Test the PointCloud<PointT> method
  CropBox<PointXYZ> cropBoxFilter (true);
  cropBoxFilter.setInputCloud (input);
  Eigen::Vector4f min_pt (-1.0f, -1.0f, -1.0f, 1.0f);
  Eigen::Vector4f max_pt (1.0f, 1.0f, 1.0f, 1.0f);

  // Cropbox slighlty bigger then bounding box of points
  cropBoxFilter.setMin (min_pt);
  cropBoxFilter.setMax (max_pt);

  // Indices
  vector<int> indices;
  cropBoxFilter.filter (indices);

  // Cloud
  PointCloud<PointXYZ> cloud_out;
  cropBoxFilter.filter (cloud_out);

  // Should contain all
  EXPECT_EQ (int (indices.size ()), 9);
  EXPECT_EQ (int (cloud_out.size ()), 9);
  EXPECT_EQ (int (cloud_out.width), 9);
  EXPECT_EQ (int (cloud_out.height), 1);

  IndicesConstPtr removed_indices;
  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 0);

  // Test setNegative
  PointCloud<PointXYZ> cloud_out_negative;
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 0);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 0);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate crop box up by 1
  cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 5);
  EXPECT_EQ (int (cloud_out.size ()), 5);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 4);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 4);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 4);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate crop box up by 45
  cropBoxFilter.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 1);
  EXPECT_EQ (int (cloud_out.size ()), 1);
  EXPECT_EQ (int (cloud_out.width), 1);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 8);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 8);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 8);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate point cloud by -45
  cropBoxFilter.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 3);
  EXPECT_EQ (int (cloud_out.size ()), 3);
  EXPECT_EQ (int (cloud_out.width), 3);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 6);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 6);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 6);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate point cloud down by -1
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, -45.0 * float (M_PI) / 180.0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 2);
  EXPECT_EQ (int (cloud_out.size ()), 2);
  EXPECT_EQ (int (cloud_out.width), 2);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 7);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 7);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 7);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Remove point cloud rotation
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 0);
  EXPECT_EQ (int (cloud_out.size ()), 0);
  EXPECT_EQ (int (cloud_out.width), 0);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 9);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 9);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 9);

  // PointCloud2
  // -------------------------------------------------------------------------

  // Create cloud with center point and corner points
  PointCloud2::Ptr input2 (new PointCloud2);
  pcl::toROSMsg (*input, *input2);

  // Test the PointCloud<PointT> method
  CropBox<PointCloud2> cropBoxFilter2(true);
  cropBoxFilter2.setInputCloud (input2);

  // Cropbox slighlty bigger then bounding box of points
  cropBoxFilter2.setMin (min_pt);
  cropBoxFilter2.setMax (max_pt);

  // Indices
  vector<int> indices2;
  cropBoxFilter2.filter (indices2);

  // Cloud
  PointCloud2 cloud_out2;
  cropBoxFilter2.filter (cloud_out2);

  // Should contain all
  EXPECT_EQ (int (indices2.size ()), 9);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  IndicesConstPtr removed_indices2;
  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 0);

  // Test setNegative
  PointCloud2 cloud_out2_negative;
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 0);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 0);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Translate crop box up by 1
  cropBoxFilter2.setTranslation (Eigen::Vector3f(0, 1, 0));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 5);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 4);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 4);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 4);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Rotate crop box up by 45
  cropBoxFilter2.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 1);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  // Rotate point cloud by -45
  cropBoxFilter2.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 3);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 3);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 6);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 6);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 6);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Translate point cloud down by -1
  cropBoxFilter2.setTransform (getTransformation (0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 2);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 2);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 7);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 7);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 7);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Remove point cloud rotation
  cropBoxFilter2.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 0);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 0);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 9);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 9);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 9);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (StatisticalOutlierRemoval, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;
  // Remove outliers using a spherical density criterion
  StatisticalOutlierRemoval<PointXYZ> outrem;
  outrem.setInputCloud (cloud);
  outrem.setMeanK (50);
  outrem.setStddevMulThresh (1.0);
  outrem.filter (output);

  EXPECT_EQ (int (output.points.size ()), 352);
  EXPECT_EQ (int (output.width), 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.00071029, 1e-4);

  outrem.setNegative (true);
  outrem.filter (output);

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()) - 352);
  EXPECT_EQ (int (output.width), int (cloud->width) - 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.0444, 1e-4);

  // Test the sensor_msgs::PointCloud2 method
  PointCloud2 output2;
  StatisticalOutlierRemoval<PointCloud2> outrem2;
  outrem2.setInputCloud (cloud_blob);
  outrem2.setMeanK (50);
  outrem2.setStddevMulThresh (1.0);
  outrem2.filter (output2);

  fromROSMsg (output2, output);

  EXPECT_EQ (int (output.points.size ()), 352);
  EXPECT_EQ (int (output.width), 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.00071029, 1e-4);

  outrem2.setNegative (true);
  outrem2.filter (output2);

  fromROSMsg (output2, output);

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()) - 352);
  EXPECT_EQ (int (output.width), int (cloud->width) - 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.0444, 1e-4);

  // Remove outliers using a spherical density criterion
  StatisticalOutlierRemoval<PointXYZ> outrem_(true);
  outrem_.setInputCloud (cloud);
  outrem_.setMeanK (50);
  outrem_.setStddevMulThresh (1.0);
  outrem_.filter (output);

  EXPECT_EQ (int (output.points.size ()), 352);
  EXPECT_EQ (int (output.width), 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size ()-outrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.00071029, 1e-4);

  outrem_.setNegative (true);
  outrem_.filter (output);

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()) - 352);
  EXPECT_EQ (int (output.width), int (cloud->width) - 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()) ,cloud->points.size ()-outrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.0444, 1e-4);

  // Test the sensor_msgs::PointCloud2 method
  StatisticalOutlierRemoval<PointCloud2> outrem2_(true);
  outrem2_.setInputCloud (cloud_blob);
  outrem2_.setMeanK (50);
  outrem2_.setStddevMulThresh (1.0);
  outrem2_.filter (output2);

  fromROSMsg (output2, output);

  EXPECT_EQ (int (output.points.size ()), 352);
  EXPECT_EQ (int (output.width), 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.00071029, 1e-4);

  outrem2_.setNegative (true);
  outrem2_.filter (output2);

  fromROSMsg (output2, output);

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()) - 352);
  EXPECT_EQ (int (output.width), int (cloud->width) - 352);
  EXPECT_EQ (bool (output.is_dense), true);
  EXPECT_EQ (int (output.points.size ()), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, -0.0444, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ConditionalRemoval, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;

  // build the condition
  ConditionAnd<PointXYZ>::Ptr range_cond (new ConditionAnd<PointXYZ> ());
  range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("z",
                                                                                                 ComparisonOps::GT,
                                                                                                 0.02)));
  range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("z",
                                                                                                 ComparisonOps::LT,
                                                                                                 0.04)));
  range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("y",
                                                                                                 ComparisonOps::GT,
                                                                                                 0.10)));
  range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("y",
                                                                                                 ComparisonOps::LT,
                                                                                                 0.12)));

  // build the filter
  ConditionalRemoval<PointXYZ> condrem (range_cond);
  condrem.setInputCloud (cloud);

  // try the dense version
  condrem.setKeepOrganized (false);
  condrem.filter (output);

  EXPECT_EQ (bool (output.isOrganized ()), false);
  EXPECT_EQ (int (output.points.size ()), 28);
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.087292, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.103140, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, 0.020825, 1e-4);
  EXPECT_EQ (bool (output.is_dense), true);

  // try the not dense version
  condrem.setKeepOrganized (true);
  condrem.filter (output);

  int num_not_nan = 0;
  for (size_t i = 0; i < output.points.size (); i++)
  {
    if (pcl_isfinite (output.points[i].x) &&
        pcl_isfinite (output.points[i].y) &&
        pcl_isfinite (output.points[i].z))
    num_not_nan++;
  }

  EXPECT_EQ (bool (output.isOrganized ()), bool (cloud->isOrganized ()));
  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()));
  EXPECT_EQ (int (output.width), int (cloud->width));
  EXPECT_EQ (int (output.height), int (cloud->height));
  EXPECT_EQ (num_not_nan, 28);
  EXPECT_EQ (bool (output.is_dense), false);

  // build the filter
  ConditionalRemoval<PointXYZ> condrem_ (range_cond,true);
  condrem_.setInputCloud (cloud);

  // try the dense version
  condrem_.setKeepOrganized (false);
  condrem_.filter (output);

  EXPECT_EQ (bool (output.isOrganized ()), false);
  EXPECT_EQ (int (output.points.size ()), 28);
  EXPECT_EQ (int (output.points.size ()), cloud->points.size()-condrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output.points[output.points.size () - 1].x, -0.087292, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].y, 0.103140, 1e-4);
  EXPECT_NEAR (output.points[output.points.size () - 1].z, 0.020825, 1e-4);
  EXPECT_EQ (bool (output.is_dense), true);

  // try the not dense version
  condrem_.setKeepOrganized (true);
  condrem_.filter (output);

  num_not_nan = 0;
  for (size_t i = 0; i < output.points.size (); i++)
  {
    if (pcl_isfinite (output.points[i].x) &&
        pcl_isfinite (output.points[i].y) &&
        pcl_isfinite (output.points[i].z))
    num_not_nan++;
  }

  EXPECT_EQ (bool (output.isOrganized ()), bool (cloud->isOrganized ()));
  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()));
  EXPECT_EQ (int (output.width), int (cloud->width));
  EXPECT_EQ (int (output.height), int (cloud->height));
  EXPECT_EQ (num_not_nan, 28);
  EXPECT_EQ (bool (output.is_dense), false);
  EXPECT_EQ (int (num_not_nan), cloud->points.size()-condrem_.getRemovedIndices()->size());
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ConditionalRemovalSetIndices, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;

  // build some indices
  boost::shared_ptr<vector<int> > indices (new vector<int> (2));
  (*indices)[0] = 0;
  (*indices)[1] = static_cast<int> (cloud->points.size ()) - 1;

  // build a condition which is always true
  ConditionAnd<PointXYZ>::Ptr true_cond (new ConditionAnd<PointXYZ> ());
  true_cond->addComparison (TfQuadraticXYZComparison<PointXYZ>::ConstPtr (new TfQuadraticXYZComparison<PointXYZ> (ComparisonOps::EQ, Eigen::Matrix3f::Zero (),
                                                                                                                  Eigen::Vector3f::Zero (), 0)));

  // build the filter
  ConditionalRemoval<PointXYZ> condrem2 (true_cond);
  condrem2.setInputCloud (cloud);
  condrem2.setIndices (indices);

  // try the dense version
  condrem2.setKeepOrganized (false);
  condrem2.filter (output);

  EXPECT_EQ (int (output.points.size ()), 2);
  EXPECT_EQ (int (output.width), 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[1].z);

  // try the not dense version
  condrem2.setKeepOrganized (true);
  condrem2.filter (output);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[output.points.size () - 1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[output.points.size () - 1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[output.points.size () - 1].z);

  int num_not_nan = 0;
  for (size_t i = 0; i < output.points.size (); i++)
  {
    if (pcl_isfinite (output.points[i].x) &&
        pcl_isfinite (output.points[i].y) &&
        pcl_isfinite (output.points[i].z))
      num_not_nan++;
  }

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()));
  EXPECT_EQ (int (output.width), int (cloud->width));
  EXPECT_EQ (int (output.height), int (cloud->height));
  EXPECT_EQ (num_not_nan, 2);

  // build the filter
  ConditionalRemoval<PointXYZ> condrem2_ (true_cond, true);
  condrem2_.setIndices (indices);
  condrem2_.setInputCloud (cloud);

  // try the dense version
  condrem2_.setKeepOrganized (false);
  condrem2_.filter (output);

  EXPECT_EQ (int (output.points.size ()), 2);
  EXPECT_EQ (int (output.width), 2);
  EXPECT_EQ (int (output.height), 1);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[1].z);

  EXPECT_EQ (int (output.points.size ()), int (indices->size ()) - int (condrem2_.getRemovedIndices ()->size ()));

  // try the not dense version
  condrem2_.setKeepOrganized (true);
  condrem2_.filter (output);

  EXPECT_EQ (cloud->points[0].x, output.points[0].x);
  EXPECT_EQ (cloud->points[0].y, output.points[0].y);
  EXPECT_EQ (cloud->points[0].z, output.points[0].z);

  EXPECT_EQ (cloud->points[cloud->points.size () - 1].x, output.points[output.points.size () - 1].x);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].y, output.points[output.points.size () - 1].y);
  EXPECT_EQ (cloud->points[cloud->points.size () - 1].z, output.points[output.points.size () - 1].z);

  num_not_nan = 0;
  for (size_t i = 0; i < output.points.size (); i++)
  {
    if (pcl_isfinite (output.points[i].x) &&
        pcl_isfinite (output.points[i].y) &&
        pcl_isfinite (output.points[i].z))
      num_not_nan++;
  }

  EXPECT_EQ (int (output.points.size ()), int (cloud->points.size ()));
  EXPECT_EQ (int (output.width), int (cloud->width));
  EXPECT_EQ (int (output.height), int (cloud->height));
  EXPECT_EQ (num_not_nan, 2);

  EXPECT_EQ (num_not_nan, int (indices->size ()) - int (condrem2_.getRemovedIndices ()->size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (SamplingSurfaceNormal, Filters)
{
  PointCloud <PointNormal>::Ptr incloud (new PointCloud <PointNormal> ());
  PointCloud <PointNormal> outcloud;

  //Creating a point cloud on the XY plane
  for (float i = 0.0f; i < 5.0f; i += 0.1f)
  {
    for (float j = 0.0f; j < 5.0f; j += 0.1f)
    {
      PointNormal pt;
      pt.x = i;
      pt.y = j;
      pt.z = 1;
      incloud->points.push_back (pt);
    }
  }
  incloud->width = 1;
  incloud->height = uint32_t (incloud->points.size ());

  pcl::SamplingSurfaceNormal <pcl::PointNormal> ssn_filter;
  ssn_filter.setInputCloud (incloud);
  ssn_filter.setRatio (0.3f);
  ssn_filter.filter (outcloud);

  // All the sampled points should have normals along the direction of Z axis
  for (unsigned int i = 0; i < outcloud.points.size (); i++)
  {
    EXPECT_NEAR (outcloud.points[i].normal[0], 0, 1e-3);
    EXPECT_NEAR (outcloud.points[i].normal[1], 0, 1e-3);
    EXPECT_NEAR (outcloud.points[i].normal[2], 1, 1e-3);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ShadowPoints, Filters)
{
  //Creating a point cloud on the XY plane
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());
  for (float i = 0.0f; i < 10.0f; i+=0.1f)
  {
    for (float j = 0.0f; j < 10.0f; j+=0.1f)
    {
      input->push_back (PointXYZ (i, j, 1.0f));
    }
  }

  // Adding a shadow point
  unsigned int N = static_cast<unsigned int> (input->points.size ());
  PointXYZ pt = input->points[N];
  pt.z = input->points[N].z + 0.1f;
  input->points.push_back (pt);

  input->width = 1;
  input->height = static_cast<uint32_t> (input->points.size ());

	NormalEstimation<PointXYZ, PointNormal> ne;
	ne.setInputCloud (input);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	pcl::PointCloud<PointNormal>::Ptr input_normals (new PointCloud<PointNormal>);
	ne.setKSearch (15);
	ne.compute (*input_normals);

  PointCloud<PointXYZ> output;
  ShadowPoints <PointXYZ, PointNormal> spfilter;
  spfilter.setInputCloud (input);
  spfilter.setThreshold (0.1f);
  spfilter.setNormals (input_normals);

  spfilter.filter (output);

  // Should filter out the one shadow point that was added.
  EXPECT_EQ (int (output.points.size ()), 10000);
}


//////////////////////////////////////////////////////////////////////////////////////////////
TEST (FrustumCulling, Filters)
{
  //Creating a point cloud on the XY plane
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());

  for (int i = 0; i < 5; i++) 
  {
    for (int j = 0; j < 5; j++) 
    {
      for (int k = 0; k < 5; k++) 
      {
        pcl::PointXYZ pt;
        pt.x = float (i);
        pt.y = float (j);
        pt.z = float (k);
        input->points.push_back (pt);
      }
    }
  }
  input->width = 1;
  input->height = static_cast<uint32_t> (input->points.size ());

  pcl::FrustumCulling<pcl::PointXYZ> fc;
  fc.setInputCloud (input);
  fc.setVerticalFOV (90);
  fc.setHorizontalFOV (90);
  fc.setNearPlaneDistance (0.0);
  fc.setFarPlaneDistance (10);

  Eigen::Matrix4f camera_pose;
  camera_pose.setZero ();

  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitX ()) *
    Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitY ()) *
    Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitZ ());

  camera_pose.block (0, 0, 3, 3) = R;

  Eigen::Vector3f T;
  T (0) = -5; T (1) = 0; T (2) = 0;
  camera_pose.block (0, 3, 3, 1) = T;
  camera_pose (3, 3) = 1;

  fc.setCameraPose (camera_pose);

  pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
  fc.filter (*output);

  // Should filter all points in the input cloud
  EXPECT_EQ (int (output->points.size ()), int (input->points.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ConditionalRemovalTfQuadraticXYZComparison, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;

  // Create cloud: a line along the x-axis
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());

  input->push_back (PointXYZ (-1.0, 0.0, 0.0));
  input->push_back (PointXYZ (0.0, 0.0, 0.0));
  input->push_back (PointXYZ (1.0, 0.0, 0.0));
  input->push_back (PointXYZ (2.0, 0.0, 0.0));
  input->push_back (PointXYZ (3.0, 0.0, 0.0));
  input->push_back (PointXYZ (4.0, 0.0, 0.0));
  input->push_back (PointXYZ (5.0, 0.0, 0.0));
  input->push_back (PointXYZ (6.0, 0.0, 0.0));
  input->push_back (PointXYZ (7.0, 0.0, 0.0));
  input->push_back (PointXYZ (8.0, 0.0, 0.0));

  // build a condition representing the inner of a cylinder including the edge located at the origin and pointing along the x-axis.
  ConditionAnd<PointXYZ>::Ptr cyl_cond (new ConditionAnd<PointXYZ> ());
  Eigen::Matrix3f cylinderMatrix;
  cylinderMatrix << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  float cylinderScalar = -4; //radius² of cylinder
  TfQuadraticXYZComparison<PointXYZ>::Ptr cyl_comp (new TfQuadraticXYZComparison<PointXYZ> (ComparisonOps::LE, cylinderMatrix,
                                                                                            Eigen::Vector3f::Zero (), cylinderScalar));
  cyl_cond->addComparison (cyl_comp);

  // build the filter
  ConditionalRemoval<PointXYZ> condrem (cyl_cond);
  condrem.setInputCloud (input);
  condrem.setKeepOrganized (false);

  // apply it
  condrem.filter (output);

  EXPECT_EQ (10, int (output.points.size ()));

  EXPECT_EQ (input->points[0].x, output.points[0].x);
  EXPECT_EQ (input->points[0].y, output.points[0].y);
  EXPECT_EQ (input->points[0].z, output.points[0].z);

  EXPECT_EQ (input->points[9].x, output.points[9].x);
  EXPECT_EQ (input->points[9].y, output.points[9].y);
  EXPECT_EQ (input->points[9].z, output.points[9].z);

  // rotate cylinder comparison along z-axis by PI/2
  cyl_comp->transformComparison (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, float (M_PI) / 2.0f).inverse ());

  condrem.filter (output);

  EXPECT_EQ (4, int (output.points.size ()));

  EXPECT_EQ (input->points[0].x, output.points[0].x);
  EXPECT_EQ (input->points[0].y, output.points[0].y);
  EXPECT_EQ (input->points[0].z, output.points[0].z);

  EXPECT_EQ (input->points[3].x, output.points[3].x);
  EXPECT_EQ (input->points[3].y, output.points[3].y);
  EXPECT_EQ (input->points[3].z, output.points[3].z);

  // change comparison to a simple plane (x < 5)
  Eigen::Vector3f planeVector;
  planeVector << 1.0, 0.0, 0.0;
  Eigen::Matrix3f planeMatrix = Eigen::Matrix3f::Zero ();
  cyl_comp->setComparisonMatrix (planeMatrix);
  cyl_comp->setComparisonVector (planeVector);
  cyl_comp->setComparisonScalar (-2 * 5.0);
  cyl_comp->setComparisonOperator (ComparisonOps::LT); 

  condrem.filter (output);

  EXPECT_EQ (6, int (output.points.size ()));

  EXPECT_EQ (input->points[0].x, output.points[0].x);
  EXPECT_EQ (input->points[0].y, output.points[0].y);
  EXPECT_EQ (input->points[0].z, output.points[0].z);

  EXPECT_EQ (input->points[5].x, output.points[5].x);
  EXPECT_EQ (input->points[5].y, output.points[5].y);
  EXPECT_EQ (input->points[5].z, output.points[5].z);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  loadPCDFile (file_name, *cloud_blob);
  fromROSMsg (*cloud_blob, *cloud);

  indices_.resize (cloud->points.size ());
  for (int i = 0; i < static_cast<int> (indices_.size ()); ++i)
    indices_[i] = i;

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
