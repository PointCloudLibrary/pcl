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

#include <pcl/test/gtest.h>
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
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_refinement.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <pcl/segmentation/sac_segmentation.h>

using namespace pcl;
using namespace pcl::io;
using namespace Eigen;


PCLPointCloud2::Ptr cloud_blob (new PCLPointCloud2);
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

PointCloud<PointXYZRGB>::Ptr cloud_organized (new PointCloud<PointXYZRGB>);


//pcl::IndicesConstPtr indices;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractIndicesSelf, Filters)
{
  // Test the PointCloud<PointT> method
  ExtractIndices<PointXYZ> ei;
  pcl::IndicesPtr indices (new pcl::Indices (2));
  (*indices)[0] = 0;
  (*indices)[1] = cloud->size () - 1;

  PointCloud<PointXYZ>::Ptr output (new PointCloud<PointXYZ>);
  ei.setInputCloud (cloud);
  ei.setIndices (indices);
  ei.filter (*output);

  EXPECT_EQ (output->size (), 2);
  EXPECT_EQ (output->width, 2);
  EXPECT_EQ (output->height, 1);

  EXPECT_EQ ((*cloud)[0].x, (*output)[0].x);
  EXPECT_EQ ((*cloud)[0].y, (*output)[0].y);
  EXPECT_EQ ((*cloud)[0].z, (*output)[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, (*output)[1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, (*output)[1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, (*output)[1].z);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ExtractIndices, Filters)
{
  // Test the PointCloud<PointT> method
  ExtractIndices<PointXYZ> ei;
  pcl::IndicesPtr indices (new pcl::Indices (2));
  (*indices)[0] = 0;
  (*indices)[1] = cloud->size () - 1;

  PointCloud<PointXYZ> output;
  ei.setInputCloud (cloud);
  ei.setIndices (indices);
  ei.filter (output);

  EXPECT_EQ (output.size (), 2);
  EXPECT_EQ (output.width, 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[1].z);

  ei.setNegative (true);
  ei.filter (output);

  EXPECT_EQ (output.size (), cloud->size () - 2);
  EXPECT_EQ (output.width, cloud->size () - 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[1].x, output[0].x);
  EXPECT_EQ ((*cloud)[1].y, output[0].y);
  EXPECT_EQ ((*cloud)[1].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 2].x, output[output.size () - 1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 2].y, output[output.size () - 1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 2].z, output[output.size () - 1].z);

  // Test the pcl::PCLPointCloud2 method
  ExtractIndices<PCLPointCloud2> ei2;

  PCLPointCloud2 output_blob;
  ei2.setInputCloud (cloud_blob);
  ei2.setIndices (indices);
  ei2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 2);
  EXPECT_EQ (output.width, 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[1].z);

  ei2.setNegative (true);
  ei2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), cloud->size () - 2);
  EXPECT_EQ (output.width, cloud->size () - 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[1].x, output[0].x);
  EXPECT_EQ ((*cloud)[1].y, output[0].y);
  EXPECT_EQ ((*cloud)[1].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 2].x, output[output.size () - 1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 2].y, output[output.size () - 1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 2].z, output[output.size () - 1].z);

  ei2.setNegative (false);
  ei2.setKeepOrganized (true);
  ei2.filter (output_blob);

  fromPCLPointCloud2(output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  EXPECT_EQ (output[0].x, (*cloud)[0].x);
  EXPECT_EQ (output[0].y, (*cloud)[0].y);
  EXPECT_EQ (output[0].z, (*cloud)[0].z);
  EXPECT_TRUE (std::isnan(output[1].x));
  EXPECT_TRUE (std::isnan(output[1].y));
  EXPECT_TRUE (std::isnan(output[1].z));

  ei2.setNegative (true);
  ei2.setKeepOrganized (true);
  ei2.filter (output_blob);

  fromPCLPointCloud2(output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  EXPECT_TRUE (std::isnan(output[0].x));
  EXPECT_TRUE (std::isnan(output[0].y));
  EXPECT_TRUE (std::isnan(output[0].z));
  EXPECT_EQ (output[1].x, (*cloud)[1].x);
  EXPECT_EQ (output[1].y, (*cloud)[1].y);
  EXPECT_EQ (output[1].z, (*cloud)[1].z);

  // Test setNegative on empty datasets
  PointCloud<PointXYZ> empty, result;
  ExtractIndices<PointXYZ> eie;
  eie.setInputCloud (empty.makeShared ());
  eie.setNegative (false);
  eie.filter (result);

  EXPECT_EQ (result.size (), 0);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (result.size (), 0);

  pcl::IndicesPtr idx (new pcl::Indices (10));
  eie.setIndices (idx);
  eie.setNegative (false);
  eie.filter (result);
  EXPECT_EQ (result.size (), 0);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (result.size (), 0);

  empty.resize (10);
  empty.width = 10; empty.height = 1;
  eie.setInputCloud (empty.makeShared ());
  for (int i = 0; i < 10; ++i)
    (*idx)[i] = i;
  eie.setIndices (idx);
  eie.setNegative (false);
  eie.filter (result);
  EXPECT_EQ (result.size (), 10);
  eie.setNegative (true);
  eie.filter (result);
  EXPECT_EQ (result.size (), 0);

  /*
  PointCloud<PointXYZ> sc, scf;
  sc.resize (5); sc.width = 5; sc.height = 1; sc.is_dense = true;
  for (int i = 0; i < 5; i++)
  {
    sc[i].x = sc[i].z = 0;
    sc[i].y = i;
  }
  PassThrough<PointXYZ> ps;
  ps.setInputCloud (sc.makeShared ());
  ps.setFilterFieldName ("y");
  ps.setFilterLimits (0.99, 2.01);
  for (int i = 0; i < 2; i++)
  {
    ps.setNegative ((bool)i);
    ps.filter (scf);
    std::cerr << scf.size () << std::endl;
    for (index_t j = 0; j < scf.size (); ++j)
      std::cerr << scf[j] << std::endl;
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

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0.05f, 0.1f);
  pt.filter (output);

  EXPECT_EQ (output.size (), 42);
  EXPECT_EQ (output.width, 42);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output[41].z, 0.052133, 1e-5);

  pt.setNegative (true);
  pt.filter (output);

  EXPECT_EQ (output.size (), 355);
  EXPECT_EQ (output.width, 355);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output[354].z, -0.0444, 1e-5);

  PassThrough<PointXYZ> pt_(true);

  pt_.setInputCloud (cloud);
  pt_.filter (output);

  EXPECT_EQ (pt_.getRemovedIndices()->size(), 0);
  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt_.setFilterFieldName ("z");
  pt_.setFilterLimits (0.05f, 0.1f);
  pt_.filter (output);

  EXPECT_EQ (output.size (), 42);
  EXPECT_EQ (output.width, 42);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud->size ()-pt_.getRemovedIndices()->size());

  EXPECT_NEAR (output[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output[41].z, 0.052133, 1e-5);

  pt_.setNegative (true);
  pt_.filter (output);

  EXPECT_EQ (output.size (), 355);
  EXPECT_EQ (output.width, 355);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud->size ()-pt_.getRemovedIndices()->size());

  EXPECT_NEAR (output[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output[354].z, -0.0444, 1e-5);

  // Test the keep organized structure
  pt.setUserFilterValue (std::numeric_limits<float>::quiet_NaN ());
  pt.setFilterFieldName ("");
  pt.filter (output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (output.is_dense, cloud->is_dense);
  EXPECT_NEAR (output[0].x, (*cloud)[0].x, 1e-5);
  EXPECT_NEAR (output[output.size () - 1].x, (*cloud)[cloud->size () - 1].x, 1e-5);

  pt.setFilterFieldName ("z");
  pt.setNegative (false);
  pt.setKeepOrganized (true);
  pt.filter (output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_FALSE (output.is_dense); // NaN was set as a user filter value

  EXPECT_TRUE (std::isnan (output[0].x));
  EXPECT_TRUE (std::isnan (output[0].y));
  EXPECT_TRUE (std::isnan (output[0].z));
  EXPECT_TRUE (std::isnan (output[41].x));
  EXPECT_TRUE (std::isnan (output[41].y));
  EXPECT_TRUE (std::isnan (output[41].z));

  pt.setNegative (true);
  pt.filter (output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_FALSE (output.is_dense); // NaN was set as a user filter value

  EXPECT_NEAR (output[0].x, (*cloud)[0].x, 1e-5);
  EXPECT_NEAR (output[0].y, (*cloud)[0].y, 1e-5);
  EXPECT_NEAR (output[0].z, (*cloud)[0].z, 1e-5);

  EXPECT_NEAR (output[41].x, (*cloud)[41].x, 1e-5);
  EXPECT_NEAR (output[41].y, (*cloud)[41].y, 1e-5);
  EXPECT_NEAR (output[41].z, (*cloud)[41].z, 1e-5);

  // Test the PCLPointCloud2 method
  PassThrough<PCLPointCloud2> pt2;

  PCLPointCloud2 output_blob;
  pt2.setInputCloud (cloud_blob);
  pt2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt2.setFilterFieldName ("z");
  pt2.setFilterLimits (0.05, 0.1);
  pt2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 42);
  EXPECT_EQ (output.width, 42);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output[41].z, 0.052133, 1e-5);

  pt2.setNegative (true);
  pt2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 355);
  EXPECT_EQ (output.width, 355);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output[354].z, -0.0444, 1e-5);

  PassThrough<PCLPointCloud2> pt2_(true);
  pt2_.setInputCloud (cloud_blob);
  pt2_.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (pt2_.getRemovedIndices()->size(), 0);
  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);

  pt2_.setFilterFieldName ("z");
  pt2_.setFilterLimits (0.05, 0.1);
  pt2_.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 42);
  EXPECT_EQ (output.width, 42);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud->size ()-pt2_.getRemovedIndices()->size());

  EXPECT_NEAR (output[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output[41].z, 0.052133, 1e-5);

  pt2_.setNegative (true);
  pt2_.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 355);
  EXPECT_EQ (output.width, 355);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud->size ()-pt2_.getRemovedIndices()->size());

  EXPECT_NEAR (output[0].x, 0.0054216, 1e-5);
  EXPECT_NEAR (output[0].y, 0.11349, 1e-5);
  EXPECT_NEAR (output[0].z, 0.040749, 1e-5);

  EXPECT_NEAR (output[354].x, -0.07793, 1e-5);
  EXPECT_NEAR (output[354].y, 0.17516, 1e-5);
  EXPECT_NEAR (output[354].z, -0.0444, 1e-5);

  // Test the keep organized structure
  pt2.setUserFilterValue (std::numeric_limits<float>::quiet_NaN ());
  pt2.setFilterFieldName ("");
  pt2.filter (output_blob);
  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (output.is_dense, cloud->is_dense);
  EXPECT_NEAR (output[0].x, (*cloud)[0].x, 1e-5);
  EXPECT_NEAR (output[output.size () - 1].x, (*cloud)[cloud->size () - 1].x, 1e-5);

  pt2.setFilterFieldName ("z");
  pt2.setNegative (false);
  pt2.setKeepOrganized (true);
  pt2.filter (output_blob);
  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_FALSE (output.is_dense); // NaN was set as a user filter value

  EXPECT_TRUE (std::isnan (output[0].x));
  EXPECT_TRUE (std::isnan (output[0].y));
  EXPECT_TRUE (std::isnan (output[0].z));

  EXPECT_TRUE (std::isnan (output[41].x));
  EXPECT_TRUE (std::isnan (output[41].y));
  EXPECT_TRUE (std::isnan (output[41].z));

  pt2.setNegative (true);
  pt2.filter (output_blob);
  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_FALSE (output.is_dense); // NaN was set as a user filter value

  EXPECT_NEAR (output[0].x, (*cloud)[0].x, 1e-5);
  EXPECT_NEAR (output[0].y, (*cloud)[0].y, 1e-5);
  EXPECT_NEAR (output[0].z, (*cloud)[0].z, 1e-5);

  EXPECT_NEAR (output[41].x, (*cloud)[41].x, 1e-5);
  EXPECT_NEAR (output[41].y, (*cloud)[41].y, 1e-5);
  EXPECT_NEAR (output[41].z, (*cloud)[41].z, 1e-5);
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

  EXPECT_EQ (output.size (), 103);
  EXPECT_EQ (output.width, 103);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  grid.setFilterFieldName ("z");
  grid.setFilterLimits (0.05, 0.1);
  grid.filter (output);

  EXPECT_EQ (output.size (), 14);
  EXPECT_EQ (output.width, 14);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output[13].z, 0.051333, 1e-4);

  grid.setFilterLimitsNegative (true);
  grid.setSaveLeafLayout(true);
  grid.filter (output);

  EXPECT_EQ (output.size (), 100);
  EXPECT_EQ (output.width, 100);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  //EXPECT_NEAR (output[0].x, -0.070192, 1e-4);
  //EXPECT_NEAR (output[0].y, 0.17653, 1e-4);
  //EXPECT_NEAR (output[0].z, -0.048774, 1e-4);

  //EXPECT_NEAR (output[99].x, -0.068948, 1e-4);
  //EXPECT_NEAR (output[99].y, 0.1447, 1e-4);
  //EXPECT_NEAR (output[99].z, 0.042178, 1e-4);

  // centroids should be identified correctly
  EXPECT_EQ (grid.getCentroidIndex (output[0]), 0);
  EXPECT_EQ (grid.getCentroidIndex (output[99]), 99);
  EXPECT_EQ (grid.getCentroidIndexAt (grid.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx = grid.getCentroidIndex ((*cloud)[195]);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (std::abs (output[centroidIdx].x - (*cloud)[195].x), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx].y - (*cloud)[195].y), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx].z - (*cloud)[195].z), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[0], Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[99], Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions = Eigen::Vector3i (0, 0, 1);
  std::vector<int> neighbors = grid.getNeighborCentroidIndices ((*cloud)[195], directions);
  EXPECT_EQ (neighbors.size (), std::size_t (directions.cols ()));
  EXPECT_NE (neighbors.at (0), -1);
  EXPECT_LE (std::abs (output[neighbors.at (0)].x - output[centroidIdx].x), 0.02);
  EXPECT_LE (std::abs (output[neighbors.at (0)].y - output[centroidIdx].y), 0.02);
  EXPECT_LE ( output[neighbors.at (0)].z - output[centroidIdx].z, 0.02 * 2);

  // Test the pcl::PCLPointCloud2 method
  VoxelGrid<PCLPointCloud2> grid2;

  PCLPointCloud2 output_blob;

  grid2.setLeafSize (0.02f, 0.02f, 0.02f);
  grid2.setInputCloud (cloud_blob);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 103);
  EXPECT_EQ (output.width, 103);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  grid2.setFilterFieldName ("z");
  grid2.setFilterLimits (0.05, 0.1);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 14);
  EXPECT_EQ (output.width, 14);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output[13].z, 0.051333, 1e-4);

  grid2.setFilterLimitsNegative (true);
  grid2.setSaveLeafLayout(true);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 100);
  EXPECT_EQ (output.width, 100);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  //EXPECT_NEAR (output[0].x, -0.070192, 1e-4);
  //EXPECT_NEAR (output[0].y, 0.17653, 1e-4);
  //EXPECT_NEAR (output[0].z, -0.048774, 1e-4);

  //EXPECT_NEAR (output[99].x, -0.068948, 1e-4);
  //EXPECT_NEAR (output[99].y, 0.1447, 1e-4);
  //EXPECT_NEAR (output[99].z, 0.042178, 1e-4);

  // centroids should be identified correctly
  EXPECT_EQ (grid2.getCentroidIndex (output[0].x, output[0].y, output[0].z), 0);
  EXPECT_EQ (grid2.getCentroidIndex (output[99].x, output[99].y, output[99].z), 99);
  EXPECT_EQ (grid2.getCentroidIndexAt (grid2.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx2 = grid2.getCentroidIndex (0.048722f, 0.073760f, 0.017434f);
  EXPECT_NE (centroidIdx2, -1);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (std::abs (output[centroidIdx2].x - 0.048722), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx2].y - 0.073760), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx2].z - 0.017434), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output[0].x, output[0].y, output[0].z, Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output[99].x, output[99].y, output[99].z, Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions2 = Eigen::Vector3i (0, 0, 1);
  std::vector<int> neighbors2 = grid2.getNeighborCentroidIndices (0.048722f, 0.073760f, 0.017434f, directions2);
  EXPECT_EQ (neighbors2.size (), std::size_t (directions2.cols ()));
  EXPECT_NE (neighbors2.at (0), -1);
  EXPECT_LE (std::abs (output[neighbors2.at (0)].x - output[centroidIdx2].x), 0.02);
  EXPECT_LE (std::abs (output[neighbors2.at (0)].y - output[centroidIdx2].y), 0.02);
  EXPECT_LE (output[neighbors2.at (0)].z - output[centroidIdx2].z, 0.02 * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid_No_DownsampleAllData, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;
  VoxelGrid<PointXYZ> grid;

  grid.setLeafSize (0.02f, 0.02f, 0.02f);
  grid.setDownsampleAllData(false);
  grid.setInputCloud (cloud);
  grid.filter (output);

  EXPECT_EQ (output.size (), 103);
  EXPECT_EQ (output.width, 103);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  grid.setFilterFieldName ("z");
  grid.setFilterLimits (0.05, 0.1);
  grid.filter (output);

  EXPECT_EQ (output.size (), 14);
  EXPECT_EQ (output.width, 14);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output[13].z, 0.051333, 1e-4);

  grid.setFilterLimitsNegative (true);
  grid.setSaveLeafLayout(true);
  grid.filter (output);

  EXPECT_EQ (output.size (), 100);
  EXPECT_EQ (output.width, 100);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  // centroids should be identified correctly
  EXPECT_EQ (grid.getCentroidIndex (output[0]), 0);
  EXPECT_EQ (grid.getCentroidIndex (output[99]), 99);
  EXPECT_EQ (grid.getCentroidIndexAt (grid.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx = grid.getCentroidIndex ((*cloud)[195]);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (std::abs (output[centroidIdx].x - (*cloud)[195].x), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx].y - (*cloud)[195].y), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx].z - (*cloud)[195].z), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[0], Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[99], Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions = Eigen::Vector3i (0, 0, 1);
  std::vector<int> neighbors = grid.getNeighborCentroidIndices ((*cloud)[195], directions);
  EXPECT_EQ (neighbors.size (), std::size_t (directions.cols ()));
  EXPECT_NE (neighbors.at (0), -1);
  EXPECT_LE (std::abs (output[neighbors.at (0)].x - output[centroidIdx].x), 0.02);
  EXPECT_LE (std::abs (output[neighbors.at (0)].y - output[centroidIdx].y), 0.02);
  EXPECT_LE ( output[neighbors.at (0)].z - output[centroidIdx].z, 0.02 * 2);

  // Test the pcl::PCLPointCloud2 method
  VoxelGrid<PCLPointCloud2> grid2;

  PCLPointCloud2 output_blob;

  grid2.setLeafSize (0.02f, 0.02f, 0.02f);
  grid2.setDownsampleAllData(false);
  grid2.setInputCloud (cloud_blob);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 103);
  EXPECT_EQ (output.width, 103);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  grid2.setFilterFieldName ("z");
  grid2.setFilterLimits (0.05, 0.1);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 14);
  EXPECT_EQ (output.width, 14);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.026125, 1e-4);
  EXPECT_NEAR (output[0].y, 0.039788, 1e-4);
  EXPECT_NEAR (output[0].z, 0.052827, 1e-4);

  EXPECT_NEAR (output[13].x, -0.073202, 1e-4);
  EXPECT_NEAR (output[13].y, 0.1296, 1e-4);
  EXPECT_NEAR (output[13].z, 0.051333, 1e-4);

  grid2.setFilterLimitsNegative (true);
  grid2.setSaveLeafLayout(true);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  EXPECT_EQ (output.size (), 100);
  EXPECT_EQ (output.width, 100);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  // centroids should be identified correctly
  EXPECT_EQ (grid2.getCentroidIndex (output[0].x, output[0].y, output[0].z), 0);
  EXPECT_EQ (grid2.getCentroidIndex (output[99].x, output[99].y, output[99].z), 99);
  EXPECT_EQ (grid2.getCentroidIndexAt (grid2.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 195 [0.04872199893, 0.07376000285, 0.01743399911]
  int centroidIdx2 = grid2.getCentroidIndex (0.048722f, 0.073760f, 0.017434f);
  EXPECT_NE (centroidIdx2, -1);

  // for arbitrary points, the centroid should be close
  EXPECT_LE (std::abs (output[centroidIdx2].x - 0.048722), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx2].y - 0.073760), 0.02);
  EXPECT_LE (std::abs (output[centroidIdx2].z - 0.017434), 0.02);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output[0].x, output[0].y, output[0].z, Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid2.getNeighborCentroidIndices (output[99].x, output[99].y, output[99].z, Eigen::MatrixXi::Zero(3,1))[0], 99);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions2 = Eigen::Vector3i (0, 0, 1);
  std::vector<int> neighbors2 = grid2.getNeighborCentroidIndices (0.048722f, 0.073760f, 0.017434f, directions2);
  EXPECT_EQ (neighbors2.size (), std::size_t (directions2.cols ()));
  EXPECT_NE (neighbors2.at (0), -1);
  EXPECT_LE (std::abs (output[neighbors2.at (0)].x - output[centroidIdx2].x), 0.02);
  EXPECT_LE (std::abs (output[neighbors2.at (0)].y - output[centroidIdx2].y), 0.02);
  EXPECT_LE (output[neighbors2.at (0)].z - output[centroidIdx2].z, 0.02 * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid_RGB, Filters)
{
  PCLPointCloud2 cloud_rgb_blob_;
  PCLPointCloud2::Ptr cloud_rgb_blob_ptr_;
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
    pt.r = col_r[i];
    pt.g = col_g[i];
    pt.b = col_b[i];
    cloud_rgb_.push_back (pt);
  }

  toPCLPointCloud2 (cloud_rgb_, cloud_rgb_blob_);
  cloud_rgb_blob_ptr_.reset (new PCLPointCloud2 (cloud_rgb_blob_));
  cloud_rgb_ptr_.reset (new PointCloud<PointXYZRGB> (cloud_rgb_));

  PointCloud<PointXYZRGB> output_rgb;
  VoxelGrid<PointXYZRGB> grid_rgb;

  grid_rgb.setLeafSize (0.03f, 0.03f, 0.03f);
  grid_rgb.setInputCloud (cloud_rgb_ptr_);
  grid_rgb.filter (output_rgb);

  EXPECT_EQ (output_rgb.size (), 1);
  EXPECT_EQ (output_rgb.width, 1);
  EXPECT_EQ (output_rgb.height, 1);
  EXPECT_TRUE (output_rgb.is_dense);
  {
    int rgb;
    int r,g,b;
    memcpy (&rgb, &(output_rgb[0].rgb), sizeof(int));
    r = (rgb >> 16) & 0xFF; g = (rgb >> 8 ) & 0xFF; b = (rgb >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgb[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
  }

  VoxelGrid<PCLPointCloud2> grid2;
  PCLPointCloud2 output_rgb_blob;

  grid2.setLeafSize (0.03f, 0.03f, 0.03f);
  grid2.setInputCloud (cloud_rgb_blob_ptr_);
  grid2.filter (output_rgb_blob);

  fromPCLPointCloud2 (output_rgb_blob, output_rgb);

  EXPECT_EQ (output_rgb.size (), 1);
  EXPECT_EQ (output_rgb.width, 1);
  EXPECT_EQ (output_rgb.height, 1);
  EXPECT_TRUE (output_rgb.is_dense);
  {
    int rgb;
    int r,g,b;
    memcpy (&rgb, &(output_rgb[0].rgb), sizeof(int));
    r = (rgb >> 16) & 0xFF; g = (rgb >> 8 ) & 0xFF; b = (rgb >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgb[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgb[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VoxelGrid_RGBA, Filters)
{
  PCLPointCloud2 cloud_rgba_blob_;
  PCLPointCloud2::Ptr cloud_rgba_blob_ptr_;
  PointCloud<PointXYZRGBA> cloud_rgba_;
  PointCloud<PointXYZRGBA>::Ptr cloud_rgba_ptr_;

  int col_r[] = {214, 193, 180, 164, 133, 119, 158, 179, 178, 212};
  int col_g[] = {10, 39, 219, 231, 142, 169, 84, 158, 139, 214};
  int col_b[] = {101, 26, 46, 189, 211, 154, 246, 16, 139, 153};
  int col_a[] = {232, 161, 24, 71, 139, 244, 246, 40, 247, 244};
  float ave_r = 0.0f;
  float ave_b = 0.0f;
  float ave_g = 0.0f;
  float ave_a = 0.0f;
  for (int i = 0; i < 10; ++i)
  {
    ave_r += static_cast<float> (col_r[i]);
    ave_g += static_cast<float> (col_g[i]);
    ave_b += static_cast<float> (col_b[i]);
    ave_a += static_cast<float> (col_a[i]);
  }
  ave_r /= 10.0f;
  ave_g /= 10.0f;
  ave_b /= 10.0f;
  ave_a /= 10.0f;

  for (int i = 0; i < 10; ++i)
  {
    PointXYZRGBA pt;
    int rgba = (col_a[i] << 24) | (col_r[i] << 16) | (col_g[i] << 8) | col_b[i];
    pt.x = 0.0f;
    pt.y = 0.0f;
    pt.z = 0.0f;
    pt.rgba = *reinterpret_cast<std::uint32_t*> (&rgba);
    cloud_rgba_.push_back (pt);
  }

  toPCLPointCloud2 (cloud_rgba_, cloud_rgba_blob_);
  cloud_rgba_blob_ptr_.reset (new PCLPointCloud2 (cloud_rgba_blob_));
  cloud_rgba_ptr_.reset (new PointCloud<PointXYZRGBA> (cloud_rgba_));

  PointCloud<PointXYZRGBA> output_rgba;
  VoxelGrid<PointXYZRGBA> grid_rgba;

  grid_rgba.setLeafSize (0.03f, 0.03f, 0.03f);
  grid_rgba.setInputCloud (cloud_rgba_ptr_);
  grid_rgba.filter (output_rgba);

  EXPECT_EQ (output_rgba.size (), 1);
  EXPECT_EQ (output_rgba.width, 1);
  EXPECT_EQ (output_rgba.height, 1);
  EXPECT_TRUE (output_rgba.is_dense);
  {
    int rgba;
    int r,g,b,a;
    memcpy (&rgba, &(output_rgba[0].rgba), sizeof(int));
    a = (rgba >> 24) & 0xFF; r = (rgba >> 16) & 0xFF; g = (rgba >> 8 ) & 0xFF; b = (rgba >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgba[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgba[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgba[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
    EXPECT_NEAR (a, ave_a, 1.0);
  }

  VoxelGrid<PCLPointCloud2> grid2;
  PCLPointCloud2 output_rgba_blob;

  grid2.setLeafSize (0.03f, 0.03f, 0.03f);
  grid2.setInputCloud (cloud_rgba_blob_ptr_);
  grid2.filter (output_rgba_blob);

  fromPCLPointCloud2 (output_rgba_blob, output_rgba);

  EXPECT_EQ (output_rgba.size (), 1);
  EXPECT_EQ (output_rgba.width, 1);
  EXPECT_EQ (output_rgba.height, 1);
  EXPECT_TRUE (output_rgba.is_dense);
  {
    int rgba;
    int r,g,b,a;
    memcpy (&rgba, &(output_rgba[0].rgba), sizeof(int));
    a = (rgba >> 24) & 0xFF; r = (rgba >> 16) & 0xFF; g = (rgba >> 8 ) & 0xFF; b = (rgba >> 0 ) & 0xFF;
    EXPECT_NEAR (output_rgba[0].x, 0.0, 1e-4);
    EXPECT_NEAR (output_rgba[0].y, 0.0, 1e-4);
    EXPECT_NEAR (output_rgba[0].z, 0.0, 1e-4);
    EXPECT_NEAR (r, ave_r, 1.0);
    EXPECT_NEAR (g, ave_g, 1.0);
    EXPECT_NEAR (b, ave_b, 1.0);
    EXPECT_NEAR (a, ave_a, 1.0);
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
  PCLPointCloud2 cloud_blob_;
  PCLPointCloud2::Ptr cloud_blob_ptr_;

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
        else if (std::isfinite (point.normal_x))
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
        PointNormal& point = output[idx];
        // check for point equalities
        EXPECT_EQ (voxel.x, point.x);
        EXPECT_EQ (voxel.y, point.y);
        EXPECT_EQ (voxel.z, point.z);

        if (std::isfinite(voxel.normal_x) || std::isfinite (point.normal_x))
        {
          EXPECT_EQ (voxel.normal_x, point.normal_x);
          EXPECT_EQ (voxel.normal_y, point.normal_y);
          EXPECT_EQ (voxel.normal_z, point.normal_z);
        }
      }
    }
  }

  toPCLPointCloud2 (*input, cloud_blob_);
  cloud_blob_ptr_.reset (new PCLPointCloud2 (cloud_blob_));

  VoxelGrid<PCLPointCloud2> grid2;
  PCLPointCloud2 output_blob;

  grid2.setLeafSize (1.0f, 1.0f, 1.0f);
  grid2.setFilterLimits (0.0f, 2.0f);
  grid2.setInputCloud (cloud_blob_ptr_);
  grid2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);
  // check the output
  for (unsigned idx = 0, zIdx = 0; zIdx < 2; ++zIdx)
  {
    for (unsigned yIdx = 0; yIdx < 2; ++yIdx)
    {
      for (unsigned xIdx = 0; xIdx < 2; ++xIdx, ++idx)
      {
        PointNormal& voxel = ground_truth [xIdx][yIdx][zIdx];
        PointNormal& point = output[idx];
        // check for point equalities
        EXPECT_EQ (voxel.x, point.x);
        EXPECT_EQ (voxel.y, point.y);
        EXPECT_EQ (voxel.z, point.z);

        if (std::isfinite(voxel.normal_x) || std::isfinite (point.normal_x))
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

  EXPECT_EQ (output.size (), 23);
  EXPECT_EQ (output.width, 23);
  EXPECT_EQ (output.height, 1);
  EXPECT_TRUE (output.is_dense);

  EXPECT_NEAR (output[0].x, -0.073619894683361053, 1e-4);
  EXPECT_NEAR (output[0].y,  0.16789889335632324,  1e-4);
  EXPECT_NEAR (output[0].z, -0.03018110990524292,  1e-4);

  EXPECT_NEAR (output[13].x, -0.06865914911031723, 1e-4);
  EXPECT_NEAR (output[13].y,  0.15243285894393921, 1e-4);
  EXPECT_NEAR (output[13].z,  0.03266800194978714, 1e-4);

  grid.setSaveLeafLayout (true);
  grid.filter (output);

  // centroids should be identified correctly
  EXPECT_EQ (grid.getCentroidIndex (output[0]), 0);
  EXPECT_EQ (grid.getCentroidIndex (output[17]), 17);
  EXPECT_EQ (grid.getCentroidIndexAt (grid.getGridCoordinates (-1,-1,-1)), -1);
  //PCL_ERROR ("IGNORE PREVIOUS ERROR: testing it's functionality!\n");

  // input point 38 [-0.066091, 0.11973, 0.050881]
  int centroidIdx = grid.getCentroidIndex ((*cloud)[38]);
  EXPECT_NE (centroidIdx, -1);

  // if getNeighborCentroidIndices works then the other helper functions work as well
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[0], Eigen::MatrixXi::Zero(3,1))[0], 0);
  EXPECT_EQ (grid.getNeighborCentroidIndices (output[17], Eigen::MatrixXi::Zero(3,1))[0], 17);

  // neighboring centroid should be in the right position
  Eigen::MatrixXi directions = Eigen::Vector3i (0, 1, 0);
  std::vector<int> neighbors = grid.getNeighborCentroidIndices ((*cloud)[38], directions);
  EXPECT_EQ (neighbors.size (), std::size_t (directions.cols ()));
  EXPECT_NE (neighbors.at (0), -1);
  EXPECT_LE (std::abs (output[neighbors.at (0)].x - output[centroidIdx].x), 0.02);
  EXPECT_LE (std::abs (output[neighbors.at (0)].y - output[centroidIdx].y), 0.02);
  EXPECT_LE (output[neighbors.at (0)].z - output[centroidIdx].z, 0.02 * 2);

  // testing search functions
  grid.setSaveLeafLayout (false);
  grid.filter (output, true);

  // testing k nearest neighbors search
  std::vector<VoxelGridCovariance<pcl::PointXYZ>::LeafConstPtr> leaves;
  std::vector<float> distances;
  grid.nearestKSearch (PointXYZ(0,1,0), 1, leaves, distances);

  EXPECT_EQ (leaves.size (), 1);

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
TEST (VoxelGridMinPoints, Filters)
{
  // Setup input with 4 clusters, single point at 0,0,0 and 1,1,1 with 5 point cluster around 0.11,0.11,0.11 and 6 point cluster around 0.31,0.31,0.31
  PointCloud<PointXYZRGB>::Ptr input(new PointCloud<PointXYZRGB>());

  input->emplace_back(0.0f, 0.0f, 0.0f);
  std::vector<float> offsets {0.001, 0.002, 0.003, -0.001, -0.002, -0.003};
  for (unsigned int i=0; i<5; ++i) {
    input->emplace_back(0.11f+offsets[i], 0.11f+offsets[i], 0.11f+offsets[i],200,0,0);
    input->emplace_back(0.31f+offsets[i], 0.31f+offsets[i], 0.31f+offsets[i],0,127,127);
  }
  input->emplace_back(0.31f+offsets[5], 0.31f+offsets[5], 0.31f+offsets[5],0,127,127);
  input->emplace_back(1.0f, 1.0f, 1.0f);

  // Test the PointCloud<PointT> VoxelGrid filter method
  PointCloud<PointXYZRGB> outputMin4;
  VoxelGrid<PointXYZRGB> grid;
  // Run filter on input with MinimumPoints threshold at 4
  grid.setLeafSize (0.02f, 0.02f, 0.02f);
  grid.setInputCloud (input);
  grid.setMinimumPointsNumberPerVoxel(4);
  grid.setDownsampleAllData(true);
  grid.filter (outputMin4);

  // Verify 2 clusters (0.11 and 0.31) passed threshold and verify their location and color
  EXPECT_EQ (outputMin4.size (), 2);
  // Offset noise applied by offsets vec are 1e-3 magnitude, so check within 1e-2 
  EXPECT_NEAR (outputMin4[0].x, input->at(1).x, 1e-2);
  EXPECT_NEAR (outputMin4[0].y, input->at(1).y, 1e-2);
  EXPECT_NEAR (outputMin4[0].z, input->at(1).z, 1e-2);
  EXPECT_NEAR (outputMin4[0].r, input->at(1).r, 1);

  EXPECT_NEAR (outputMin4[1].x, input->at(2).x, 1e-2);
  EXPECT_NEAR (outputMin4[1].y, input->at(2).y, 1e-2);
  EXPECT_NEAR (outputMin4[1].z, input->at(2).z, 1e-2);
  EXPECT_NEAR (outputMin4[1].g, input->at(2).g, 1);
  EXPECT_NEAR (outputMin4[1].b, input->at(2).b, 1);

  // Run filter again on input with MinimumPoints threshold at 6
  PointCloud<PointXYZRGB> outputMin6;
  grid.setMinimumPointsNumberPerVoxel(6);
  grid.setDownsampleAllData(false);
  grid.filter (outputMin6);

  // Verify 1 cluster (0.31) passed threshold and verify location
  EXPECT_EQ (outputMin6.size (), 1);

  EXPECT_NEAR (outputMin6[0].x, input->at(2).x, 1e-2);
  EXPECT_NEAR (outputMin6[0].y, input->at(2).y, 1e-2);
  EXPECT_NEAR (outputMin6[0].z, input->at(2).z, 1e-2);

  // Test the pcl::PCLPointCloud2 VoxelGrid filter method
  PCLPointCloud2 output_pc2;
  VoxelGrid<PCLPointCloud2> grid_pc2;
  PCLPointCloud2::Ptr input_pc2(new PCLPointCloud2());

  // Use same input as above converted to PCLPointCloud2
  toPCLPointCloud2(*input, *input_pc2);

  // Run filter on input with MinimumPoints threshold at 4
  grid_pc2.setLeafSize (0.02f, 0.02f, 0.02f);
  grid_pc2.setInputCloud (input_pc2);
  grid_pc2.setMinimumPointsNumberPerVoxel(4);
  grid_pc2.setDownsampleAllData(true);
  grid_pc2.filter (output_pc2);

  // Convert back to PointXYZRGB for easier data access
  PointCloud<pcl::PointXYZRGB>::Ptr out_pc( new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::fromPCLPointCloud2( output_pc2, *out_pc );

  // Verify 2 clusters (0.11 and 0.31) passed threshold and verify their location and color
  // PCLPointCloud2 output should be same as PointCloudXYZRGB, account for floating point rounding error with 1e-4
  EXPECT_EQ (out_pc->points.size (), outputMin4.size());

  EXPECT_NEAR (out_pc->at(0).x, outputMin4[0].x, 1e-4);
  EXPECT_NEAR (out_pc->at(0).y, outputMin4[0].y, 1e-4);
  EXPECT_NEAR (out_pc->at(0).z, outputMin4[0].z, 1e-4);
  EXPECT_NEAR (out_pc->at(0).r, outputMin4[0].r, 1);

  EXPECT_NEAR (out_pc->at(1).x, outputMin4[1].x, 1e-4);
  EXPECT_NEAR (out_pc->at(1).y, outputMin4[1].y, 1e-4);
  EXPECT_NEAR (out_pc->at(1).z, outputMin4[1].z, 1e-4);
  EXPECT_NEAR (out_pc->at(1).g, outputMin4[1].g, 1);
  EXPECT_NEAR (out_pc->at(1).b, outputMin4[1].b, 1);

  // Run filter again on input with MinimumPoints threshold at 6
  grid_pc2.setMinimumPointsNumberPerVoxel(6);
  grid_pc2.setDownsampleAllData(false);
  grid_pc2.filter (output_pc2);

  // Convert back to PointXYZRGB for easier data access
  pcl::fromPCLPointCloud2( output_pc2, *out_pc );

  // Verify 1 cluster (0.31) passed threshold and verify location
  EXPECT_EQ (out_pc->points.size (), outputMin6.size());

  EXPECT_NEAR (out_pc->at(0).x, outputMin6[0].x, 1e-4);
  EXPECT_NEAR (out_pc->at(0).y, outputMin6[0].y, 1e-4);
  EXPECT_NEAR (out_pc->at(0).z, outputMin6[0].z, 1e-4);
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

  for (const auto &point : output)
    EXPECT_NEAR (point.z, 0.0, 1e-4);

  // Test the pcl::PCLPointCloud2 method
  ProjectInliers<PCLPointCloud2> proj2;

  PCLPointCloud2 output_blob;

  proj2.setModelType (SACMODEL_PLANE);
  proj2.setInputCloud (cloud_blob);
  proj2.setModelCoefficients (coefficients);
  proj2.filter (output_blob);

  fromPCLPointCloud2 (output_blob, output);

  for (const auto &point : output)
    EXPECT_NEAR (point.z, 0.0, 1e-4);
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

  EXPECT_EQ (cloud_out.size (), 307);
  EXPECT_EQ (cloud_out.width, 307);
  EXPECT_TRUE (cloud_out.is_dense);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].z, -0.021299, 1e-4);

  // Test the pcl::PCLPointCloud2 method
  PCLPointCloud2 cloud_out2;
  RadiusOutlierRemoval<PCLPointCloud2> outrem2;
  outrem2.setInputCloud (cloud_blob);
  outrem2.setRadiusSearch (0.02);
  outrem2.setMinNeighborsInRadius (15);
  outrem2.filter (cloud_out2);

  fromPCLPointCloud2 (cloud_out2, cloud_out);
  EXPECT_EQ (cloud_out.size (), 307);
  EXPECT_EQ (cloud_out.width, 307);
  EXPECT_TRUE (cloud_out.is_dense);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].z, -0.021299, 1e-4);

  // Remove outliers using a spherical density criterion
  RadiusOutlierRemoval<PointXYZ> outrem_(true);
  outrem_.setInputCloud (cloud);
  outrem_.setRadiusSearch (0.02);
  outrem_.setMinNeighborsInRadius (14);
  outrem_.filter (cloud_out);

  EXPECT_EQ (cloud_out.size (), 307);
  EXPECT_EQ (cloud_out.width, 307);
  EXPECT_TRUE (cloud_out.is_dense);
  EXPECT_EQ (cloud_out.size (), cloud->size ()-outrem_.getRemovedIndices()->size());

  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].z, -0.021299, 1e-4);

  // Test the pcl::PCLPointCloud2 method
  RadiusOutlierRemoval<PCLPointCloud2> outrem2_(true);
  outrem2_.setInputCloud (cloud_blob);
  outrem2_.setRadiusSearch (0.02);
  outrem2_.setMinNeighborsInRadius (15);
  outrem2_.filter (cloud_out2);

  fromPCLPointCloud2 (cloud_out2, cloud_out);
  EXPECT_EQ (cloud_out.size (), 307);
  EXPECT_EQ (cloud_out.width, 307);
  EXPECT_TRUE (cloud_out.is_dense);
  EXPECT_EQ (cloud_out.size (), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());

  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].x, -0.077893, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].y, 0.16039, 1e-4);
  EXPECT_NEAR (cloud_out[cloud_out.size () - 1].z, -0.021299, 1e-4);
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

  EXPECT_EQ (output.size (), 352);
  EXPECT_EQ (output.width, 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_NEAR (output[output.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.00071029, 1e-4);

  outrem.setNegative (true);
  outrem.filter (output);

  EXPECT_EQ (output.size (), cloud->size () - 352);
  EXPECT_EQ (output.width, cloud->width - 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_NEAR (output[output.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.0444, 1e-4);

  // Test the pcl::PCLPointCloud2 method
  PCLPointCloud2 output2;
  StatisticalOutlierRemoval<PCLPointCloud2> outrem2;
  outrem2.setInputCloud (cloud_blob);
  outrem2.setMeanK (50);
  outrem2.setStddevMulThresh (1.0);
  outrem2.filter (output2);

  fromPCLPointCloud2 (output2, output);

  EXPECT_EQ (output.size (), 352);
  EXPECT_EQ (output.width, 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_NEAR (output[output.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.00071029, 1e-4);

  outrem2.setNegative (true);
  outrem2.filter (output2);

  fromPCLPointCloud2 (output2, output);

  EXPECT_EQ (output.size (), cloud->size () - 352);
  EXPECT_EQ (output.width, cloud->width - 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_NEAR (output[output.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.0444, 1e-4);

  // Remove outliers using a spherical density criterion
  StatisticalOutlierRemoval<PointXYZ> outrem_(true);
  outrem_.setInputCloud (cloud);
  outrem_.setMeanK (50);
  outrem_.setStddevMulThresh (1.0);
  outrem_.filter (output);

  EXPECT_EQ (output.size (), 352);
  EXPECT_EQ (output.width, 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud->size ()-outrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output[output.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.00071029, 1e-4);

  outrem_.setNegative (true);
  outrem_.filter (output);

  EXPECT_EQ (output.size (), cloud->size () - 352);
  EXPECT_EQ (output.width, cloud->width - 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size () ,cloud->size ()-outrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output[output.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.0444, 1e-4);

  // Test the pcl::PCLPointCloud2 method
  StatisticalOutlierRemoval<PCLPointCloud2> outrem2_(true);
  outrem2_.setInputCloud (cloud_blob);
  outrem2_.setMeanK (50);
  outrem2_.setStddevMulThresh (1.0);
  outrem2_.filter (output2);

  fromPCLPointCloud2 (output2, output);

  EXPECT_EQ (output.size (), 352);
  EXPECT_EQ (output.width, 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());
  EXPECT_NEAR (output[output.size () - 1].x, -0.034667, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.15131, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.00071029, 1e-4);

  outrem2_.setNegative (true);
  outrem2_.filter (output2);

  fromPCLPointCloud2 (output2, output);

  EXPECT_EQ (output.size (), cloud->size () - 352);
  EXPECT_EQ (output.width, cloud->width - 352);
  EXPECT_TRUE (output.is_dense);
  EXPECT_EQ (output.size (), cloud_blob->width*cloud_blob->height-outrem2_.getRemovedIndices()->size());
  EXPECT_NEAR (output[output.size () - 1].x, -0.07793, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.17516, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, -0.0444, 1e-4);
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
  ConditionalRemoval<PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud);

  // try the dense version
  condrem.setKeepOrganized (false);
  condrem.filter (output);

  EXPECT_FALSE (output.isOrganized ());
  EXPECT_EQ (output.size (), 28);
  EXPECT_NEAR (output[output.size () - 1].x, -0.087292, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.103140, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, 0.020825, 1e-4);
  EXPECT_TRUE (output.is_dense);

  // try the not dense version
  condrem.setKeepOrganized (true);
  condrem.filter (output);

  int num_not_nan = 0;
  for (const auto &point : output)
  {
    if (std::isfinite (point.x) &&
        std::isfinite (point.y) &&
        std::isfinite (point.z))
    num_not_nan++;
  }

  EXPECT_EQ (output.isOrganized (), cloud->isOrganized ());
  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (num_not_nan, 28);
  EXPECT_FALSE (output.is_dense);

  // build the filter
  ConditionalRemoval<PointXYZ> condrem_ (true);
  condrem_.setCondition (range_cond);
  condrem_.setInputCloud (cloud);

  // try the dense version
  condrem_.setKeepOrganized (false);
  condrem_.filter (output);

  EXPECT_FALSE (output.isOrganized ());
  EXPECT_EQ (output.size (), 28);
  EXPECT_EQ (output.size (), cloud->size()-condrem_.getRemovedIndices()->size());
  EXPECT_NEAR (output[output.size () - 1].x, -0.087292, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].y, 0.103140, 1e-4);
  EXPECT_NEAR (output[output.size () - 1].z, 0.020825, 1e-4);
  EXPECT_TRUE (output.is_dense);

  // try the not dense version
  condrem_.setKeepOrganized (true);
  condrem_.filter (output);

  num_not_nan = 0;
  for (const auto &point : output)
  {
    if (std::isfinite (point.x) &&
        std::isfinite (point.y) &&
        std::isfinite (point.z))
    num_not_nan++;
  }

  EXPECT_EQ (output.isOrganized (), bool (cloud->isOrganized ()));
  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (num_not_nan, 28);
  EXPECT_FALSE (output.is_dense);
  EXPECT_EQ (num_not_nan, cloud->size()-condrem_.getRemovedIndices()->size());
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (ConditionalRemovalSetIndices, Filters)
{
  // Test the PointCloud<PointT> method
  PointCloud<PointXYZ> output;

  // build some indices
  pcl::IndicesPtr indices (new pcl::Indices (2));
  (*indices)[0] = 0;
  (*indices)[1] = cloud->size () - 1;

  // build a condition which is always true
  ConditionAnd<PointXYZ>::Ptr true_cond (new ConditionAnd<PointXYZ> ());
  true_cond->addComparison (TfQuadraticXYZComparison<PointXYZ>::ConstPtr (new TfQuadraticXYZComparison<PointXYZ> (ComparisonOps::EQ, Eigen::Matrix3f::Zero (),
                                                                                                                  Eigen::Vector3f::Zero (), 0)));

  // build the filter
  ConditionalRemoval<PointXYZ> condrem2;
  condrem2.setCondition (true_cond);
  condrem2.setInputCloud (cloud);
  condrem2.setIndices (indices);

  // try the dense version
  condrem2.setKeepOrganized (false);
  condrem2.filter (output);

  EXPECT_EQ (output.size (), 2);
  EXPECT_EQ (output.width, 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[1].z);

  // try the not dense version
  condrem2.setKeepOrganized (true);
  condrem2.filter (output);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[output.size () - 1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[output.size () - 1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[output.size () - 1].z);

  int num_not_nan = 0;
  for (const auto &point : output)
  {
    if (std::isfinite (point.x) &&
        std::isfinite (point.y) &&
        std::isfinite (point.z))
      num_not_nan++;
  }

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (num_not_nan, 2);

  // build the filter
  ConditionalRemoval<PointXYZ> condrem2_ (true);
  condrem2_.setCondition (true_cond);
  condrem2_.setIndices (indices);
  condrem2_.setInputCloud (cloud);

  // try the dense version
  condrem2_.setKeepOrganized (false);
  condrem2_.filter (output);

  EXPECT_EQ (output.size (), 2);
  EXPECT_EQ (output.width, 2);
  EXPECT_EQ (output.height, 1);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[1].z);

  EXPECT_EQ (output.size (), indices->size () - condrem2_.getRemovedIndices ()->size ());

  // try the not dense version
  condrem2_.setKeepOrganized (true);
  condrem2_.filter (output);

  EXPECT_EQ ((*cloud)[0].x, output[0].x);
  EXPECT_EQ ((*cloud)[0].y, output[0].y);
  EXPECT_EQ ((*cloud)[0].z, output[0].z);

  EXPECT_EQ ((*cloud)[cloud->size () - 1].x, output[output.size () - 1].x);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].y, output[output.size () - 1].y);
  EXPECT_EQ ((*cloud)[cloud->size () - 1].z, output[output.size () - 1].z);

  num_not_nan = 0;
  for (const auto &point : output)
  {
    if (std::isfinite (point.x) &&
        std::isfinite (point.y) &&
        std::isfinite (point.z))
      num_not_nan++;
  }

  EXPECT_EQ (output.size (), cloud->size ());
  EXPECT_EQ (output.width, cloud->width);
  EXPECT_EQ (output.height, cloud->height);
  EXPECT_EQ (num_not_nan, 2);

  EXPECT_EQ (num_not_nan, indices->size () - condrem2_.getRemovedIndices ()->size ());
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
      incloud->push_back (pt);
    }
  }
  incloud->height = 1;
  incloud->width = incloud->size ();

  pcl::SamplingSurfaceNormal <pcl::PointNormal> ssn_filter;
  ssn_filter.setInputCloud (incloud);
  ssn_filter.setRatio (0.3f);
  ssn_filter.filter (outcloud);

  // All the sampled points should have normals along the direction of Z axis
  for (const auto &point : outcloud)
  {
    EXPECT_NEAR (point.normal[0], 0, 1e-3);
    EXPECT_NEAR (point.normal[1], 0, 1e-3);
    EXPECT_NEAR (point.normal[2], 1, 1e-3);
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
  PointXYZ pt (.0f, .0f, .1f);
  input->push_back (pt);

  input->height = 1;
  input->width = input->size ();

	NormalEstimation<PointXYZ, PointNormal> ne;
	ne.setInputCloud (input);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	pcl::PointCloud<PointNormal>::Ptr input_normals (new PointCloud<PointNormal>);
	ne.setKSearch (15);
	ne.compute (*input_normals);

  PointCloud<PointXYZ> output;
  ShadowPoints <PointXYZ, PointNormal> spfilter (true); // Extract removed indices
  spfilter.setInputCloud (input);
  spfilter.setThreshold (0.1f);
  spfilter.setNormals (input_normals);

  spfilter.filter (output);

  // Should filter out the one shadow point that was added.
  EXPECT_EQ (output.size (), 10000);
  pcl::IndicesConstPtr removed = spfilter.getRemovedIndices ();
  EXPECT_EQ (removed->size (), 1);
  EXPECT_EQ (removed->at (0), output.size ());
  // Try organized
  spfilter.setKeepOrganized (true);
  spfilter.filter (output);
  EXPECT_EQ (output.size (), input->size ());
  EXPECT_TRUE (std::isnan (output.at (input->size () - 1).x));
  removed = spfilter.getRemovedIndices ();
  EXPECT_EQ (removed->size (), 1);

  // Now try negative
  spfilter.setKeepOrganized (false);
  spfilter.setNegative (true);
  spfilter.filter (output);
  EXPECT_EQ (output.size (), 1);
  EXPECT_EQ (output.at (0).x, pt.x);
  EXPECT_EQ (output.at (0).y, pt.y);
  EXPECT_EQ (output.at (0).z, pt.z);
  removed = spfilter.getRemovedIndices ();
  EXPECT_EQ (removed->size (), 10000);
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
        input->push_back (pt);
      }
    }
  }
  input->height = 1;
  input->width = input->size ();

  pcl::FrustumCulling<pcl::PointXYZ> fc (true); // Extract removed indices
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
  EXPECT_EQ (output->size (), input->size ());
  pcl::IndicesConstPtr removed;
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), 0);
  // Check negative: no points should remain
  fc.setNegative (true);
  fc.filter (*output);
  EXPECT_EQ (output->size (), 0);
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), input->size ());
  // Make sure organized works
  fc.setKeepOrganized (true);
  fc.filter (*output);
  EXPECT_EQ (output->size (), input->size ());
  for (const auto &point : *output)
  {
    EXPECT_TRUE (std::isnan (point.x));
    EXPECT_TRUE (std::isnan (point.y));
    EXPECT_TRUE (std::isnan (point.z));
  }
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), input->size ());

  // Should filter all points in the input cloud
  fc.setNegative (false);
  fc.setKeepOrganized (false);
  fc.setRegionOfInterest (0.5f, 0.5f, 1.0f, 1.0f);
  fc.filter (*output);
  EXPECT_EQ (output->size (), input->size ());
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), 0);
  // Check invalid ROI values
  EXPECT_THROW (fc.setRegionOfInterest (0.5f, 0.5f, 0.0f, 0.0f), PCLException);
  EXPECT_THROW (fc.setRegionOfInterest (-0.4f, 0.0f, 8.2f, -1.3f), PCLException);

  // Test on real point cloud, cut out milk cartoon in milk_cartoon_all_small_clorox.pcd
  pcl::PointCloud <pcl::PointXYZ>::Ptr model (new pcl::PointCloud <pcl::PointXYZ>);
  pcl::copyPointCloud (*cloud_organized, *model);

  Eigen::Matrix4f cam2robot;
  cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  // Cut out object based on ROI 
  fc.setInputCloud (model);
  fc.setNegative (false);
  fc.setVerticalFOV (43);
  fc.setHorizontalFOV (57);
  fc.setNearPlaneDistance (0);
  fc.setFarPlaneDistance (0.9);
  fc.setRegionOfInterest (0.44f, 0.30f, 0.16f, 0.38f);
  fc.setCameraPose (cam2robot);
  fc.filter (*output);
  // Should extract milk cartoon with 13541 points
  EXPECT_EQ (output->size (), 13541); 
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), model->size () - output->size ());

  // Cut out object based on field of view
  fc.setRegionOfInterest (0.5f, 0.5f, 1.0f, 1.0f); // reset ROI
  fc.setVerticalFOV (-22, 6);
  fc.setHorizontalFOV (-22.5, -13.5);
  fc.filter (*output);
  // Should extract "all" laundry detergent with 10689 points
  EXPECT_EQ (output->size (), 10689);
  removed = fc.getRemovedIndices ();
  EXPECT_EQ (removed->size (), model->size () - output->size ());
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
  ConditionalRemoval<PointXYZ> condrem;
  condrem.setCondition (cyl_cond);
  condrem.setInputCloud (input);
  condrem.setKeepOrganized (false);

  // apply it
  condrem.filter (output);

  EXPECT_EQ (10, output.size ());

  EXPECT_EQ ((*input)[0].x, output[0].x);
  EXPECT_EQ ((*input)[0].y, output[0].y);
  EXPECT_EQ ((*input)[0].z, output[0].z);

  EXPECT_EQ ((*input)[9].x, output[9].x);
  EXPECT_EQ ((*input)[9].y, output[9].y);
  EXPECT_EQ ((*input)[9].z, output[9].z);

  // rotate cylinder comparison along z-axis by PI/2
  cyl_comp->transformComparison (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, float (M_PI) / 2.0f).inverse ());

  condrem.filter (output);

  EXPECT_EQ (4, output.size ());

  EXPECT_EQ ((*input)[0].x, output[0].x);
  EXPECT_EQ ((*input)[0].y, output[0].y);
  EXPECT_EQ ((*input)[0].z, output[0].z);

  EXPECT_EQ ((*input)[3].x, output[3].x);
  EXPECT_EQ ((*input)[3].y, output[3].y);
  EXPECT_EQ ((*input)[3].z, output[3].z);

  // change comparison to a simple plane (x < 5)
  Eigen::Vector3f planeVector;
  planeVector << 1.0, 0.0, 0.0;
  Eigen::Matrix3f planeMatrix = Eigen::Matrix3f::Zero ();
  cyl_comp->setComparisonMatrix (planeMatrix);
  cyl_comp->setComparisonVector (planeVector);
  cyl_comp->setComparisonScalar (-2 * 5.0);
  cyl_comp->setComparisonOperator (ComparisonOps::LT);

  condrem.filter (output);

  EXPECT_EQ (6, output.size ());

  EXPECT_EQ ((*input)[0].x, output[0].x);
  EXPECT_EQ ((*input)[0].y, output[0].y);
  EXPECT_EQ ((*input)[0].z, output[0].z);

  EXPECT_EQ ((*input)[5].x, output[5].x);
  EXPECT_EQ ((*input)[5].y, output[5].y);
  EXPECT_EQ ((*input)[5].z, output[5].z);
}


//////////////////////////////////////////////////////////////////////////////////////////////
TEST (MedianFilter, Filters)
{
  // Create the following cloud
  /* 1   2   3   4   5
   * 6   7   8   9   10
   * 10  9   8   7   6
   * 5   4   3   2   1
   * 100 100 500 100 100
   */
  PointCloud<PointXYZ> cloud_manual;
  cloud_manual.height = 5;
  cloud_manual.width = 5;
  cloud_manual.is_dense = false;
  cloud_manual.resize (5 * 5);

  for (std::size_t i = 0; i < 5; ++i)
  {
    cloud_manual (i, 0).z = static_cast<float> (i + 1);
    cloud_manual (i, 1).z = static_cast<float> (i + 6);
    cloud_manual (i, 2).z = static_cast<float> (10 - i);
    cloud_manual (i, 3).z = static_cast<float> (5 - i);
    cloud_manual (i, 4).z = static_cast<float> (100);
  }
  cloud_manual (2, 4).z = 500;


  MedianFilter<PointXYZ> median_filter;
  median_filter.setInputCloud (cloud_manual.makeShared ());
  median_filter.setWindowSize (3);

  PointCloud<PointXYZ> out_1;
  median_filter.filter (out_1);

  // The result should look like this
  /* 6   6   7   8   9
   * 7   7   7   7   7
   * 7   7   7   7   7
   * 10  9   8   7   7
   * 100 100 500 100 100
   */
  PointCloud<PointXYZ> out_1_correct;
  out_1_correct.height = 5;
  out_1_correct.width = 5;
  out_1_correct.is_dense = false;
  out_1_correct.resize (5 * 5);
  out_1_correct (0, 0).z = 6.f;
  out_1_correct (1, 0).z = 6.f;
  out_1_correct (2, 0).z = 7.f;
  out_1_correct (3, 0).z = 8.f;
  out_1_correct (4, 0).z = 9.f;

  out_1_correct (0, 1).z = 7.f;
  out_1_correct (1, 1).z = 7.f;
  out_1_correct (2, 1).z = 7.f;
  out_1_correct (3, 1).z = 7.f;
  out_1_correct (4, 1).z = 7.f;

  out_1_correct (0, 2).z = 7.f;
  out_1_correct (1, 2).z = 7.f;
  out_1_correct (2, 2).z = 7.f;
  out_1_correct (3, 2).z = 7.f;
  out_1_correct (4, 2).z = 7.f;

  out_1_correct (0, 3).z = 10.f;
  out_1_correct (1, 3).z = 9.f;
  out_1_correct (2, 3).z = 8.f;
  out_1_correct (3, 3).z = 7.f;
  out_1_correct (4, 3).z = 7.f;

  out_1_correct (0, 4).z = 100.f;
  out_1_correct (1, 4).z = 100.f;
  out_1_correct (2, 4).z = 100.f;
  out_1_correct (3, 4).z = 100.f;
  out_1_correct (4, 4).z = 100.f;

  for (std::size_t i = 0; i < 5 * 5; ++i)
    EXPECT_NEAR (out_1_correct[i].z, out_1[i].z, 1e-5);


  // Now limit the maximum value a dexel can change
  PointCloud<PointXYZ> out_2;
  median_filter.setMaxAllowedMovement (50.f);
  median_filter.filter (out_2);

  // The result should look like this
  /* 6   6   7   8   9
   * 7   7   7   7   7
   * 7   7   7   7   7
   * 10  9   8   7   7
   * 100 100 450 100 100
   */
  PointCloud<PointXYZ> out_2_correct;
  out_2_correct = out_1_correct;
  out_2_correct (2, 4).z = 450.f;

  for (std::size_t i = 0; i < 5 * 5; ++i)
    EXPECT_NEAR (out_2_correct[i].z, out_2[i].z, 1e-5);


  // Now load a bigger Kinect cloud and filter it
  MedianFilter<PointXYZRGB> median_filter_xyzrgb;
  median_filter_xyzrgb.setInputCloud (cloud_organized);
  median_filter_xyzrgb.setWindowSize (7);
  median_filter_xyzrgb.setMaxAllowedMovement (0.01f);
  PointCloud<PointXYZRGB> out_3;
  median_filter_xyzrgb.filter (out_3);

  // Check some positions for their values
  EXPECT_NEAR (1.300999999f, out_3(30, 100).z, 1e-5);
  EXPECT_NEAR (1.300999999f, out_3(50, 100).z, 1e-5);
  EXPECT_NEAR (1.305999994f, out_3(90, 100).z, 1e-5);
  EXPECT_NEAR (0.908000111f, out_3(50, 200).z, 1e-5);
  EXPECT_NEAR (0.695000112f, out_3(100, 300).z, 1e-5);
  EXPECT_NEAR (1.177000045f, out_3(128, 128).z, 1e-5);
  EXPECT_NEAR (0.778999984f, out_3(256, 256).z, 1e-5);
  EXPECT_NEAR (0.703000009f, out_3(428, 300).z, 1e-5);
}


//////////////////////////////////////////////////////////////////////////////////////////////
#include <pcl/common/time.h>
TEST (NormalRefinement, Filters)
{
  /*
   * Initialization of parameters
   */

  // Input without NaN
  pcl::PointCloud<pcl::PointXYZRGB> cloud_organized_nonan;
  pcl::Indices dummy;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB> (*cloud_organized, cloud_organized_nonan, dummy);

  // Viewpoint
  const float vp_x = cloud_organized_nonan.sensor_origin_[0];
  const float vp_y = cloud_organized_nonan.sensor_origin_[1];
  const float vp_z = cloud_organized_nonan.sensor_origin_[2];

  // Search parameters
  const int k = 5;
  std::vector<pcl::Indices> k_indices;
  std::vector<std::vector<float> > k_sqr_distances;

  // Estimated and refined normal containers
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_organized_normal;
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_organized_normal_refined;

  /*
   * Neighbor search
   */

  // Search for neighbors
  pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud_organized_nonan.makeShared ());
  kdtree.nearestKSearch (cloud_organized_nonan, pcl::Indices (), k, k_indices, k_sqr_distances);

  /*
   * Estimate normals
   */

  // Run estimation
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  cloud_organized_normal.reserve (cloud_organized_nonan.size ());
  for (index_t i = 0; i < static_cast<index_t>(cloud_organized_nonan.size ()); ++i)
  {
    // Output point
    pcl::PointXYZRGBNormal normali;
    // XYZ and RGB
    std::memcpy (normali.data, cloud_organized_nonan[i].data, 3*sizeof (float));
    normali.rgba = cloud_organized_nonan[i].rgba;
    // Normal
    ne.computePointNormal (cloud_organized_nonan, k_indices[i], normali.normal_x, normali.normal_y, normali.normal_z, normali.curvature);
    pcl::flipNormalTowardsViewpoint (cloud_organized_nonan[i], vp_x, vp_y, vp_z, normali.normal_x, normali.normal_y, normali.normal_z);
    // Store
    cloud_organized_normal.push_back (normali);
  }

  /*
   * Refine normals
   */

  // Run refinement
  pcl::NormalRefinement<pcl::PointXYZRGBNormal> nr (k_indices, k_sqr_distances);
  nr.setInputCloud (cloud_organized_normal.makeShared());
  nr.setMaxIterations (15);
  nr.setConvergenceThreshold (0.00001f);
  nr.filter (cloud_organized_normal_refined);

  /*
   * Find dominant plane in the scene
   */

  // Calculate SAC model
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.005);
  seg.setInputCloud (cloud_organized_normal.makeShared ());
  seg.segment (*inliers, *coefficients);

  // Read out SAC model
  const auto& idx_table = inliers->indices;
  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  const float d = coefficients->values[3];

  // Find a point on the plane [0 0 z] => z = -d / c
  pcl::PointXYZ p_table;
  p_table.x = 0.0f;
  p_table.y = 0.0f;
  p_table.z = -d / c;

  // Use the point to orient the SAC normal correctly
  pcl::flipNormalTowardsViewpoint (p_table, vp_x, vp_y, vp_z, a, b, c);

  /*
   * Test: check that the refined table normals are closer to the SAC model normal
   */

  // Errors for the two normal sets and their means
  std::vector<float> errs_est;
  float err_est_mean = 0.0f;
  std::vector<float> errs_refined;
  float err_refined_mean = 0.0f;

  // Number of zero or NaN vectors produced by refinement
  int num_zeros = 0;
  int num_nans = 0;

  // Loop
  for (const auto &idx : idx_table)
  {
    float tmp;

    // Estimated (need to avoid zeros and NaNs)
    const pcl::PointXYZRGBNormal& calci = cloud_organized_normal[idx];
    if ((std::abs (calci.normal_x) + std::abs (calci.normal_y) + std::abs (calci.normal_z)) > 0.0f)
    {
      tmp = 1.0f - (calci.normal_x * a + calci.normal_y * b + calci.normal_z * c);
      if (std::isfinite (tmp))
      {
        errs_est.push_back (tmp);
        err_est_mean += tmp;
      }
    }

    // Refined
    const pcl::PointXYZRGBNormal& refinedi = cloud_organized_normal_refined[idx];
    if ((std::abs (refinedi.normal_x) + std::abs (refinedi.normal_y) + std::abs (refinedi.normal_z)) > 0.0f)
    {
      tmp = 1.0f - (refinedi.normal_x * a + refinedi.normal_y * b + refinedi.normal_z * c);
      if (std::isfinite(tmp))
      {
        errs_refined.push_back (tmp);
        err_refined_mean += tmp;
      }
      else
      {
        // Non-finite normal encountered
        ++num_nans;
      }
    } else
    {
      // Zero normal encountered
      ++num_zeros;
    }
  }

  // Mean errors
  err_est_mean /= static_cast<float> (errs_est.size ());
  err_refined_mean /= static_cast<float> (errs_refined.size ());

  // Error variance of estimated
  float err_est_var = 0.0f;
  for (const float &err : errs_est)
    err_est_var = (err - err_est_mean) * (err - err_est_mean);
  err_est_var /= static_cast<float> (errs_est.size () - 1);

  // Error variance of refined
  float err_refined_var = 0.0f;
  for (const float &err : errs_refined)
    err_refined_var = (err - err_refined_mean) * (err - err_refined_mean);
  err_refined_var /= static_cast<float> (errs_refined.size () - 1);

  // Refinement should not produce any zeros and NaNs
  EXPECT_EQ(num_zeros, 0);
  EXPECT_EQ(num_nans, 0);

  // Expect mean/variance of error of refined to be smaller, i.e. closer to SAC model
  EXPECT_LT(err_refined_mean, err_est_mean);
  EXPECT_LT(err_refined_var, err_est_var);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 3)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and `milk_cartoon_all_small_clorox.pcd` and pass their paths to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  loadPCDFile (file_name, *cloud_blob);
  fromPCLPointCloud2 (*cloud_blob, *cloud);


  loadPCDFile (argv[2], *cloud_organized);


  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
