/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2016-, Open Perception, Inc.
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
#include <pcl/registration/ia_ReLOC.h>
#include <pcl/common/geometry.h>

#include <vector>

pcl::PointCloud<pcl::PointXYZ> cloud_source, cloud_target;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ReLOCInitialAlignment)
{
  Eigen::Matrix4f transform_to_estimate = Eigen::Matrix4f::Identity ();
  float theta = M_PI / 4;
  transform_to_estimate (0, 0) = cos (theta);
  transform_to_estimate (0, 1) = -sin (theta);
  transform_to_estimate (1, 0) = sin (theta);
  transform_to_estimate (1, 1) = cos (theta);
  transform_to_estimate (0, 3) = 0.4;
  transform_to_estimate (1, 3) = 0.5;

  //rototranslate the source cloud by the inverse of the transform to estimate
  pcl::PointCloud<pcl::PointXYZ> cloud_source_transformed;
  pcl::transformPointCloud (cloud_source, cloud_source_transformed, transform_to_estimate.inverse ());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr = cloud_source_transformed.makeShared ();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_ptr = cloud_target.makeShared ();

  const float mesh_res = 0.004f;

  // Compute source normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> source_ne;
  source_ne.setRadiusSearch (2.0f*mesh_res);

  //get centroid of source cloud to disambiguate normals
  Eigen::Vector4f source_centroid;
  pcl::compute3DCentroid (*cloud_source_ptr, source_centroid);
  source_ne.setViewPoint (source_centroid[0], source_centroid[1], source_centroid[2]);

  source_ne.setInputCloud (cloud_source_ptr);

  pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal> ());
  source_ne.compute (*source_normals);
  //flip normals
  for (int no = 0; no < source_normals->size (); no++)
  {
    (*source_normals)[no].getNormalVector3fMap () *= -1;
  }

  // Compute target normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> target_ne;
  target_ne.setRadiusSearch (2.0f*mesh_res);

  //get centroid of target cloud to disambiguate normals
  Eigen::Vector4f target_centroid;
  pcl::compute3DCentroid (*cloud_target_ptr, target_centroid);
  target_ne.setViewPoint (target_centroid[0], target_centroid[1], target_centroid[2]);

  target_ne.setInputCloud (cloud_target_ptr);

  pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal> ());
  target_ne.compute (*target_normals);
  //flip normals
  for (int no = 0; no < target_normals->size (); no++)
  {
    (*target_normals)[no].getNormalVector3fMap () *= -1;
  }

  // initialize ReLOC
  pcl::PointCloud <pcl::PointXYZ> source_aligned;
  pcl::registration::ReLOCInitialAlignment <pcl::PointXYZ, pcl::PointXYZ> ReLOC_ia;
  ReLOC_ia.setSeed (0);
  ReLOC_ia.setInputSource (cloud_source_ptr);
  ReLOC_ia.setInputTarget (cloud_target_ptr);
  ReLOC_ia.setSourceNormals (source_normals);
  ReLOC_ia.setTargetNormals (target_normals);
  ReLOC_ia.setFlatKeypointRf (3.0*mesh_res); //5 in the paper
  ReLOC_ia.setFlatKeypointRdiscard (2.0*mesh_res);
  ReLOC_ia.setFlatKeypointR1search (2.0*mesh_res);
  ReLOC_ia.setFlatKeypointR2search (20.0*mesh_res);

  ReLOC_ia.setUseRandomDetector (true); // as the clouds have a limited number of points, a random extraction is sufficient
  ReLOC_ia.setNrandomKeypoints (300);

  ReLOC_ia.setFlareNormalRadius (5.0*mesh_res);
  ReLOC_ia.setFlareTangentRadius (20.0*mesh_res);
  ReLOC_ia.setHoughSbin (2.0*mesh_res);
  ReLOC_ia.setRansacT (8.0*mesh_res);

  // perform alignment
  ReLOC_ia.align (source_aligned);

  //Compare source and transformed source
  for (int po = 0; po < cloud_source_ptr->size (); po++)
  {
    float dist = pcl::geometry::distance (cloud_source.at (0), source_aligned.at (0));
    EXPECT_LT (dist, 0.05f);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` pass their path to the test." << std::endl;
    return (-1);
  }

  // Input
  if (pcl::io::loadPCDFile (argv[1], cloud_source) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (argv[2], cloud_target) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
