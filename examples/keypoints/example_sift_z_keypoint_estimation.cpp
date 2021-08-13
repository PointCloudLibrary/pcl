/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 *
 */

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>

/* This examples shows how to estimate the SIFT points based on the 
 * z gradient of the 3D points than using the Intensity gradient as
 * usually used for SIFT keypoint estimation.
 */

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
	return p.z;
      }
    };
}

int
main(int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1) // load the file
  {
    PCL_ERROR("Couldn't read file\n");
    return -1;
  }
  std::cout << "points: " << cloud_xyz->size () <<std::endl;
  
  // Parameters for sift computation
  const float min_scale = 0.005f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.005f;
  
  // Estimate the sift interest points using z values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_xyz);
  sift.compute(result);

  std::cout << "No of SIFT points in the result are " << result.size () << std::endl;

/*
  // Copying the pointwithscale to pointxyz so as visualize the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(result, *cloud_temp);
  std::cout << "SIFT points in the result are " << cloud_temp->size () << std::endl;
  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud_xyz, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(cloud_xyz, cloud_color_handler, "cloud");
  viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
*/

  return 0;
  
}
