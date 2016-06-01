/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014-, Open Perception, Inc.
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
*/

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ReLOC.h>

#include <pcl/common/geometry.h>

#include <vector>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace std;



PointCloud<PointXYZ> cloud_source, cloud_target;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ReLOCInitialAlignment)
{

  Eigen::Matrix4f transform_to_estimate = Eigen::Matrix4f::Identity();
  float theta = M_PI/4;
  transform_to_estimate (0,0) = cos (theta);
  transform_to_estimate (0,1) = -sin(theta);
  transform_to_estimate (1,0) = sin (theta);
  transform_to_estimate (1,1) = cos (theta);
  transform_to_estimate (0,3) = 0.4;
  transform_to_estimate (1,3) = 0.5;

  // transform the source cloud by a large amount
  Eigen::Vector3f initial_offset (1.f, 0.f, 0.f);
  float angle = static_cast<float> (M_PI) / 2.f;
  Eigen::Quaternionf initial_rotation (cos (angle / 2.f), 0, 0, sin (angle / 2.f));
  PointCloud<PointXYZ> cloud_source_transformed;
  //transformPointCloud (cloud_source, cloud_source_transformed, initial_offset, initial_rotation);

  Eigen::Matrix4f inverse_transform = transform_to_estimate.inverse();
  transformPointCloud (cloud_source, cloud_source_transformed, inverse_transform);

  // create shared pointers
  PointCloud<PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = cloud_source_transformed.makeShared ();
  cloud_target_ptr = cloud_target.makeShared ();

  float meshRes = 0.004f;




  // Compute source normals
	vector<int> source_indices (cloud_source_ptr->points.size ());
	for (size_t i = 0; i < source_indices.size (); ++i)
		source_indices[i] = static_cast<int> (i);
	boost::shared_ptr<vector<int> > source_indices_ptr (new vector<int> (source_indices));

	NormalEstimation<PointXYZ, Normal> source_ne;
	source_ne.setRadiusSearch (2.0f*meshRes);
	source_ne.setIndices (source_indices_ptr);

	//get centroid of source cloud to disambiguate normals
	Eigen::Vector4f source_centroid; 
	pcl::compute3DCentroid(*cloud_source_ptr, source_centroid); 

	source_ne.setViewPoint (source_centroid[0], source_centroid[1], source_centroid[2]);
	source_ne.setInputCloud (cloud_source_ptr);

	PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal> ());
	source_ne.compute (*source_normals);
	//disambiguate normals
	for(int no=0; no<source_normals->size(); no++)
	{
		(*source_normals)[no].getNormalVector3fMap() *= -1;
	}


  // Compute target normals
	vector<int> target_indices (cloud_target_ptr->points.size ());
	for (size_t i = 0; i < target_indices.size (); ++i)
		target_indices[i] = static_cast<int> (i);
	boost::shared_ptr<vector<int> > target_indices_ptr (new vector<int> (target_indices));

	NormalEstimation<PointXYZ, Normal> target_ne;
	target_ne.setRadiusSearch (2.0f*meshRes);
	target_ne.setIndices (target_indices_ptr);

	//get centroid of target cloud to disambiguate normals
	Eigen::Vector4f target_centroid; 
	pcl::compute3DCentroid(*cloud_target_ptr, target_centroid); 

	target_ne.setViewPoint (target_centroid[0], target_centroid[1], target_centroid[2]);
	target_ne.setInputCloud (cloud_target_ptr);

	PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal> ());
	target_ne.compute (*target_normals);
	//disambiguate normals
	for(int no=0; no<target_normals->size(); no++)
	{
		(*target_normals)[no].getNormalVector3fMap() *= -1;
	}



  
  
  // initialize ReLOC
  PointCloud <PointXYZ> source_aligned;
  ReLOCInitialAlignment <PointXYZ, PointXYZ> ReLOC_ia;
  ReLOC_ia.setSeed(0);
  ReLOC_ia.setInputSource (cloud_source_ptr);
  ReLOC_ia.setInputTarget (cloud_target_ptr);
	ReLOC_ia.setSourceNormals (source_normals); 
	ReLOC_ia.setTargetNormals (target_normals); 
  ReLOC_ia.setFlatKeypointRf (3.0*meshRes); //5 in the paper
  ReLOC_ia.setFlatKeypointRdiscard (2.0*meshRes);
  ReLOC_ia.setFlatKeypointR1search (2.0*meshRes);
  ReLOC_ia.setFlatKeypointR2search (20.0*meshRes);

  ReLOC_ia.useRandomDetector(true); // as the clouds have a limited number of points, a random extraction is sufficient
  ReLOC_ia.setNrandomKeypoints(300);

  ReLOC_ia.setFlareNormalRadius (5.0*meshRes);
  ReLOC_ia.setFlareTangentRadius (20.0*meshRes);
  ReLOC_ia.setHoughSbin (2.0*meshRes);
  ReLOC_ia.setRansacT (8.0*meshRes);


  // align
  ReLOC_ia.align (source_aligned);




  //Compare source and transformed source
  for(int po=0; po<cloud_source_ptr->size(); po++)
  {
    float dist = pcl::geometry::distance(cloud_source.at(0), source_aligned.at(0));
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
  if (loadPCDFile (argv[1], cloud_source) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (loadPCDFile (argv[2], cloud_target) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
