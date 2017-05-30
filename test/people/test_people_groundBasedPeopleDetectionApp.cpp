/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Test file for testing people detection on a point cloud.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum { COLS = 640, ROWS = 480 };
PointCloudT::Ptr cloud;
pcl::people::PersonClassifier<pcl::RGB> person_classifier;
std::string svm_filename;
float min_confidence;
float min_width;
float max_width;
float min_height;
float max_height;
float voxel_size;
Eigen::Matrix3f rgb_intrinsics_matrix;
Eigen::VectorXf ground_coeffs;

TEST (PCL, PersonClassifier)
{
  // Create classifier for people detection:
  EXPECT_TRUE (person_classifier.loadSVMFromFile(svm_filename)); // load trained SVM
}

TEST (PCL, GroundBasedPeopleDetectionApp)
{
  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setPersonClusterLimits(min_height, max_height, min_width, max_width);

  // Perform people detection on the new cloud:
  std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
  people_detector.setInputCloud(cloud);
  people_detector.setGround(ground_coeffs);                    // set floor coefficients
  EXPECT_TRUE (people_detector.compute(clusters));             // perform people detection

  unsigned int k = 0;
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
      k++;
  }
  EXPECT_EQ (k, 5);		// verify number of people found (should be five)
}

int main (int argc, char** argv)
{
  if (argc < 2)
  {
    cerr << "No svm filename provided. Please download `trainedLinearSVMForPeopleDetectionWithHOG.yaml` and pass its path to the test." << endl;
    return (-1);
  }
  	
  if (argc < 3)
  {
    cerr << "No test file given. Please download 'five_people.pcd` and pass its path to the test." << endl;
    return (-1);
  }

  cloud = PointCloudT::Ptr (new PointCloudT);
  if (pcl::io::loadPCDFile (argv[2], *cloud) < 0)
  {
    cerr << "Failed to read test file. Please download `five_people.pcd` and pass its path to the test." << endl;
    return (-1);
  }	
	
  // Algorithm parameters:
  svm_filename = argv[1];
  min_confidence = -1.5;
  min_width = 0.1;
  max_width = 8.0;
  min_height = 1.3;
  max_height = 2.3;
  voxel_size = 0.06;

  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
  ground_coeffs.resize(4);
  ground_coeffs << -0.0103586, 0.997011, 0.0765573, -1.26614;			// set ground coefficients

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
