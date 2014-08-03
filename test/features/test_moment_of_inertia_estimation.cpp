/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud <pcl::PointXYZ>::Ptr cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MomentOfInertia, FeatureExtraction)
{
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  float major, middle, minor;
  feature_extractor.getEigenValues (major, middle, minor);
  EXPECT_LE (minor, middle);
  EXPECT_LE (middle, major);

  float dot_product = 0.0f;
  Eigen::Vector3f major_vec, middle_vec, minor_vec;
  feature_extractor.getEigenVectors (major_vec, middle_vec, minor_vec);
  dot_product = major_vec.dot (middle_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);
  dot_product = major_vec.dot (minor_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);
  dot_product = middle_vec.dot (minor_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  unsigned int m_size = static_cast <unsigned int> (moment_of_inertia.size ());
  unsigned int e_size = static_cast <unsigned int> (eccentricity.size ());
  EXPECT_EQ (m_size, e_size);
  EXPECT_NE (0, m_size);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MomentOfInertia, InvalidParameters)
{
  float angle_step = 0.0f;
  float point_mass = 0.0f;

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);

  angle_step = 0.0f;
  feature_extractor.setAngleStep (angle_step);
  angle_step = feature_extractor.getAngleStep ();
  EXPECT_LT (0.0f, angle_step);

  angle_step = -1.0f;
  feature_extractor.setAngleStep (angle_step);
  angle_step = feature_extractor.getAngleStep ();
  EXPECT_LT (0.0f, angle_step);

  point_mass = 0.0f;
  feature_extractor.setPointMass (point_mass);
  point_mass = feature_extractor.getPointMass ();
  EXPECT_LT (0.0f, point_mass);

  point_mass = -1.0f;
  feature_extractor.setPointMass (point_mass);
  point_mass = feature_extractor.getPointMass ();
  EXPECT_LT (0.0f, point_mass);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `lamppost.pcd` and pass its paths to the test." << std::endl;
    return (-1);
  }

  cloud = (new pcl::PointCloud<pcl::PointXYZ> ())->makeShared ();
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `lamppost.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
