/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#include <pcl/test/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/model_outlier_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

/* Expectation:
 A model found by ransac has the same inliers and outliers as the same model filtered with model_outlier_removal
 as long as the thresholdfunction of ransac and model_outlier_removal is the same */

using namespace pcl;

PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ModelOutlierRemoval, Model_Outlier_Filter)
{
  PointCloud<PointXYZ>::Ptr cloud_filter_out (new PointCloud<PointXYZ>);
  pcl::Indices ransac_inliers;
  float thresh = 0.01;
  //run ransac
  Eigen::VectorXf model_coefficients;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_in));
  RandomSampleConsensus < pcl::PointXYZ > ransac (model_p);
  ransac.setDistanceThreshold (thresh);
  ransac.computeModel ();
  ransac.getInliers (ransac_inliers);
  ransac.getModelCoefficients (model_coefficients);
  // test ransacs result
  EXPECT_EQ (model_coefficients.size (), 4);
  if (model_coefficients.size () != 4)
    return;
  //run filter
  pcl::ModelCoefficients model_coeff;
  model_coeff.values.resize (4);
  for (int i = 0; i < 4; i++)
    model_coeff.values[i] = model_coefficients[i];
  pcl::ModelOutlierRemoval < pcl::PointXYZ > filter;
  filter.setModelCoefficients (model_coeff);
  filter.setThreshold (thresh);
  filter.setModelType (pcl::SACMODEL_PLANE);
  filter.setInputCloud (cloud_in);
  filter.filter (*cloud_filter_out);
  //compare results
  EXPECT_EQ (cloud_filter_out->size (), ransac_inliers.size ());
  //TODO: also compare content
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `milk_cartoon_all_small_clorox.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  io::loadPCDFile (file_name, *cloud_in);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
