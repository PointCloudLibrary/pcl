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
 * $Id: $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr training_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud;
pcl::PointCloud<pcl::Normal>::Ptr training_normals;
pcl::PointCloud<pcl::Normal>::Ptr testing_normals;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ISM, TrainRecognize)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr > normals;
  std::vector<unsigned int> classes;

  clouds.push_back (training_cloud);
  normals.push_back (training_normals);
  classes.push_back (0);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model (new pcl::features::ISMModel);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator);
  ism.setTrainingClouds (clouds);
  ism.setTrainingClasses (classes);
  ism.setTrainingNormals (normals);
  ism.setSamplingSize (2.0f);
  ism.trainISM (model);

  int _class = 0;
  double radius = model->sigmas_[_class] * 10.0;
  double sigma = model->sigmas_[_class];

  auto vote_list = ism.findObjects (model, testing_cloud, testing_normals, _class);
  EXPECT_NE (vote_list->getNumberOfVotes (), 0);
  std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
  vote_list->findStrongestPeaks (strongest_peaks, _class, radius, sigma);

  EXPECT_NE (strongest_peaks.size (), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (ISM, TrainWithWrongParameters)
{
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;

  float prev_sampling_size = ism.getSamplingSize ();
  EXPECT_NE (prev_sampling_size, 0.0);
  ism.setSamplingSize (0.0f);
  float curr_sampling_size = ism.getSamplingSize ();
  EXPECT_EQ (curr_sampling_size, prev_sampling_size);

  unsigned int prev_number_of_clusters = ism.getNumberOfClusters ();
  EXPECT_NE (prev_number_of_clusters, 0);
  ism.setNumberOfClusters (0);
  unsigned int curr_number_of_clusters = ism.getNumberOfClusters ();
  EXPECT_EQ (curr_number_of_clusters, prev_number_of_clusters);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "This test requires two point clouds (one for training and one for testing)." << std::endl;
    std::cerr << "You can use these two clouds 'ism_train.pcd' and 'ism_test.pcd'." << std::endl;
    return (-1);
  }

  training_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile (argv[1], *training_cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `ism_train.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  testing_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile (argv[2], *testing_cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `ism_test.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  training_normals.reset (new pcl::PointCloud<pcl::Normal>);
  testing_normals.reset (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);
  normal_estimator.setInputCloud(training_cloud);
  normal_estimator.compute(*training_normals);
  normal_estimator.setInputCloud(testing_cloud);
  normal_estimator.compute(*testing_normals);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
