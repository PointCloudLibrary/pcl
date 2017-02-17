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
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/eigen.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;

typedef PointXYZ PointType;
typedef Normal NormalType;
typedef ReferenceFrame RFType;
typedef SHOT352 DescriptorType;

PointCloud<PointType>::Ptr model_ (new PointCloud<PointType> ());
PointCloud<PointType>::Ptr model_downsampled_ (new PointCloud<PointType> ());
PointCloud<PointType>::Ptr scene_ (new PointCloud<PointType> ());
PointCloud<PointType>::Ptr scene_downsampled_ (new PointCloud<PointType> ());
PointCloud<NormalType>::Ptr model_normals_ (new PointCloud<NormalType> ());
PointCloud<NormalType>::Ptr scene_normals_ (new PointCloud<NormalType> ());
PointCloud<DescriptorType>::Ptr model_descriptors_ (new PointCloud<DescriptorType> ());
PointCloud<DescriptorType>::Ptr scene_descriptors_ (new PointCloud<DescriptorType> ());
CorrespondencesPtr model_scene_corrs_ (new Correspondences ());

double
computeRmsE (const PointCloud<PointType>::ConstPtr &model, const PointCloud<PointType>::ConstPtr &scene, const Eigen::Matrix4f &rototranslation)
{
  PointCloud<PointType> transformed_model;
  transformPointCloud (*model, transformed_model, rototranslation);

  KdTreeFLANN<PointType> tree;
  tree.setInputCloud (scene);

  double sqr_norm_sum = 0;
  int found_points = 0;

  vector<int> neigh_indices (1);
  vector<float> neigh_sqr_dists (1);
  for (size_t i = 0; i < transformed_model.size (); ++i)
  {

    int found_neighs = tree.nearestKSearch (transformed_model.at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1)
    {
      ++found_points;
      sqr_norm_sum += static_cast<double> (neigh_sqr_dists[0]);
    }
  }

  if (found_points > 0)
    return sqrt (sqr_norm_sum / double (transformed_model.size ()));

  return numeric_limits<double>::max ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Hough3DGrouping)
{
  PointCloud<RFType>::Ptr model_rf (new PointCloud<RFType> ());
  PointCloud<RFType>::Ptr scene_rf (new PointCloud<RFType> ());

  //RFs
  BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
  rf_est.setRadiusSearch (0.015);
  rf_est.setInputCloud (model_downsampled_);
  rf_est.setInputNormals (model_normals_);
  rf_est.setSearchSurface (model_);
  rf_est.compute (*model_rf);

  rf_est.setInputCloud (scene_downsampled_);
  rf_est.setInputNormals (scene_normals_);
  rf_est.setSearchSurface (scene_);
  rf_est.compute (*scene_rf);

  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

  //Actual CG
  Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
  clusterer.setInputCloud (model_downsampled_);
  clusterer.setInputRf (model_rf);
  clusterer.setSceneCloud (scene_downsampled_);
  clusterer.setSceneRf (scene_rf);
  clusterer.setModelSceneCorrespondences (model_scene_corrs_);
  clusterer.setHoughBinSize (0.03);
  clusterer.setHoughThreshold (25);
  EXPECT_TRUE (clusterer.recognize (rototranslations));

  //Assertions
  EXPECT_EQ (rototranslations.size (), 1);
  EXPECT_LT (computeRmsE (model_, scene_, rototranslations[0]), 1E-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GeometricConsistencyGrouping)
{
  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

  GeometricConsistencyGrouping<PointType, PointType> clusterer;
  clusterer.setInputCloud (model_downsampled_);
  clusterer.setSceneCloud (scene_downsampled_);
  clusterer.setModelSceneCorrespondences (model_scene_corrs_);
  clusterer.setGCSize (0.015);
  clusterer.setGCThreshold (25);
  EXPECT_TRUE (clusterer.recognize (rototranslations));

  //Assertions
  EXPECT_EQ (rototranslations.size (), 1);
  EXPECT_LT (computeRmsE (model_, scene_, rototranslations[0]), 1E-4);
}


/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    cerr << "No test file given. Please download `milk.pcd` and `milk_cartoon_all_small_clorox.pcd` and pass their paths to the test." << endl;
    return (-1);
  }

  if (loadPCDFile (argv[1], *model_) < 0)
  {
    cerr << "Failed to read test file. Please download `milk.pcd` and pass its path to the test." << endl;
    return (-1);
  }

  if (loadPCDFile (argv[2], *scene_) < 0)
  {
    cerr << "Failed to read test file. Please download `milk_cartoon_all_small_clorox.pcd` and pass its path to the test." << endl;
    return (-1);
  }

  //Normals
  NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model_);
  norm_est.compute (*model_normals_);

  norm_est.setInputCloud (scene_);
  norm_est.compute (*scene_normals_);

  //Downsampling
  UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model_);
  uniform_sampling.setRadiusSearch (0.005);
  uniform_sampling.filter (*model_downsampled_);

  uniform_sampling.setInputCloud (scene_);
  uniform_sampling.setRadiusSearch (0.02);
  uniform_sampling.filter (*scene_downsampled_);

  //Descriptor
  SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (0.015);
  descr_est.setInputCloud (model_downsampled_);
  descr_est.setInputNormals (model_normals_);
  descr_est.setSearchSurface (model_);
  descr_est.compute (*model_descriptors_);

  descr_est.setInputCloud (scene_downsampled_);
  descr_est.setInputNormals (scene_normals_);
  descr_est.setSearchSurface (scene_);
  descr_est.compute (*scene_descriptors_);

  //Correspondences with KdTree
  KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors_);

  for (size_t i = 0; i < scene_descriptors_->size (); ++i)
  {
    if ( pcl_isfinite( scene_descriptors_->at (i).descriptor[0] ) )
    {
      vector<int> neigh_indices (1);
      vector<float> neigh_sqr_dists (1);
      int found_neighs = match_search.nearestKSearch (scene_descriptors_->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
      {
        Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs_->push_back (corr);
      }
    }
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
