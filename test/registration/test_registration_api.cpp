/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <limits>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/eigen.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d.h>

#include "test_registration_api_data.h"

typedef pcl::PointXYZ              PointXYZ;
typedef pcl::PointCloud <PointXYZ> CloudXYZ;
typedef CloudXYZ::Ptr              CloudXYZPtr;
typedef CloudXYZ::ConstPtr         CloudXYZConstPtr;

typedef pcl::PointNormal              PointNormal;
typedef pcl::PointCloud <PointNormal> CloudNormal;
typedef CloudNormal::Ptr              CloudNormalPtr;
typedef CloudNormal::ConstPtr         CloudNormalConstPtr;

CloudXYZ cloud_source, cloud_target, cloud_reg;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceEstimation)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  // check for correct order and number of matches
  EXPECT_EQ (int (correspondences->size ()), nr_original_correspondences);
  if (int (correspondences->size ()) == nr_original_correspondences)
  {
    for (int i = 0; i < nr_original_correspondences; ++i)
    {
      EXPECT_EQ ((*correspondences)[i].index_query, i);
      EXPECT_EQ ((*correspondences)[i].index_match, correspondences_original[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceEstimationReciprocal)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineReciprocalCorrespondences (*correspondences);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences->size ()), nr_reciprocal_correspondences);
  if (int (correspondences->size ()) == nr_reciprocal_correspondences)
  {
    for (int i = 0; i < nr_reciprocal_correspondences; ++i)
    {
      EXPECT_EQ ((*correspondences)[i].index_query, correspondences_reciprocal[i][0]);
      EXPECT_EQ ((*correspondences)[i].index_match, correspondences_reciprocal[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorDistance)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_dist (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;
  corr_rej_dist.setInputCorrespondences(correspondences);
  corr_rej_dist.setMaximumDistance(rej_dist_max_dist);
  corr_rej_dist.getCorrespondences(*correspondences_result_rej_dist);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences_result_rej_dist->size ()), nr_correspondences_result_rej_dist);
  if (int (correspondences_result_rej_dist->size ()) == nr_correspondences_result_rej_dist)
  {
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_dist)[i].index_query, correspondences_dist[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_dist)[i].index_match, correspondences_dist[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorMedianDistance)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_median_dist (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorMedianDistance corr_rej_median_dist;
  corr_rej_median_dist.setInputCorrespondences(correspondences);
  corr_rej_median_dist.setMedianFactor (rej_median_factor);

  corr_rej_median_dist.getCorrespondences(*correspondences_result_rej_median_dist);

  // check for correct matches
  EXPECT_NEAR (corr_rej_median_dist.getMedianDistance (), rej_median_distance, 1e-4);
  EXPECT_EQ (int (correspondences_result_rej_median_dist->size ()), nr_correspondences_result_rej_median_dist);
  if (int (correspondences_result_rej_median_dist->size ()) == nr_correspondences_result_rej_median_dist)
  {
    for (int i = 0; i < nr_correspondences_result_rej_median_dist; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_median_dist)[i].index_query, correspondences_median_dist[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_median_dist)[i].index_match, correspondences_median_dist[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorOneToOne)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_one_to_one (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
  corr_rej_one_to_one.setInputCorrespondences(correspondences);
  corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences_result_rej_one_to_one->size ()), nr_correspondences_result_rej_one_to_one);
  if (int (correspondences_result_rej_one_to_one->size ()) == nr_correspondences_result_rej_one_to_one)
  {
    for (int i = 0; i < nr_correspondences_result_rej_one_to_one; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_one_to_one)[i].index_query, correspondences_one_to_one[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_one_to_one)[i].index_match, correspondences_one_to_one[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorSampleConsensus)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);
  EXPECT_EQ (int (correspondences->size ()), nr_original_correspondences);

  boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZ> corr_rej_sac;
  corr_rej_sac.setInputSource (source);
  corr_rej_sac.setInputTarget (target);
  corr_rej_sac.setInlierThreshold (rej_sac_max_dist);
  corr_rej_sac.setMaximumIterations (rej_sac_max_iter);
  corr_rej_sac.setInputCorrespondences (correspondences);
  corr_rej_sac.getCorrespondences (*correspondences_result_rej_sac);
  Eigen::Matrix4f transform_res_from_SAC = corr_rej_sac.getBestTransformation ();

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences_result_rej_sac->size ()), nr_correspondences_result_rej_sac);
  if (int (correspondences_result_rej_sac->size ()) == nr_correspondences_result_rej_sac)
  {
    for (int i = 0; i < nr_correspondences_result_rej_sac; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_sac)[i].index_query, correspondences_sac[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_sac)[i].index_match, correspondences_sac[i][1]);
    }
  }

  // check for correct transformation
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NEAR (transform_res_from_SAC (i, j), transform_from_SAC[i][j], 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorSurfaceNormal)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);


  CloudNormalPtr source_normals(new CloudNormal ());
  pcl::copyPointCloud(*source, *source_normals);
  CloudNormalPtr target_normals(new CloudNormal ());
  pcl::copyPointCloud(*target, *target_normals);

  pcl::NormalEstimation<PointNormal, PointNormal> norm_est_src;
  norm_est_src.setSearchMethod (pcl::search::KdTree<PointNormal>::Ptr (new pcl::search::KdTree<PointNormal>));
  norm_est_src.setKSearch (10);
  norm_est_src.setInputCloud (source_normals);
  norm_est_src.compute (*source_normals);

  pcl::NormalEstimation<PointNormal, PointNormal> norm_est_tgt;
  norm_est_tgt.setSearchMethod (pcl::search::KdTree<PointNormal>::Ptr (new pcl::search::KdTree<PointNormal>));
  norm_est_tgt.setKSearch (10);
  norm_est_tgt.setInputCloud (target_normals);
  norm_est_tgt.compute (*target_normals);

  pcl::registration::CorrespondenceRejectorSurfaceNormal  corr_rej_surf_norm;
  corr_rej_surf_norm.initializeDataContainer <PointXYZ, PointNormal> ();
  corr_rej_surf_norm.setInputSource <PointXYZ> (source);
  corr_rej_surf_norm.setInputTarget <PointXYZ> (target);
  corr_rej_surf_norm.setInputNormals <PointXYZ, PointNormal> (source_normals);
  corr_rej_surf_norm.setTargetNormals <PointXYZ, PointNormal> (target_normals);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_surf_norm (new pcl::Correspondences);
  corr_rej_surf_norm.setInputCorrespondences (correspondences);
  corr_rej_surf_norm.setThreshold (0.5);

  corr_rej_surf_norm.getCorrespondences (*correspondences_result_rej_surf_norm);

  // check for correct matches
  if (int (correspondences_result_rej_surf_norm->size ()) == nr_correspondences_result_rej_dist)
  {
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_surf_norm)[i].index_query, correspondences_dist[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_surf_norm)[i].index_match, correspondences_dist[i][1]);
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorTrimmed)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_trimmed (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorTrimmed corr_rej_trimmed;
  corr_rej_trimmed.setOverlapRadio(rej_trimmed_overlap);
  corr_rej_trimmed.setInputCorrespondences(correspondences);
  corr_rej_trimmed.getCorrespondences(*correspondences_result_rej_trimmed);

  // check for correct matches, number of matches, and for sorting (correspondences should be sorted w.r.t. distance)
  EXPECT_EQ (int (correspondences_result_rej_trimmed->size ()), nr_correspondences_result_rej_trimmed);
  if (int (correspondences_result_rej_trimmed->size ()) == nr_correspondences_result_rej_trimmed)
  {
    for (int i = 0; i < nr_correspondences_result_rej_trimmed; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_trimmed)[i].index_query, correspondences_trimmed[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_trimmed)[i].index_match, correspondences_trimmed[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorVarTrimmed)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_source));
  CloudXYZConstPtr target (new CloudXYZ (cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointXYZ, PointXYZ> corr_est;
  corr_est.setInputSource (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_var_trimmed_dist (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorVarTrimmed corr_rej_var_trimmed_dist;
  corr_rej_var_trimmed_dist.setInputSource<PointXYZ> (source);
  corr_rej_var_trimmed_dist.setInputTarget<PointXYZ> (target);
  corr_rej_var_trimmed_dist.setInputCorrespondences(correspondences);

  corr_rej_var_trimmed_dist.getCorrespondences(*correspondences_result_rej_var_trimmed_dist);

  // check for correct matches
  if (int (correspondences_result_rej_var_trimmed_dist->size ()) == nr_correspondences_result_rej_dist)
  {
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
    {
      EXPECT_EQ ((*correspondences_result_rej_var_trimmed_dist)[i].index_query, correspondences_dist[i][0]);
      EXPECT_EQ ((*correspondences_result_rej_var_trimmed_dist)[i].index_match, correspondences_dist[i][1]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformationEstimationSVD)
{
  // Ideal conditions for the estimation (no noise, exact correspondences)
  CloudXYZConstPtr source (new CloudXYZ (cloud_target));
  CloudXYZPtr      target (new CloudXYZ ());
  pcl::transformPointCloud (*source, *target, T_ref);

  Eigen::Matrix4f T_SVD_1;
  const pcl::registration::TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est_svd;
  trans_est_svd.estimateRigidTransformation(*source, *target, T_SVD_1);

  const Eigen::Quaternionf   R_SVD_1 (T_SVD_1.topLeftCorner  <3, 3> ());
  const Eigen::Translation3f t_SVD_1 (T_SVD_1.topRightCorner <3, 1> ());

  EXPECT_NEAR (R_SVD_1.x (), R_ref.x (), 1e-6f);
  EXPECT_NEAR (R_SVD_1.y (), R_ref.y (), 1e-6f);
  EXPECT_NEAR (R_SVD_1.z (), R_ref.z (), 1e-6f);
  EXPECT_NEAR (R_SVD_1.w (), R_ref.w (), 1e-6f);

  EXPECT_NEAR (t_SVD_1.x (), t_ref.x (), 1e-6f);
  EXPECT_NEAR (t_SVD_1.y (), t_ref.y (), 1e-6f);
  EXPECT_NEAR (t_SVD_1.z (), t_ref.z (), 1e-6f);

  // Check if the estimation with correspondences gives the same results
  Eigen::Matrix4f T_SVD_2;
  pcl::Correspondences corr; corr.reserve (source->size ());
  for (int i=0; i<source->size (); ++i) corr.push_back (pcl::Correspondence (i, i, 0.f));
  trans_est_svd.estimateRigidTransformation(*source, *target, corr, T_SVD_2);

  const Eigen::Quaternionf   R_SVD_2 (T_SVD_2.topLeftCorner  <3, 3> ());
  const Eigen::Translation3f t_SVD_2 (T_SVD_2.topRightCorner <3, 1> ());

  EXPECT_FLOAT_EQ (R_SVD_1.x (), R_SVD_2.x ());
  EXPECT_FLOAT_EQ (R_SVD_1.y (), R_SVD_2.y ());
  EXPECT_FLOAT_EQ (R_SVD_1.z (), R_SVD_2.z ());
  EXPECT_FLOAT_EQ (R_SVD_1.w (), R_SVD_2.w ());

  EXPECT_FLOAT_EQ (t_SVD_1.x (), t_SVD_2.x ());
  EXPECT_FLOAT_EQ (t_SVD_1.y (), t_SVD_2.y ());
  EXPECT_FLOAT_EQ (t_SVD_1.z (), t_SVD_2.z ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformationEstimationLM)
{
  CloudXYZConstPtr source (new CloudXYZ (cloud_target));
  CloudXYZPtr      target (new CloudXYZ ());
  pcl::transformPointCloud (*source, *target, T_ref);

  Eigen::Matrix4f T_LM;
  const pcl::registration::TransformationEstimationLM<PointXYZ, PointXYZ> trans_est_lm;
  trans_est_lm.estimateRigidTransformation(*source, *target, T_LM);

  const Eigen::Quaternionf   R_LM_1 (T_LM.topLeftCorner  <3, 3> ());
  const Eigen::Translation3f t_LM_1 (T_LM.topRightCorner <3, 1> ());

  EXPECT_NEAR (R_LM_1.x (), R_ref.x (), 1e-6f);
  EXPECT_NEAR (R_LM_1.y (), R_ref.y (), 1e-6f);
  EXPECT_NEAR (R_LM_1.z (), R_ref.z (), 1e-6f);
  EXPECT_NEAR (R_LM_1.w (), R_ref.w (), 1e-6f);

  EXPECT_NEAR (t_LM_1.x (), t_ref.x (), 1e-6f);
  EXPECT_NEAR (t_LM_1.y (), t_ref.y (), 1e-6f);
  EXPECT_NEAR (t_LM_1.z (), t_ref.z (), 1e-6f);

  // Check if the estimation with correspondences gives the same results
  Eigen::Matrix4f T_LM_2;
  pcl::Correspondences corr; corr.reserve (source->size ());
  for (int i=0; i<source->size (); ++i) corr.push_back (pcl::Correspondence (i, i, 0.f));
  trans_est_lm.estimateRigidTransformation(*source, *target, corr, T_LM_2);

  const Eigen::Quaternionf   R_LM_2 (T_LM_2.topLeftCorner  <3, 3> ());
  const Eigen::Translation3f t_LM_2 (T_LM_2.topRightCorner <3, 1> ());

  EXPECT_FLOAT_EQ (R_LM_1.x (), R_LM_2.x ());
  EXPECT_FLOAT_EQ (R_LM_1.y (), R_LM_2.y ());
  EXPECT_FLOAT_EQ (R_LM_1.z (), R_LM_2.z ());
  EXPECT_FLOAT_EQ (R_LM_1.w (), R_LM_2.w ());

  EXPECT_FLOAT_EQ (t_LM_1.x (), t_LM_2.x ());
  EXPECT_FLOAT_EQ (t_LM_1.y (), t_LM_2.y ());
  EXPECT_FLOAT_EQ (t_LM_1.z (), t_LM_2.z ());
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` and pass their path to the test." << std::endl;
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

  // Tranpose the cloud_model
  /*for (size_t i = 0; i < cloud_model.points.size (); ++i)
  {
  //  cloud_model.points[i].z += 1;
  }*/
}
/* ]--- */
