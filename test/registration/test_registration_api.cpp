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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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

pcl::PointCloud<pcl::PointXYZ> cloud_source, cloud_target, cloud_reg;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceEstimation)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  // check for correct order and number of matches
  EXPECT_EQ (int (correspondences->size ()), nr_original_correspondences);
  if (int (correspondences->size ()) == nr_original_correspondences)
  {
    for (int i = 0; i < nr_original_correspondences; ++i)
      EXPECT_EQ ((*correspondences)[i].index_query, i);

    // check for correct matches
    for (int i = 0; i < nr_original_correspondences; ++i)
      EXPECT_EQ ((*correspondences)[i].index_match, correspondences_original[i][1]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceEstimationReciprocal)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineReciprocalCorrespondences (*correspondences);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences->size ()), nr_reciprocal_correspondences);
  if (int (correspondences->size ()) == nr_reciprocal_correspondences)
    for (int i = 0; i < nr_reciprocal_correspondences; ++i)
      EXPECT_EQ ((*correspondences)[i].index_match, correspondences_reciprocal[i][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorDistance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
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
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
      EXPECT_EQ ((*correspondences_result_rej_dist)[i].index_match, correspondences_dist[i][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorMedianDistance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_median_dist (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorMedianDistance corr_rej_median_dist;
  corr_rej_median_dist.setInputCorrespondences(correspondences);
  corr_rej_median_dist.setMedianFactor (0.5);

  corr_rej_median_dist.getCorrespondences(*correspondences_result_rej_median_dist);

  // check for correct matches
  if (int (correspondences_result_rej_median_dist->size ()) == nr_correspondences_result_rej_dist)
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
      EXPECT_EQ ((*correspondences_result_rej_median_dist)[i].index_match, correspondences_dist[i][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorOneToOne)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_one_to_one (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
  corr_rej_one_to_one.setInputCorrespondences(correspondences);
  corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences_result_rej_one_to_one->size ()), nr_correspondences_result_rej_one_to_one);
  if (int (correspondences_result_rej_one_to_one->size ()) == nr_correspondences_result_rej_one_to_one)
    for (int i = 0; i < nr_correspondences_result_rej_one_to_one; ++i)
      EXPECT_EQ ((*correspondences_result_rej_one_to_one)[i].index_match, correspondences_one_to_one[i][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorSampleConsensus)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);
  EXPECT_EQ (int (correspondences->size ()), nr_original_correspondences);

  boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> corr_rej_sac;
  corr_rej_sac.setInputCloud (source);
  corr_rej_sac.setTargetCloud (target);
  corr_rej_sac.setInlierThreshold (rej_sac_max_dist);
  corr_rej_sac.setMaxIterations (rej_sac_max_iter);
  corr_rej_sac.setInputCorrespondences (correspondences);
  corr_rej_sac.getCorrespondences (*correspondences_result_rej_sac);
  Eigen::Matrix4f transform_res_from_SAC = corr_rej_sac.getBestTransformation ();

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences_result_rej_sac->size ()), nr_correspondences_result_rej_sac);
  if (int (correspondences_result_rej_sac->size ()) == nr_correspondences_result_rej_sac)
    for (int i = 0; i < nr_correspondences_result_rej_sac; ++i)
      EXPECT_EQ ((*correspondences_result_rej_sac)[i].index_match, correspondences_sac[i][1]);

  // check for correct transformation
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NEAR (transform_res_from_SAC (i, j), transform_from_SAC[i][j], 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorSurfaceNormal)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);


  pcl::PointCloud<pcl::PointNormal>::Ptr source_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*source, *source_normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr target_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*target, *target_normals);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est_src;
  norm_est_src.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est_src.setKSearch (10);
  norm_est_src.setInputCloud (source_normals);
  norm_est_src.compute (*source_normals);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est_tgt;
  norm_est_tgt.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est_tgt.setKSearch (10);
  norm_est_tgt.setInputCloud (target_normals);
  norm_est_tgt.compute (*target_normals);

  pcl::registration::CorrespondenceRejectorSurfaceNormal  corr_rej_surf_norm;
  corr_rej_surf_norm.initializeDataContainer <pcl::PointXYZ, pcl::PointNormal> ();
  corr_rej_surf_norm.setInputCloud <pcl::PointXYZ> (source);
  corr_rej_surf_norm.setInputTarget <pcl::PointXYZ> (target);
  corr_rej_surf_norm.setInputNormals <pcl::PointXYZ, pcl::PointNormal> (source_normals);
  corr_rej_surf_norm.setTargetNormals <pcl::PointXYZ, pcl::PointNormal> (target_normals);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_surf_norm (new pcl::Correspondences);
  corr_rej_surf_norm.setInputCorrespondences (correspondences);
  corr_rej_surf_norm.setThreshold (0.5);

  corr_rej_surf_norm.getCorrespondences (*correspondences_result_rej_surf_norm);

  // check for correct matches
  if (int (correspondences_result_rej_surf_norm->size ()) == nr_correspondences_result_rej_dist)
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
      EXPECT_EQ ((*correspondences_result_rej_surf_norm)[i].index_match, correspondences_dist[i][1]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorTrimmed)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
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
      EXPECT_EQ ((*correspondences_result_rej_trimmed)[i].index_query, correspondences_trimmed[i][0]);
    for (int i = 0; i < nr_correspondences_result_rej_trimmed; ++i)
      EXPECT_EQ ((*correspondences_result_rej_trimmed)[i].index_match, correspondences_trimmed[i][1]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CorrespondenceRejectorVarTrimmed)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineCorrespondences (*correspondences);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_var_trimmed_dist (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorVarTrimmed corr_rej_var_trimmed_dist;
  corr_rej_var_trimmed_dist.setInputCloud<pcl::PointXYZ> (source);
  corr_rej_var_trimmed_dist.setInputTarget<pcl::PointXYZ> (target);
  corr_rej_var_trimmed_dist.setInputCorrespondences(correspondences);

  corr_rej_var_trimmed_dist.getCorrespondences(*correspondences_result_rej_var_trimmed_dist);

  // check for correct matches
  if (int (correspondences_result_rej_var_trimmed_dist->size ()) == nr_correspondences_result_rej_dist)
    for (int i = 0; i < nr_correspondences_result_rej_dist; ++i)
      EXPECT_EQ ((*correspondences_result_rej_var_trimmed_dist)[i].index_match, correspondences_dist[i][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformationEstimationSVD)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do reciprocal correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineReciprocalCorrespondences (*correspondences);

  Eigen::Matrix4f transform_res_from_SVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est_svd;
  trans_est_svd.estimateRigidTransformation(*source, *target,
                                            *correspondences,
                                            transform_res_from_SVD);

  // check for correct transformation
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NEAR (transform_res_from_SVD(i, j), transform_from_SVD[i][j], 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformationEstimationLM)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do reciprocal correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineReciprocalCorrespondences (*correspondences);

  Eigen::Matrix4f transform_res_from_LM;
  pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> trans_est_lm;
  trans_est_lm.estimateRigidTransformation(*source, *target,
                                           *correspondences,
                                           transform_res_from_LM);

  // check for correct matches and number of matches
  EXPECT_EQ (int (correspondences->size ()), nr_reciprocal_correspondences);
  for (int i = 0; i < nr_reciprocal_correspondences; ++i)
  {
    EXPECT_EQ ((*correspondences)[i].index_query, correspondences_reciprocal[i][0]);
    EXPECT_EQ ((*correspondences)[i].index_match, correspondences_reciprocal[i][1]);
  }

//  // check for correct transformation
//  for (int i = 0; i < 4; ++i)
//    for (int j = 0; j < 4; ++j)
//      EXPECT_NEAR (transform_res_from_LM(i, j), transform_from_LM[i][j], 1e-4);
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
