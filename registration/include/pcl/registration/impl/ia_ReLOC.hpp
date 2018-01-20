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
*/

#ifndef PCL_REGISTRATION_IMPL_IA_RELOC_H_
#define PCL_REGISTRATION_IMPL_IA_RELOC_H_

#include <pcl/registration/ia_ReLOC.h>

#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/flat_keypoint.h>

#include <pcl/common/geometry.h>
#include <pcl/features/flare.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

 ///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::ReLOCInitialAlignment () :
  source_keypoints_ (new pcl::PointCloud<PointSource> ()),
  target_keypoints_ (new pcl::PointCloud<PointTarget> ()),
  source_lrf_cloud_ (new ReferenceFrames ()),
  target_lrf_cloud_ (new ReferenceFrames ()),
  source_flare_signed_distances_ (),
  target_flare_signed_distances_ (),
  source_normals_ (),
  target_normals_ (),
  seed_ (static_cast<unsigned int> (time (NULL))),
  use_random_detector_ (false),
  flat_keypoint_min_neighbors_ (5),
  flat_keypoint_Rf_ (0.0),
  flat_keypoint_R_discard_ (0.0),
  flat_keypoint_R1_search_ (0.0),
  flat_keypoint_R2_search_ (0.0),
  flat_keypoint_T1_search_ (0.9),
  flat_keypoint_T2_search_ (0.9),
  n_random_keypoints_ (2000),
  flare_normal_radius_ (0.0f),
  flare_tangent_radius_ (0.0f),
  flare_margin_thresh_ (0.85f),
  flare_min_neighbors_for_normal_axis_ (6),
  flare_min_neighbors_for_tangent_axis_ (6),
  flare_x_support_sampling_perc_ (1.0f),
  matcher_TD_ (0.01f),
  hough_f_ (2.8), //1.4 in the paper
  hough_Sbin_ (0.0),
  ransac_T_ (0.0),
  ransac_N_ (1000)
{
  reg_name_ = "pcl::registration::ReLOCInitialAlignment";
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
template <typename PointT> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::detectKeypoints (
  typename pcl::PointCloud<PointT>::ConstPtr cloud,
  NormalsConstPtr normals,
  typename pcl::PointCloud<PointT>::Ptr &keypoints)
{
  if (use_random_detector_)
  {
    //Random detector
    pcl::RandomSample<PointT> random_sampling;
    random_sampling.setSeed (seed_);
    random_sampling.setSample (n_random_keypoints_);
    random_sampling.setInputCloud (cloud);
    random_sampling.filter (*keypoints);
  }
  else
  {
    //FlatKeypoint detector
    pcl::FlatKeypoint<PointT, PointT, NormalT> flat_keypoint;
    flat_keypoint.setSeed (seed_);
    flat_keypoint.setRadiusSearch (flat_keypoint_Rf_);
    flat_keypoint.setRdiscard (flat_keypoint_R_discard_);
    flat_keypoint.setR1search (flat_keypoint_R1_search_);
    flat_keypoint.setR2search (flat_keypoint_R2_search_);
    flat_keypoint.setT1search (flat_keypoint_T1_search_);
    flat_keypoint.setT2search (flat_keypoint_T2_search_);
    flat_keypoint.setMinNeighbors (flat_keypoint_min_neighbors_);
    flat_keypoint.setInputCloud (cloud);
    flat_keypoint.setNormals (normals);
    flat_keypoint.compute (*keypoints);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
template <typename PointT> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::computeFlares (
  typename pcl::PointCloud<PointT>::ConstPtr cloud,
  NormalsConstPtr normals,
  typename pcl::PointCloud<PointT>::ConstPtr keypoints,
  ReferenceFramesPtr &flares,
  std::vector<float> &flare_signed_distances)
{
  FLARELocalReferenceFrameEstimation<PointT, NormalT, ReferenceFrame> flare_descriptor;
  flare_descriptor.setRadiusSearch (flare_normal_radius_);
  flare_descriptor.setTangentRadius (flare_tangent_radius_);
  flare_descriptor.setMarginThresh (flare_margin_thresh_);
  flare_descriptor.setMinNeighboursForNormalAxis (flare_min_neighbors_for_normal_axis_);
  flare_descriptor.setMinNeighboursForTangentAxis (flare_min_neighbors_for_tangent_axis_);
  flare_descriptor.setInputCloud (keypoints);
  flare_descriptor.setSearchSurface (cloud);
  flare_descriptor.setInputNormals (normals);

  if (flare_x_support_sampling_perc_ != 1.0)
  {
    pcl::PointCloud<PointT>::Ptr sampled_cloud (new pcl::PointCloud<PointT>);
    std::vector<int> indices_sampled;
    const float samplingIncr = 1.0f / flare_x_support_sampling_perc_;
    for (float sa = 0.0f; sa < (float)cloud->points.size (); sa += samplingIncr)
      indices_sampled.push_back (static_cast<int> (sa));
    copyPointCloud (*cloud, indices_sampled, *sampled_cloud);
    flare_descriptor.setSearchSampledSurface (sampled_cloud);

    pcl::search::KdTree<PointT>::Ptr sampled_tree (new pcl::search::KdTree<PointT>);
    sampled_tree->setInputCloud (sampled_cloud);
    flare_descriptor.setSearchMethodForSampledSurface (sampled_tree);
  }

  flare_descriptor.compute (*flares);
  flare_signed_distances = flare_descriptor.getSignedDistancesFromHighestPoints ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::match (
  const std::vector<float> &target_flare_signed_distances,
  const std::vector<float> &source_flare_signed_distances,
  pcl::CorrespondencesPtr &correspondences)
{
  //The matching scheme proposed in the paper can be replaced by a simple radius search in the 1-dimensional space of the signed distances
  pcl::PointCloud<float>::Ptr target_distances_cloud (new pcl::PointCloud<float> ());
  pcl::PointCloud<float>::Ptr source_distances_cloud (new pcl::PointCloud<float> ());
  target_distances_cloud->points.insert (target_distances_cloud->points.end (), target_flare_signed_distances.begin (), target_flare_signed_distances.end ());
  source_distances_cloud->points.insert (source_distances_cloud->points.end (), source_flare_signed_distances.begin (), source_flare_signed_distances.end ());

  //find max difference between source and target distances
  float trg_max_dist = -std::numeric_limits<float>::max ();
  float trg_min_dist = std::numeric_limits<float>::max ();
  for (size_t ta = 0; ta < target_distances_cloud->points.size (); ta++)
  {
    if (target_distances_cloud->points[ta] == std::numeric_limits<float>::max ())
    {
      target_distances_cloud->points[ta] = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }
    trg_max_dist = std::max (target_distances_cloud->points[ta], trg_max_dist);
    trg_min_dist = std::min (target_distances_cloud->points[ta], trg_min_dist);
  }
  if (trg_max_dist == -std::numeric_limits<float>::max ())  //check if there are, at least, a good signed distance
  {
    return;
  }

  float src_max_dist = -std::numeric_limits<float>::max ();
  float src_min_dist = std::numeric_limits<float>::max ();
  for (size_t so = 0; so < source_distances_cloud->points.size (); so++)
  {
    if (source_distances_cloud->points[so] == std::numeric_limits<float>::max ())
    {
      source_distances_cloud->points[so] = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }
    src_max_dist = std::max (source_distances_cloud->points[so], src_max_dist);
    src_min_dist = std::min (source_distances_cloud->points[so], src_min_dist);
  }
  if (src_max_dist == -std::numeric_limits<float>::max ())  //check if there are, at least, a good signed distance
  {
    return;
  }

  float src_trg_max_diff = std::max (fabs (trg_max_dist - src_min_dist), fabs (src_max_dist - trg_min_dist));

  float dist_radius = src_trg_max_diff * matcher_TD_; //search radius

  //perform matching process
  correspondences->clear ();
  pcl::search::KdTree<float> match_search;
  match_search.setInputCloud (source_distances_cloud);

  for (size_t ta = 0; ta < target_flare_signed_distances_.size (); ++ta)
  {
    if (!pcl_isfinite (target_distances_cloud->points[ta]))
    {
      continue;
    }

    std::vector<int> neigh_indices;
    std::vector<float> neigh_sqr_dists;
    int found_neighs = match_search.radiusSearch (target_distances_cloud->points[ta], dist_radius, neigh_indices, neigh_sqr_dists);

    for (size_t co = 0; co < found_neighs; co++)
    {
      pcl::Correspondence corr (neigh_indices[co], static_cast<int> (ta), neigh_sqr_dists[co]);
      correspondences->push_back (corr);
    }
  }
}

template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::houghVoting (
  pcl::CorrespondencesConstPtr matcher_correspondences,
  pcl::CorrespondencesPtr &hough_correspondences)
{
  PointCloudSourceConstPtr &source_ = input_;

  if (source_cloud_updated_)
  {
    // compute source centroid
    Eigen::Vector4f source_centroid_4f;
    pcl::compute3DCentroid<PointSource> (*source_, source_centroid_4f);
    Eigen::Vector3f source_centroid = source_centroid_4f.head<3> ();

    // compute source votes
    source_votes_.resize (source_keypoints_->size ());
    for (size_t i = 0; i < source_keypoints_->size (); ++i)
    {
      Eigen::Vector3f x_ax ((*source_lrf_cloud_)[i].x_axis[0], (*source_lrf_cloud_)[i].x_axis[1], (*source_lrf_cloud_)[i].x_axis[2]);
      Eigen::Vector3f y_ax ((*source_lrf_cloud_)[i].y_axis[0], (*source_lrf_cloud_)[i].y_axis[1], (*source_lrf_cloud_)[i].y_axis[2]);
      Eigen::Vector3f z_ax ((*source_lrf_cloud_)[i].z_axis[0], (*source_lrf_cloud_)[i].z_axis[1], (*source_lrf_cloud_)[i].z_axis[2]);

      source_votes_[i].x () = x_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
      source_votes_[i].y () = y_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
      source_votes_[i].z () = z_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
    }
  }

  if (target_cloud_updated_)
  {
    //allocate hough space
    Eigen::Vector3d d_min, d_max, bin_size;

    bin_size.setConstant (hough_Sbin_);

    //determine extension of hough space
    std::vector<double> bbox = pcl::computeBBoxFromPointDistribution<PointTarget> (*target_);
    pcl::geometry::extendBBox (&bbox[0], hough_f_, hough_f_, hough_f_);
    d_min.x () = bbox[0];
    d_min.y () = bbox[2];
    d_min.z () = bbox[4];

    d_max.x () = bbox[1];
    d_max.y () = bbox[3];
    d_max.z () = bbox[5];

    hough_space_.reset (new pcl::HoughSpace3D<unsigned short int> (d_min, bin_size, d_max));
  }
  else
  {
    hough_space_->reset ();
  }

  //cast votes
  for (size_t ma = 0; ma < matcher_correspondences->size (); ++ma)
  {
    Eigen::Vector3d target_vote;

    int target_index = matcher_correspondences->at (ma).index_match;
    int source_index = matcher_correspondences->at (ma).index_query;

    const Eigen::Vector3f& source_point_vote = source_votes_[source_index];

    const Eigen::Vector3f& target_point = target_keypoints_->at (target_index).getVector3fMap ();
    const ReferenceFrame&   target_point_rf = target_lrf_cloud_->at (target_index);

    Eigen::Vector3f target_point_rf_x (target_point_rf.x_axis[0], target_point_rf.x_axis[1], target_point_rf.x_axis[2]);
    Eigen::Vector3f target_point_rf_y (target_point_rf.y_axis[0], target_point_rf.y_axis[1], target_point_rf.y_axis[2]);
    Eigen::Vector3f target_point_rf_z (target_point_rf.z_axis[0], target_point_rf.z_axis[1], target_point_rf.z_axis[2]);

    target_vote.x () = target_point_rf_x[0] * source_point_vote.x () + target_point_rf_y[0] * source_point_vote.y () + target_point_rf_z[0] * source_point_vote.z () + target_point.x ();
    target_vote.y () = target_point_rf_x[1] * source_point_vote.x () + target_point_rf_y[1] * source_point_vote.y () + target_point_rf_z[1] * source_point_vote.z () + target_point.y ();
    target_vote.z () = target_point_rf_x[2] * source_point_vote.x () + target_point_rf_y[2] * source_point_vote.y () + target_point_rf_z[2] * source_point_vote.z () + target_point.z ();

    hough_space_->vote (target_vote, 1, (int)ma);
  }

  // Finding max bins and voters
  std::vector<int> max_ids;
  hough_space_->findMaximum (max_ids);

  hough_correspondences.reset (new pcl::Correspondences (max_ids.size ()));

  for (size_t co = 0; co < max_ids.size (); ++co)
  {
    hough_correspondences->at (co) = matcher_correspondences->at (max_ids[co]);
  }
}

template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::estimateRigidMotion (
  pcl::CorrespondencesConstPtr hough_correspondences,
  Eigen::Matrix4f &final_transformation,
  pcl::CorrespondencesPtr &ransac_correspondences)
{
  CorrespondenceRejectorSampleConsensus<PointSource> ransac;
  ransac.setInputSource (source_keypoints_);
  ransac.setInputTarget (target_keypoints_);
  ransac.setInlierThreshold (ransac_T_);
  ransac.setMaximumIterations (ransac_N_);
  ransac.setRefineModel (false);
  ransac.setInputCorrespondences (hough_correspondences);
  ransac.getCorrespondences (*ransac_correspondences);
  final_transformation_ = ransac.getBestTransformation ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::computeTransformation (
  PointCloudSource &output,
  const Eigen::Matrix4f &guess)
{
  final_transformation_ = guess;

  PointCloudSourceConstPtr &source_ = input_;

  //Detection
  if (target_cloud_updated_)
  {
    PCL_INFO ("Extract target keypoints...");
    detectKeypoints<PointTarget> (target_, target_normals_, target_keypoints_);
  }
  PCL_INFO ("Target total points: %d; Selected Keypoints: %d\n", target_->size (), target_keypoints_->size ());

  if (source_cloud_updated_)
  {
    PCL_INFO ("Extract source keypoints...");
    detectKeypoints<PointSource> (source_, source_normals_, source_keypoints_);
  }
  PCL_INFO ("Target total points: %d; Selected Keypoints: %d\n", source_->size (), source_keypoints_->size ());

  //FLARE local reference frames computation
  if (target_cloud_updated_)
  {
    PCL_INFO ("Compute target FLAREs...");
    computeFlares<PointTarget> (target_, target_normals_, target_keypoints_, target_lrf_cloud_, target_flare_signed_distances_);
  }
  PCL_INFO ("target FLAREs computed\n");

  if (source_cloud_updated_)
  {
    PCL_INFO ("Compute source FLAREs...");
    computeFlares<PointSource> (source_, source_normals_, source_keypoints_, source_lrf_cloud_, source_flare_signed_distances_);
  }
  PCL_INFO ("source FLAREs computed\n");

  //Matching
  PCL_INFO ("Perform matching...");
  pcl::CorrespondencesPtr matcher_correspondences (new pcl::Correspondences ());
  match (target_flare_signed_distances_, source_flare_signed_distances_, matcher_correspondences);
  if (matcher_correspondences->size () == 0)
  {
    PCL_INFO ("No matches\n");
    return;
  }
  PCL_INFO ("Matcher correspondences: %d\n", matcher_correspondences->size ());

  //Hough voting
  PCL_INFO ("Perform hough voting...");
  pcl::CorrespondencesPtr hough_correspondences;
  houghVoting (matcher_correspondences, hough_correspondences);
  if (hough_correspondences->size () == 0)
  {
    PCL_INFO ("No matches after Hough voting\n");
    return;
  }
  PCL_INFO ("Hough correspondences: %d\n", hough_correspondences->size ());

  //Ransac
  PCL_INFO ("Estimate rigid motion through RANSAC + Horn...");
  pcl::CorrespondencesPtr ransac_correspondences (new pcl::Correspondences ());
  estimateRigidMotion (hough_correspondences, final_transformation_, ransac_correspondences);
  if (ransac_correspondences->size () < 3)
  {
    PCL_INFO ("RANSAC failed: No consensus set\n");
    return;
  }
  PCL_INFO ("Ransac correspondences: %d\n", ransac_correspondences->size ());

  //apply the final transformation
  pcl::transformPointCloud (*source_, output, final_transformation_);

  deinitCompute ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> bool
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::initCompute ()
{
  // basic pcl initialization
  if (!pcl::PCLBase <PointSource>::initCompute ())
    return (false);

  // check if source and target are given
  if (!input_ || !target_)
  {
    PCL_ERROR ("[%s::initCompute] Source or target dataset not given!\n", reg_name_.c_str ());
    return (false);
  }

  if (source_normals_->empty ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] source_normals_ not set!\n", reg_name_.c_str ());
    return (false);
  }

  if (source_normals_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] source_normals_ given, but the number of normals does not match the number of source points!\n", reg_name_.c_str ());
    return (false);
  }

  if (target_normals_->empty ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] target_normals_ not set!\n", reg_name_.c_str ());
    return (false);
  }

  if (target_normals_->size () != target_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] target_normals_ given, but the number of normals does not match the number of target points!\n", reg_name_.c_str ());
    return (false);
  }

  if (flat_keypoint_Rf_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_Rf_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (flat_keypoint_R_discard_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R_discard_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (flat_keypoint_R1_search_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R1_search_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (flat_keypoint_R2_search_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R2_search_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (flare_normal_radius_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flare_normal_radius_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (flare_tangent_radius_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flare_tangent_radius_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (hough_Sbin_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] hough_Sbin_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if (ransac_T_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] ransac_T_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> bool
pcl::registration::ReLOCInitialAlignment<PointSource, PointTarget, NormalT, Scalar>::deinitCompute ()
{
  source_cloud_updated_ = false;
  target_cloud_updated_ = false;

  return (Registration<PointSource, PointTarget, Scalar>::deinitCompute ());
}

/////////////////////////////////////////////////////////////////////////////////////////

#define PCL_INSTANTIATE_ReLOCInitialAlignment(T,U,N,S) template class PCL_EXPORTS pcl::registration::ReLOCInitialAlignment<T,U,N,S>;

#endif // PCL_REGISTRATION_IMPL_IA_RELOC_H_