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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id:$
 *
 */

#ifndef PCL_RECOGNITION_HOUGH_3D_IMPL_H_
#define PCL_RECOGNITION_HOUGH_3D_IMPL_H_

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/board.h>


template<typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT>
template<typename PointType, typename PointRfType> void
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::computeRf (const boost::shared_ptr<const pcl::PointCloud<PointType> > &input, pcl::PointCloud<PointRfType> &rf)
{
  if (local_rf_search_radius_ == 0)
  {
    PCL_WARN ("[pcl::Hough3DGrouping::computeRf()] Warning! Reference frame search radius not set. Computing with default value. Results might be incorrect, algorithm might be slow.\n");
    local_rf_search_radius_ = static_cast<float> (hough_bin_size_);
  }
  pcl::PointCloud<Normal>::Ptr normal_cloud (new pcl::PointCloud<Normal> ());
  NormalEstimation<PointType, Normal> norm_est;
  norm_est.setInputCloud (input);
  if (local_rf_normals_search_radius_ <= 0.0f)
  {
    norm_est.setKSearch (15);
  }
  else
  {
    norm_est.setRadiusSearch (local_rf_normals_search_radius_);
  }  
  norm_est.compute (*normal_cloud);

  BOARDLocalReferenceFrameEstimation<PointType, Normal, PointRfType> rf_est;
  rf_est.setInputCloud (input);
  rf_est.setInputNormals (normal_cloud);
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (local_rf_search_radius_);
  rf_est.compute (rf);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> bool
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::train ()
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::Hough3DGrouping::train()] Error! Input cloud not set.\n");
    return (false);
  }

  if (!input_rf_)
  {
    ModelRfCloudPtr new_input_rf (new ModelRfCloud ());
    computeRf (input_, *new_input_rf);
    input_rf_ = new_input_rf;
    //PCL_ERROR(
    //  "[pcl::Hough3DGrouping::train()] Error! Input reference frame not set.\n");
    //return (false);
  }

  if (input_->size () != input_rf_->size ())
  {
    PCL_ERROR ("[pcl::Hough3DGrouping::train()] Error! Input cloud size != Input RF cloud size.\n");
    return (false);
  }

  model_votes_.clear ();
  model_votes_.resize (input_->size ());

  // compute model centroid
  Eigen::Vector3f centroid (0, 0, 0);
  for (size_t i = 0; i < input_->size (); ++i)
  {
    centroid += input_->at (i).getVector3fMap ();
  }
  centroid /= static_cast<float> (input_->size ());

  // compute model votes
  for (size_t i = 0; i < input_->size (); ++i)
  {
    Eigen::Vector3f x_ax ((*input_rf_)[i].x_axis[0], (*input_rf_)[i].x_axis[1], (*input_rf_)[i].x_axis[2]);
    Eigen::Vector3f y_ax ((*input_rf_)[i].y_axis[0], (*input_rf_)[i].y_axis[1], (*input_rf_)[i].y_axis[2]);
    Eigen::Vector3f z_ax ((*input_rf_)[i].z_axis[0], (*input_rf_)[i].z_axis[1], (*input_rf_)[i].z_axis[2]);

    model_votes_[i].x () = x_ax.dot (centroid - input_->at (i).getVector3fMap ());
    model_votes_[i].y () = y_ax.dot (centroid - input_->at (i).getVector3fMap ());
    model_votes_[i].z () = z_ax.dot (centroid - input_->at (i).getVector3fMap ());
  }

  needs_training_ = false;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> bool
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::houghVoting ()
{
  if (needs_training_)
  {
    if (!train ())//checks input and input_rf
      return (false);
  }

  //if (!scene_)
  //{
  //  PCL_ERROR(
  //    "[pcl::Hough3DGrouping::recognizeModelInstances()] Error! Scene cloud not set.\n");
  //  return (false);
  //}

  if (!scene_rf_)
  {
    ModelRfCloudPtr new_scene_rf (new ModelRfCloud ());
    computeRf (scene_, *new_scene_rf);
    scene_rf_ = new_scene_rf;
    //PCL_ERROR(
    //  "[pcl::Hough3DGrouping::recognizeModelInstances()] Error! Scene reference frame not set.\n");
    //return (false);
  }

  if (scene_->size () != scene_rf_->size ())
  {
    PCL_ERROR ("[pcl::Hough3DGrouping::recognizeModelInstances()] Error! Scene cloud size != Scene RF cloud size.\n");
    return (false);
  }

  if (!model_scene_corrs_)
  {
    PCL_ERROR ("[pcl::Hough3DGrouping::recognizeModelInstances()] Error! Correspondences not set, please set them before calling again this function.\n");
    return (false);
  }

  int n_matches = static_cast<int> (model_scene_corrs_->size ());
  if (n_matches == 0)
  {
    return (false);
  }

  std::vector<Eigen::Vector3d> scene_votes (n_matches);
  Eigen::Vector3d d_min, d_max, bin_size;

  d_min.setConstant (std::numeric_limits<double>::max ());
  d_max.setConstant (-std::numeric_limits<double>::max ());
  bin_size.setConstant (hough_bin_size_);

  float max_distance = -std::numeric_limits<float>::max ();

  // Calculating 3D Hough space dimensions and vote position for each match
  for (int i=0; i< n_matches; ++i)
  {
    int scene_index = model_scene_corrs_->at (i).index_match;
    int model_index = model_scene_corrs_->at (i).index_query;

    const Eigen::Vector3f& scene_point = scene_->at (scene_index).getVector3fMap ();
    const PointSceneRfT&   scene_point_rf = scene_rf_->at (scene_index);
    
    Eigen::Vector3f scene_point_rf_x (scene_point_rf.x_axis[0], scene_point_rf.x_axis[1], scene_point_rf.x_axis[2]);
    Eigen::Vector3f scene_point_rf_y (scene_point_rf.y_axis[0], scene_point_rf.y_axis[1], scene_point_rf.y_axis[2]);
    Eigen::Vector3f scene_point_rf_z (scene_point_rf.z_axis[0], scene_point_rf.z_axis[1], scene_point_rf.z_axis[2]);

    //const Eigen::Vector3f& model_point = input_->at (model_index).getVector3fMap ();
    const Eigen::Vector3f& model_point_vote = model_votes_[model_index];

    scene_votes[i].x () = scene_point_rf_x[0] * model_point_vote.x () + scene_point_rf_y[0] * model_point_vote.y () + scene_point_rf_z[0] * model_point_vote.z () + scene_point.x ();
    scene_votes[i].y () = scene_point_rf_x[1] * model_point_vote.x () + scene_point_rf_y[1] * model_point_vote.y () + scene_point_rf_z[1] * model_point_vote.z () + scene_point.y ();
    scene_votes[i].z () = scene_point_rf_x[2] * model_point_vote.x () + scene_point_rf_y[2] * model_point_vote.y () + scene_point_rf_z[2] * model_point_vote.z () + scene_point.z ();

    if (scene_votes[i].x () < d_min.x ()) 
      d_min.x () = scene_votes[i].x (); 
    if (scene_votes[i].x () > d_max.x ()) 
      d_max.x () = scene_votes[i].x (); 

    if (scene_votes[i].y () < d_min.y ()) 
      d_min.y () = scene_votes[i].y (); 
    if (scene_votes[i].y () > d_max.y ()) 
      d_max.y () = scene_votes[i].y (); 

    if (scene_votes[i].z () < d_min.z ()) 
      d_min.z () = scene_votes[i].z (); 
    if (scene_votes[i].z () > d_max.z ()) 
      d_max.z () = scene_votes[i].z ();

    // Calculate max distance for interpolated votes
    if (use_interpolation_ && max_distance < model_scene_corrs_->at (i).distance)
    {
      max_distance = model_scene_corrs_->at (i).distance;
    }
  }

  // Hough Voting
  hough_space_.reset (new pcl::recognition::HoughSpace3D (d_min, bin_size, d_max));

  for (int i = 0; i < n_matches; ++i)
  {
    double weight = 1.0;
    if (use_distance_weight_ && max_distance != 0)
    {
      weight = 1.0 - (model_scene_corrs_->at (i).distance / max_distance);
    }
    if (use_interpolation_)
    {
      hough_space_->voteInt (scene_votes[i], weight, i);
    } 
    else
    {
      hough_space_->vote (scene_votes[i], weight, i);
    }
  }

  hough_space_initialized_ = true;

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> void
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::clusterCorrespondences (std::vector<Correspondences> &model_instances)
{
  model_instances.clear ();
  found_transformations_.clear ();

  if (!hough_space_initialized_ && !houghVoting ())
  {
    return;
  }

  // Finding max bins and voters
  std::vector<double> max_values;
  std::vector<std::vector<int> > max_ids;

  hough_space_->findMaxima (hough_threshold_, max_values, max_ids);

  // Insert maximas into result vector, after Ransac correspondence rejection
  // Temp copy of scene cloud with the type cast to ModelT in order to use Ransac
  PointCloudPtr temp_scene_cloud_ptr (new PointCloud);
  pcl::copyPointCloud<PointSceneT, PointModelT> (*scene_, *temp_scene_cloud_ptr);

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointModelT> corr_rejector;
  corr_rejector.setMaximumIterations (10000);
  corr_rejector.setInlierThreshold (hough_bin_size_);
  corr_rejector.setInputSource (input_);
  corr_rejector.setInputTarget (temp_scene_cloud_ptr);

  for (size_t j = 0; j < max_values.size (); ++j)
  {
    Correspondences temp_corrs, filtered_corrs;
    for (size_t i = 0; i < max_ids[j].size (); ++i)
    {
      temp_corrs.push_back (model_scene_corrs_->at (max_ids[j][i]));
    }
    // RANSAC filtering
    corr_rejector.getRemainingCorrespondences (temp_corrs, filtered_corrs);
    // Save transformations for recognize
    found_transformations_.push_back (corr_rejector.getBestTransformation ());

    model_instances.push_back (filtered_corrs);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template<typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> bool
//pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::getTransformMatrix (const PointCloudConstPtr &scene_cloud, const Correspondences &corrs, Eigen::Matrix4f &transform)
//{
//  std::vector<int> model_indices;
//  std::vector<int> scene_indices;
//  pcl::registration::getQueryIndices (corrs, model_indices);
//  pcl::registration::getMatchIndices (corrs, scene_indices);
//
//  typename pcl::SampleConsensusModelRegistration<PointModelT>::Ptr model (new pcl::SampleConsensusModelRegistration<PointModelT> (input_, model_indices));
//  model->setInputTarget (scene_cloud, scene_indices);
//
//  pcl::RandomSampleConsensus<PointModelT> ransac (model);
//  ransac.setDistanceThreshold (hough_bin_size_);
//  ransac.setMaxIterations (10000);
//  if (!ransac.computeModel ())
//    return (false);
//
//  // Transform model coefficients from vectorXf to matrix4f
//  Eigen::VectorXf coeffs;
//  ransac.getModelCoefficients (coeffs);
//
//  transform.row (0) = coeffs.segment<4> (0);
//  transform.row (1) = coeffs.segment<4> (4);
//  transform.row (2) = coeffs.segment<4> (8);
//  transform.row (3) = coeffs.segment<4> (12);
//
//  return (true);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> bool
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::recognize (
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations)
{
  std::vector<pcl::Correspondences> model_instances;
  return (this->recognize (transformations, model_instances));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointModelT, typename PointSceneT, typename PointModelRfT, typename PointSceneRfT> bool
pcl::Hough3DGrouping<PointModelT, PointSceneT, PointModelRfT, PointSceneRfT>::recognize (
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, std::vector<pcl::Correspondences> &clustered_corrs)
{
  transformations.clear ();
  if (!this->initCompute ())
  {
    PCL_ERROR ("[pcl::Hough3DGrouping::recognize()] Error! Model cloud or Scene cloud not set, please set them before calling again this function.\n");
    return (false);
  }

  clusterCorrespondences (clustered_corrs);

  transformations = found_transformations_;

  //// Temp copy of scene cloud with the type cast to ModelT in order to use Ransac
  //PointCloudPtr temp_scene_cloud_ptr (new PointCloud);
  //pcl::copyPointCloud<PointSceneT, PointModelT> (*scene_, *temp_scene_cloud_ptr);

  //for (size_t i = 0; i < model_instances.size (); ++i)
  //{
  //  Eigen::Matrix4f curr_transf;
  //  if (getTransformMatrix (temp_scene_cloud_ptr, model_instances[i], curr_transf))
  //    transformations.push_back (curr_transf);
  //}

  this->deinitCompute ();
  return (true);
}


#define PCL_INSTANTIATE_Hough3DGrouping(T,ST,RFT,SRFT) template class PCL_EXPORTS pcl::Hough3DGrouping<T,ST,RFT,SRFT>;

#endif // PCL_RECOGNITION_HOUGH_3D_IMPL_H_
