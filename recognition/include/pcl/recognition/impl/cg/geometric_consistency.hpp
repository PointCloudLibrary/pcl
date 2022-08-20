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
 * $Id$
 *
 */

#ifndef PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_IMPL_H_
#define PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_IMPL_H_

#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
gcCorrespSorter (pcl::Correspondence i, pcl::Correspondence j)
{
  return (i.distance < j.distance);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT> void
pcl::GeometricConsistencyGrouping<PointModelT, PointSceneT>::clusterCorrespondences (std::vector<Correspondences> &model_instances)
{
  model_instances.clear ();
  found_transformations_.clear ();

  if (!model_scene_corrs_)
  {
    PCL_ERROR(
      "[pcl::GeometricConsistencyGrouping::clusterCorrespondences()] Error! Correspondences not set, please set them before calling again this function.\n");
    return;
  }

  CorrespondencesPtr sorted_corrs (new Correspondences (*model_scene_corrs_));

  std::sort (sorted_corrs->begin (), sorted_corrs->end (), gcCorrespSorter);

  model_scene_corrs_ = sorted_corrs;
  PCL_DEBUG_STREAM("[pcl::GeometricConsistencyGrouping::clusterCorrespondences] Five best correspondences: ");
  for(std::size_t i=0; i<std::min<std::size_t>(model_scene_corrs_->size(), 5); ++i)
    PCL_DEBUG_STREAM("[" << (*input_)[(*model_scene_corrs_)[i].index_query] << " " << (*scene_)[(*model_scene_corrs_)[i].index_match] << " " << (*model_scene_corrs_)[i].distance << "] ");
  PCL_DEBUG_STREAM(std::endl);

  std::vector<int> consensus_set;
  std::vector<bool> taken_corresps (model_scene_corrs_->size (), false);

  Eigen::Vector3f dist_ref, dist_trg;

  //temp copy of scene cloud with the type cast to ModelT in order to use Ransac
  PointCloudPtr temp_scene_cloud_ptr (new PointCloud ());
  pcl::copyPointCloud (*scene_, *temp_scene_cloud_ptr);

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointModelT> corr_rejector;
  corr_rejector.setMaximumIterations (10000);
  corr_rejector.setInlierThreshold (gc_size_);
  corr_rejector.setInputSource(input_);
  corr_rejector.setInputTarget (temp_scene_cloud_ptr);

  for (std::size_t i = 0; i < model_scene_corrs_->size (); ++i)
  {
    if (taken_corresps[i])
      continue;

    consensus_set.clear ();
    consensus_set.push_back (static_cast<int> (i));
    
    for (std::size_t j = 0; j < model_scene_corrs_->size (); ++j)
    {
      if ( j != i &&  !taken_corresps[j])
      {
        //Let's check if j fits into the current consensus set
        bool is_a_good_candidate = true;
        for (const int &k : consensus_set)
        {
          int scene_index_k = model_scene_corrs_->at (k).index_match;
          int model_index_k = model_scene_corrs_->at (k).index_query;
          int scene_index_j = model_scene_corrs_->at (j).index_match;
          int model_index_j = model_scene_corrs_->at (j).index_query;
          
          const Eigen::Vector3f& scene_point_k = scene_->at (scene_index_k).getVector3fMap ();
          const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();
          const Eigen::Vector3f& scene_point_j = scene_->at (scene_index_j).getVector3fMap ();
          const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

          dist_ref = scene_point_k - scene_point_j;
          dist_trg = model_point_k - model_point_j;

          double distance = std::abs (dist_ref.norm () - dist_trg.norm ());

          if (distance > gc_size_)
          {
            is_a_good_candidate = false;
            break;
          }
        }

        if (is_a_good_candidate)
          consensus_set.push_back (static_cast<int> (j));
      }
    }
    
    if (static_cast<int> (consensus_set.size ()) > gc_threshold_)
    {
      Correspondences temp_corrs, filtered_corrs;
      for (const int &j : consensus_set)
      {
        temp_corrs.push_back (model_scene_corrs_->at (j));
        taken_corresps[ j ] = true;
      }
      //ransac filtering
      corr_rejector.getRemainingCorrespondences (temp_corrs, filtered_corrs);
      //save transformations for recognize
      found_transformations_.push_back (corr_rejector.getBestTransformation ());

      model_instances.push_back (filtered_corrs);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT> bool
pcl::GeometricConsistencyGrouping<PointModelT, PointSceneT>::recognize (
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations)
{
  std::vector<pcl::Correspondences> model_instances;
  return (this->recognize (transformations, model_instances));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT> bool
pcl::GeometricConsistencyGrouping<PointModelT, PointSceneT>::recognize (
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, std::vector<pcl::Correspondences> &clustered_corrs)
{
  transformations.clear ();
  if (!this->initCompute ())
  {
    PCL_ERROR(
      "[pcl::GeometricConsistencyGrouping::recognize()] Error! Model cloud or Scene cloud not set, please set them before calling again this function.\n");
    return (false);
  }

  clusterCorrespondences (clustered_corrs);

  transformations = found_transformations_;

  this->deinitCompute ();
  return (true);
}

#define PCL_INSTANTIATE_GeometricConsistencyGrouping(T,ST) template class PCL_EXPORTS pcl::GeometricConsistencyGrouping<T,ST>;

#endif // PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_IMPL_H_
