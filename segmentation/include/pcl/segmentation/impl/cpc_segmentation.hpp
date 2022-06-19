/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_SEGMENTATION_IMPL_CPC_SEGMENTATION_HPP_
#define PCL_SEGMENTATION_IMPL_CPC_SEGMENTATION_HPP_

#include <pcl/sample_consensus/sac_model_plane.h> // for SampleConsensusModelPlane
#include <pcl/segmentation/cpc_segmentation.h>

template <typename PointT>
pcl::CPCSegmentation<PointT>::CPCSegmentation () :
    max_cuts_ (20),
    min_segment_size_for_cutting_ (400),
    min_cut_score_ (0.16),
    use_local_constrains_ (true),
    use_directed_weights_ (true),
    ransac_itrs_ (10000)
{
}

template <typename PointT>
pcl::CPCSegmentation<PointT>::~CPCSegmentation () = default;

template <typename PointT> void
pcl::CPCSegmentation<PointT>::segment ()
{
  if (supervoxels_set_)
  {
  // Calculate for every Edge if the connection is convex or invalid
  // This effectively performs the segmentation.
    calculateConvexConnections (sv_adjacency_list_);

    // Correct edge relations using extended convexity definition if k>0
    applyKconvexity (k_factor_);

    // Determine whether to use cutting planes
    doGrouping ();

    grouping_data_valid_ = true;

    applyCuttingPlane (max_cuts_);
    
    // merge small segments
    mergeSmallSegments ();
  }
  else
    PCL_WARN ("[pcl::CPCSegmentation::segment] WARNING: Call function setInputSupervoxels first. Nothing has been done. \n");
}

template <typename PointT> void
pcl::CPCSegmentation<PointT>::applyCuttingPlane (std::uint32_t depth_levels_left)
{
  using SegLabel2ClusterMap = std::map<std::uint32_t, pcl::PointCloud<WeightSACPointType>::Ptr>;
  
  pcl::console::print_info ("Cutting at level %d (maximum %d)\n", max_cuts_ - depth_levels_left + 1, max_cuts_);
  // stop if we reached the 0 level
  if (depth_levels_left <= 0)
    return;

  pcl::IndicesPtr support_indices (new pcl::Indices);
  SegLabel2ClusterMap seg_to_edge_points_map;
  std::map<std::uint32_t, std::vector<EdgeID> > seg_to_edgeIDs_map;
  EdgeIterator edge_itr, edge_itr_end, next_edge;
  boost::tie (edge_itr, edge_itr_end) = boost::edges (sv_adjacency_list_);
  for (next_edge = edge_itr; edge_itr != edge_itr_end; edge_itr = next_edge)
  {
    next_edge++;  // next_edge iterator is necessary, because removing an edge invalidates the iterator to the current edge
    std::uint32_t source_sv_label = sv_adjacency_list_[boost::source (*edge_itr, sv_adjacency_list_)];
    std::uint32_t target_sv_label = sv_adjacency_list_[boost::target (*edge_itr, sv_adjacency_list_)];

    std::uint32_t source_segment_label = sv_label_to_seg_label_map_[source_sv_label];
    std::uint32_t target_segment_label = sv_label_to_seg_label_map_[target_sv_label];

    // do not process edges which already split two segments
    if (source_segment_label != target_segment_label)
      continue;

    // if edge has been used for cutting already do not use it again
    if (sv_adjacency_list_[*edge_itr].used_for_cutting)
      continue;
    // get centroids of vertices
    const pcl::PointXYZRGBA source_centroid = sv_label_to_supervoxel_map_[source_sv_label]->centroid_;
    const pcl::PointXYZRGBA target_centroid = sv_label_to_supervoxel_map_[target_sv_label]->centroid_;

    // stores the information about the edge cloud (used for the weighted ransac)
    // we use the normal to express the direction of the connection
    // we use the intensity to express the normal differences between supervoxel patches. <=0: Convex, >0: Concave
    WeightSACPointType edge_centroid;
    edge_centroid.getVector3fMap () = (source_centroid.getVector3fMap () + target_centroid.getVector3fMap ()) / 2;

    // we use the normal to express the direction of the connection!
    edge_centroid.getNormalVector3fMap () = (target_centroid.getVector3fMap () - source_centroid.getVector3fMap ()).normalized ();

    // we use the intensity to express the normal differences between supervoxel patches. <=0: Convex, >0: Concave
    edge_centroid.intensity = sv_adjacency_list_[*edge_itr].is_convex ? -sv_adjacency_list_[*edge_itr].normal_difference : sv_adjacency_list_[*edge_itr].normal_difference;
    if (seg_to_edge_points_map.find (source_segment_label) == seg_to_edge_points_map.end ())
    {
      seg_to_edge_points_map[source_segment_label] = pcl::PointCloud<WeightSACPointType>::Ptr (new pcl::PointCloud<WeightSACPointType> ());
    }
    seg_to_edge_points_map[source_segment_label]->push_back (edge_centroid);
    seg_to_edgeIDs_map[source_segment_label].push_back (*edge_itr);
  }
  bool cut_found = false;
  // do the following processing for each segment separately
  for (const auto &seg_to_edge_points : seg_to_edge_points_map)
  {
    // if too small do not process
    if (seg_to_edge_points.second->size () < min_segment_size_for_cutting_)
    {
      continue;
    }

    std::vector<double> weights;
    weights.resize (seg_to_edge_points.second->size ());
    for (std::size_t cp = 0; cp < seg_to_edge_points.second->size (); ++cp)
    {
      float& cur_weight = (*seg_to_edge_points.second)[cp].intensity;
      cur_weight = cur_weight < concavity_tolerance_threshold_ ? 0 : 1;
      weights[cp] = cur_weight;
    }

    pcl::PointCloud<WeightSACPointType>::Ptr edge_cloud_cluster  = seg_to_edge_points.second;
    pcl::SampleConsensusModelPlane<WeightSACPointType>::Ptr model_p (new pcl::SampleConsensusModelPlane<WeightSACPointType> (edge_cloud_cluster));

    WeightedRandomSampleConsensus weight_sac (model_p, seed_resolution_, true);

    weight_sac.setWeights (weights, use_directed_weights_);
    weight_sac.setMaxIterations (ransac_itrs_);

    // if not enough inliers are found
    if (!weight_sac.computeModel ())
    {
      continue;
    }

    Eigen::VectorXf model_coefficients;
    weight_sac.getModelCoefficients (model_coefficients);

    model_coefficients[3] += std::numeric_limits<float>::epsilon ();    

    weight_sac.getInliers (*support_indices);

    // the support_indices which are actually cut (if not locally constrain:  cut_support_indices = support_indices
    pcl::Indices cut_support_indices;

    if (use_local_constrains_)
    {
      Eigen::Vector3f plane_normal (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
      // Cut the connections.
      // We only iterate through the points which are within the support (when we are local, otherwise all points in the segment).
      // We also just actually cut when the edge goes through the plane. This is why we check the planedistance
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<WeightSACPointType> euclidean_clusterer;
      pcl::search::KdTree<WeightSACPointType>::Ptr tree (new pcl::search::KdTree<WeightSACPointType>);
      tree->setInputCloud (edge_cloud_cluster);
      euclidean_clusterer.setClusterTolerance (seed_resolution_);
      euclidean_clusterer.setMinClusterSize (1);
      euclidean_clusterer.setMaxClusterSize (25000);
      euclidean_clusterer.setSearchMethod (tree);
      euclidean_clusterer.setInputCloud (edge_cloud_cluster);
      euclidean_clusterer.setIndices (support_indices);
      euclidean_clusterer.extract (cluster_indices);
//       sv_adjacency_list_[seg_to_edgeID_map[seg_to_edge_points.first][point_index]].used_for_cutting = true;

      for (const auto &cluster_index : cluster_indices)
      {
        // get centroids of vertices        
        int cluster_concave_pts = 0;
        float cluster_score = 0;
//         std::cout << "Cluster has " << cluster_indices[cc].indices.size () << " points" << std::endl;
        for (const auto &current_index : cluster_index.indices)
        {
          double index_score = weights[current_index];
          if (use_directed_weights_)
            index_score *= 1.414 * (std::abs (plane_normal.dot (edge_cloud_cluster->at (current_index).getNormalVector3fMap ())));
          cluster_score += index_score;
          if (weights[current_index] > 0)
            ++cluster_concave_pts;
        }
        // check if the score is below the threshold. If that is the case this segment should not be split
        cluster_score /= cluster_index.indices.size ();
//         std::cout << "Cluster score: " << cluster_score << std::endl;
        if (cluster_score >= min_cut_score_)
        {
          cut_support_indices.insert (cut_support_indices.end (), cluster_index.indices.begin (), cluster_index.indices.end ());
        }
      }
      if (cut_support_indices.empty ())
      {
//         std::cout << "Could not find planes which exceed required minimum score (threshold " << min_cut_score_ << "), not cutting" << std::endl;
        continue;
      }
    }
    else
    {
      double current_score = weight_sac.getBestScore ();
      cut_support_indices = *support_indices;
      // check if the score is below the threshold. If that is the case this segment should not be split
      if (current_score < min_cut_score_)
      {
//         std::cout << "Score too low, no cutting" << std::endl;
        continue;
      }
    }

    int number_connections_cut = 0;
    for (const auto &point_index : cut_support_indices)
    {
      if (use_clean_cutting_)
      {
        // skip edges where both centroids are on one side of the cutting plane
        std::uint32_t source_sv_label = sv_adjacency_list_[boost::source (seg_to_edgeIDs_map[seg_to_edge_points.first][point_index], sv_adjacency_list_)];
        std::uint32_t target_sv_label = sv_adjacency_list_[boost::target (seg_to_edgeIDs_map[seg_to_edge_points.first][point_index], sv_adjacency_list_)];
        // get centroids of vertices
        const pcl::PointXYZRGBA source_centroid = sv_label_to_supervoxel_map_[source_sv_label]->centroid_;
        const pcl::PointXYZRGBA target_centroid = sv_label_to_supervoxel_map_[target_sv_label]->centroid_;
        // this makes a clean cut
        if (pcl::pointToPlaneDistanceSigned (source_centroid, model_coefficients) * pcl::pointToPlaneDistanceSigned (target_centroid, model_coefficients) > 0)
        {
          continue;
        }
      }
      sv_adjacency_list_[seg_to_edgeIDs_map[seg_to_edge_points.first][point_index]].used_for_cutting = true;
      if (sv_adjacency_list_[seg_to_edgeIDs_map[seg_to_edge_points.first][point_index]].is_valid) 
      {
        ++number_connections_cut;
        sv_adjacency_list_[seg_to_edgeIDs_map[seg_to_edge_points.first][point_index]].is_valid = false;
      }
    }
//     std::cout << "We cut " << number_connections_cut << " connections" << std::endl;
    if (number_connections_cut > 0)
      cut_found = true;
  }

  // if not cut has been performed we can stop the recursion
  if (cut_found)
  {
    doGrouping ();
    --depth_levels_left;
    applyCuttingPlane (depth_levels_left);
  }
  else
    pcl::console::print_info ("Could not find any more cuts, stopping recursion\n");
}

/******************************************* Directional weighted RANSAC definitions ******************************************************************/      


template <typename PointT> bool
pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel (int)
{
  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max ())
  {
    PCL_ERROR ("[pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  best_score_ = -std::numeric_limits<double>::max ();

  pcl::Indices selection;
  Eigen::VectorXf model_coefficients;

  unsigned skipped_count = 0;
  // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;

  // Iterate
  while (iterations_ < max_iterations_ && skipped_count < max_skip)
  {
    // Get X samples which satisfy the model criteria and which have a weight > 0
    sac_model_->setIndices (model_pt_indices_);
    sac_model_->getSamples (iterations_, selection);

    if (selection.empty ())
    {
      PCL_ERROR ("[pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel] No samples could be selected!\n");
      break;
    }

    // Search for inliers in the point cloud for the current plane model M
    if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
    {
      //++iterations_;
      ++skipped_count;
      continue;
    }
    // weight distances to get the score (only using connected inliers)
    sac_model_->setIndices (full_cloud_pt_indices_);

    pcl::IndicesPtr current_inliers (new pcl::Indices);
    sac_model_->selectWithinDistance (model_coefficients, threshold_, *current_inliers);
    double current_score = 0;
    Eigen::Vector3f plane_normal (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    for (const auto &current_index : *current_inliers)
    {
      double index_score = weights_[current_index];
      if (use_directed_weights_)
        // the sqrt(2) factor was used in the paper and was meant for making the scores better comparable between directed and undirected weights
        index_score *= 1.414 * (std::abs (plane_normal.dot (point_cloud_ptr_->at (current_index).getNormalVector3fMap ())));

      current_score += index_score;
    }
    // normalize by the total number of inliers
    current_score /= current_inliers->size ();
    
    // Better match ?
    if (current_score > best_score_)
    {
      best_score_ = current_score;
      // Save the current model/inlier/coefficients selection as being the best so far
      model_ = selection;
      model_coefficients_ = model_coefficients;
    }

    ++iterations_;
    PCL_DEBUG ("[pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel] Trial %d (max %d): score is %f (best is: %f so far).\n", iterations_, max_iterations_, current_score, best_score_);
    if (iterations_ > max_iterations_)
    {
      PCL_DEBUG ("[pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
      break;
    }
  }
//   std::cout << "Took us " << iterations_ - 1 << " iterations" << std::endl;
  PCL_DEBUG ("[pcl::CPCSegmentation<PointT>::WeightedRandomSampleConsensus::computeModel] Model: %lu size, %f score.\n", model_.size (), best_score_);

  if (model_.empty ())
  {
    inliers_.clear ();
    return (false);
  }

  // Get the set of inliers that correspond to the best model found so far
  sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
  return (true);
}

#endif // PCL_SEGMENTATION_IMPL_CPC_SEGMENTATION_HPP_
