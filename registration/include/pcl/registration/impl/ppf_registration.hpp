/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_IMPL_PPF_REGISTRATION_H_
#define PCL_REGISTRATION_IMPL_PPF_REGISTRATION_H_

#include <pcl/common/transforms.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfh_tools.h> // for computePairFeatures
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
void
pcl::PPFRegistration<PointSource, PointTarget>::setInputTarget(
    const PointCloudTargetConstPtr& cloud)
{
  Registration<PointSource, PointTarget>::setInputTarget(cloud);

  scene_search_tree_ =
      typename pcl::KdTreeFLANN<PointTarget>::Ptr(new pcl::KdTreeFLANN<PointTarget>);
  scene_search_tree_->setInputCloud(target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
void
pcl::PPFRegistration<PointSource, PointTarget>::computeTransformation(
    PointCloudSource& output, const Eigen::Matrix4f& guess)
{
  if (!search_method_) {
    PCL_ERROR("[pcl::PPFRegistration::computeTransformation] Search method not set - "
              "skipping computeTransformation!\n");
    return;
  }

  if (guess != Eigen::Matrix4f::Identity()) {
    PCL_ERROR("[pcl::PPFRegistration::computeTransformation] setting initial transform "
              "(guess) not implemented!\n");
  }

  const auto aux_size = static_cast<std::size_t>(
      std::ceil(2 * M_PI / search_method_->getAngleDiscretizationStep()));
  if (std::abs(std::round(2 * M_PI / search_method_->getAngleDiscretizationStep()) -
               2 * M_PI / search_method_->getAngleDiscretizationStep()) > 0.1) {
    PCL_WARN("[pcl::PPFRegistration::computeTransformation] The chosen angle "
             "discretization step (%g) does not result in a uniform discretization. "
             "Consider using e.g. 2pi/%zu or 2pi/%zu\n",
             search_method_->getAngleDiscretizationStep(),
             aux_size - 1,
             aux_size);
  }

  const std::vector<unsigned int> tmp_vec(aux_size, 0);
  std::vector<std::vector<unsigned int>> accumulator_array(input_->size(), tmp_vec);

  PCL_DEBUG("[PPFRegistration] Accumulator array size: %u x %u.\n",
            accumulator_array.size(),
            accumulator_array.back().size());

  PoseWithVotesList voted_poses;
  // Consider every <scene_reference_point_sampling_rate>-th point as the reference
  // point => fix s_r
  float f1, f2, f3, f4;
  for (index_t scene_reference_index = 0;
       scene_reference_index < static_cast<index_t>(target_->size());
       scene_reference_index += scene_reference_point_sampling_rate_) {
    Eigen::Vector3f scene_reference_point =
                        (*target_)[scene_reference_index].getVector3fMap(),
                    scene_reference_normal =
                        (*target_)[scene_reference_index].getNormalVector3fMap();

    float rotation_angle_sg =
        std::acos(scene_reference_normal.dot(Eigen::Vector3f::UnitX()));
    bool parallel_to_x_sg =
        (scene_reference_normal.y() == 0.0f && scene_reference_normal.z() == 0.0f);
    Eigen::Vector3f rotation_axis_sg =
        (parallel_to_x_sg)
            ? (Eigen::Vector3f::UnitY())
            : (scene_reference_normal.cross(Eigen::Vector3f::UnitX()).normalized());
    Eigen::AngleAxisf rotation_sg(rotation_angle_sg, rotation_axis_sg);
    Eigen::Affine3f transform_sg(
        Eigen::Translation3f(rotation_sg * ((-1) * scene_reference_point)) *
        rotation_sg);

    // For every other point in the scene => now have pair (s_r, s_i) fixed
    pcl::Indices indices;
    std::vector<float> distances;
    scene_search_tree_->radiusSearch((*target_)[scene_reference_index],
                                     search_method_->getModelDiameter() / 2,
                                     indices,
                                     distances);
    for (const auto& scene_point_index : indices)
    //    for(std::size_t i = 0; i < target_->size (); ++i)
    {
      // size_t scene_point_index = i;
      if (scene_reference_index != scene_point_index) {
        if (/*pcl::computePPFPairFeature*/ pcl::computePairFeatures(
            (*target_)[scene_reference_index].getVector4fMap(),
            (*target_)[scene_reference_index].getNormalVector4fMap(),
            (*target_)[scene_point_index].getVector4fMap(),
            (*target_)[scene_point_index].getNormalVector4fMap(),
            f1,
            f2,
            f3,
            f4)) {
          std::vector<std::pair<std::size_t, std::size_t>> nearest_indices;
          search_method_->nearestNeighborSearch(f1, f2, f3, f4, nearest_indices);

          // Compute alpha_s angle
          const Eigen::Vector3f scene_point =
              (*target_)[scene_point_index].getVector3fMap();

          const Eigen::Vector3f scene_point_transformed = transform_sg * scene_point;
          float alpha_s =
              std::atan2(-scene_point_transformed(2), scene_point_transformed(1));
          if (std::sin(alpha_s) * scene_point_transformed(2) < 0.0f)
            alpha_s *= (-1);
          alpha_s *= (-1);

          // Go through point pairs in the model with the same discretized feature
          for (const auto& nearest_index : nearest_indices) {
            std::size_t model_reference_index = nearest_index.first;
            std::size_t model_point_index = nearest_index.second;
            // Calculate angle alpha = alpha_m - alpha_s
            float alpha =
                search_method_->alpha_m_[model_reference_index][model_point_index] -
                alpha_s;
            if (alpha < -M_PI) {
              alpha += (2 * M_PI);
            }
            else if (alpha > M_PI) {
              alpha -= (2 * M_PI);
            }
            auto alpha_discretized = static_cast<unsigned int>(std::floor(
                (alpha + M_PI) / search_method_->getAngleDiscretizationStep()));
            accumulator_array[model_reference_index][alpha_discretized]++;
          }
        }
        else
          PCL_ERROR("[pcl::PPFRegistration::computeTransformation] Computing pair "
                    "feature vector between points %u and %u went wrong.\n",
                    scene_reference_index,
                    scene_point_index);
      }
    }

    // the paper says: "For stability reasons, all peaks that received a certain amount
    // of votes relative to the maximum peak are used." No specific value is mentioned,
    // but 90% seems good
    unsigned int max_votes = 0;
    const std::size_t size_i = accumulator_array.size(),
                      size_j = accumulator_array.back().size();
    for (std::size_t i = 0; i < size_i; ++i)
      for (std::size_t j = 0; j < size_j; ++j) {
        if (accumulator_array[i][j] > max_votes)
          max_votes = accumulator_array[i][j];
      }
    max_votes *= 0.9;
    for (std::size_t i = 0; i < size_i; ++i)
      for (std::size_t j = 0; j < size_j; ++j) {
        if (accumulator_array[i][j] >= max_votes) {
          const Eigen::Vector3f model_reference_point = (*input_)[i].getVector3fMap(),
                                model_reference_normal =
                                    (*input_)[i].getNormalVector3fMap();
          const float rotation_angle_mg =
              std::acos(model_reference_normal.dot(Eigen::Vector3f::UnitX()));
          const bool parallel_to_x_mg = (model_reference_normal.y() == 0.0f &&
                                         model_reference_normal.z() == 0.0f);
          const Eigen::Vector3f rotation_axis_mg =
              (parallel_to_x_mg)
                  ? (Eigen::Vector3f::UnitY())
                  : (model_reference_normal.cross(Eigen::Vector3f::UnitX())
                         .normalized());
          const Eigen::AngleAxisf rotation_mg(rotation_angle_mg, rotation_axis_mg);
          const Eigen::Affine3f transform_mg(
              Eigen::Translation3f(rotation_mg * ((-1) * model_reference_point)) *
              rotation_mg);
          const Eigen::Affine3f max_transform =
              transform_sg.inverse() *
              Eigen::AngleAxisf((static_cast<float>(j + 0.5) *
                                     search_method_->getAngleDiscretizationStep() -
                                 M_PI),
                                Eigen::Vector3f::UnitX()) *
              transform_mg;

          voted_poses.push_back(PoseWithVotes(max_transform, accumulator_array[i][j]));
        }
        // Reset accumulator_array for the next set of iterations with a new scene
        // reference point
        accumulator_array[i][j] = 0;
      }
  }
  PCL_DEBUG("[PPFRegistration] Done with the Hough Transform ...\n");

  // Cluster poses for filtering out outliers and obtaining more precise results
  clusterPoses(voted_poses, best_pose_candidates);

  pcl::transformPointCloud(*input_, output, best_pose_candidates.front().pose);

  transformation_ = final_transformation_ = best_pose_candidates.front().pose.matrix();
  converged_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
void
pcl::PPFRegistration<PointSource, PointTarget>::clusterPoses(
    typename pcl::PPFRegistration<PointSource, PointTarget>::PoseWithVotesList& poses,
    typename pcl::PPFRegistration<PointSource, PointTarget>::PoseWithVotesList& result)
{
  PCL_DEBUG("[PPFRegistration] Clustering poses (initially got %zu poses)\n",
            poses.size());
  // Start off by sorting the poses by the number of votes
  sort(poses.begin(), poses.end(), poseWithVotesCompareFunction);

  std::vector<PoseWithVotesList> clusters;
  std::vector<std::pair<std::size_t, unsigned int>> cluster_votes;
  for (std::size_t poses_i = 0; poses_i < poses.size(); ++poses_i) {
    bool found_cluster = false;
    float lowest_position_diff = std::numeric_limits<float>::max(),
          lowest_rotation_diff_angle = std::numeric_limits<float>::max();
    std::size_t best_cluster = 0;
    for (std::size_t clusters_i = 0; clusters_i < clusters.size(); ++clusters_i) {
      // if a pose can be added to more than one cluster (posesWithinErrorBounds returns
      // true), then add it to the one where position and rotation difference are
      // smallest
      float position_diff, rotation_diff_angle;
      if (posesWithinErrorBounds(poses[poses_i].pose,
                                 clusters[clusters_i].front().pose,
                                 position_diff,
                                 rotation_diff_angle)) {
        if (!found_cluster) {
          found_cluster = true;
          best_cluster = clusters_i;
          lowest_position_diff = position_diff;
          lowest_rotation_diff_angle = rotation_diff_angle;
        }
        else if (position_diff < lowest_position_diff &&
                 rotation_diff_angle < lowest_rotation_diff_angle) {
          best_cluster = clusters_i;
          lowest_position_diff = position_diff;
          lowest_rotation_diff_angle = rotation_diff_angle;
        }
      }
    }
    if (found_cluster) {
      clusters[best_cluster].push_back(poses[poses_i]);
      cluster_votes[best_cluster].second += poses[poses_i].votes;
    }
    else {
      // Create a new cluster with the current pose
      PoseWithVotesList new_cluster;
      new_cluster.push_back(poses[poses_i]);
      clusters.push_back(new_cluster);
      cluster_votes.push_back(std::pair<std::size_t, unsigned int>(
          clusters.size() - 1, poses[poses_i].votes));
    }
  }
  PCL_DEBUG("[PPFRegistration] %zu poses remaining after clustering. Now averaging "
            "each cluster and removing clusters with too few votes.\n",
            clusters.size());

  // Sort clusters by total number of votes
  std::sort(cluster_votes.begin(), cluster_votes.end(), clusterVotesCompareFunction);
  // Compute pose average and put them in result vector
  result.clear();
  for (std::size_t cluster_i = 0; cluster_i < clusters.size(); ++cluster_i) {
    // Remove all clusters that have less than 10% of the votes of the peak cluster.
    // This way, if there is e.g. one cluster with far more votes than all other
    // clusters, only that one is kept.
    if (cluster_votes[cluster_i].second < 0.1 * cluster_votes[0].second)
      continue;
    PCL_DEBUG("Winning cluster has #votes: %d and #poses voted: %d.\n",
              cluster_votes[cluster_i].second,
              clusters[cluster_votes[cluster_i].first].size());
    Eigen::Vector3f translation_average(0.0, 0.0, 0.0);
    Eigen::Vector4f rotation_average(0.0, 0.0, 0.0, 0.0);
    for (const auto& vote : clusters[cluster_votes[cluster_i].first]) {
      translation_average += vote.pose.translation();
      /// averaging rotations by just averaging the quaternions in 4D space - reference
      /// "On Averaging Rotations" by CLAUS GRAMKOW
      rotation_average += Eigen::Quaternionf(vote.pose.rotation()).coeffs();
    }

    translation_average /=
        static_cast<float>(clusters[cluster_votes[cluster_i].first].size());
    rotation_average /=
        static_cast<float>(clusters[cluster_votes[cluster_i].first].size());

    Eigen::Affine3f transform_average;
    transform_average.translation().matrix() = translation_average;
    transform_average.linear().matrix() =
        Eigen::Quaternionf(rotation_average).normalized().toRotationMatrix();

    result.push_back(PoseWithVotes(transform_average, cluster_votes[cluster_i].second));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
bool
pcl::PPFRegistration<PointSource, PointTarget>::posesWithinErrorBounds(
    Eigen::Affine3f& pose1,
    Eigen::Affine3f& pose2,
    float& position_diff,
    float& rotation_diff_angle)
{
  position_diff = (pose1.translation() - pose2.translation()).norm();
  Eigen::AngleAxisf rotation_diff_mat(
      (pose1.rotation().inverse().lazyProduct(pose2.rotation()).eval()));

  rotation_diff_angle = std::abs(rotation_diff_mat.angle());

  return (position_diff < clustering_position_diff_threshold_ &&
          rotation_diff_angle < clustering_rotation_diff_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
bool
pcl::PPFRegistration<PointSource, PointTarget>::poseWithVotesCompareFunction(
    const typename pcl::PPFRegistration<PointSource, PointTarget>::PoseWithVotes& a,
    const typename pcl::PPFRegistration<PointSource, PointTarget>::PoseWithVotes& b)
{
  return (a.votes > b.votes);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
bool
pcl::PPFRegistration<PointSource, PointTarget>::clusterVotesCompareFunction(
    const std::pair<std::size_t, unsigned int>& a,
    const std::pair<std::size_t, unsigned int>& b)
{
  return (a.second > b.second);
}

//#define PCL_INSTANTIATE_PPFRegistration(PointSource,PointTarget) template class
// PCL_EXPORTS pcl::PPFRegistration<PointSource, PointTarget>;

#endif // PCL_REGISTRATION_IMPL_PPF_REGISTRATION_H_
