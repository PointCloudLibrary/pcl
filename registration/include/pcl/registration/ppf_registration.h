/*
 * Software License Agreement (BSD License)
 *
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

#pragma once

#include <pcl/features/ppf.h>
#include <pcl/registration/registration.h>

#include <unordered_map>

namespace pcl {
class PCL_EXPORTS PPFHashMapSearch {
public:
  /** \brief Data structure to hold the information for the key in the feature hash map
   * of the PPFHashMapSearch class \note It uses multiple pair levels in order to enable
   * the usage of the boost::hash function which has the std::pair implementation (i.e.,
   * does not require a custom hash function)
   */
  struct HashKeyStruct : public std::pair<int, std::pair<int, std::pair<int, int>>> {
    HashKeyStruct() = default;

    HashKeyStruct(int a, int b, int c, int d)
    {
      this->first = a;
      this->second.first = b;
      this->second.second.first = c;
      this->second.second.second = d;
    }

    std::size_t
    operator()(const HashKeyStruct& s) const noexcept
    {
      const std::size_t h1 = std::hash<int>{}(s.first);
      const std::size_t h2 = std::hash<int>{}(s.second.first);
      const std::size_t h3 = std::hash<int>{}(s.second.second.first);
      const std::size_t h4 = std::hash<int>{}(s.second.second.second);
      return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
  };
  using FeatureHashMapType =
      std::unordered_multimap<HashKeyStruct,
                              std::pair<std::size_t, std::size_t>,
                              HashKeyStruct>;
  using FeatureHashMapTypePtr = shared_ptr<FeatureHashMapType>;
  using Ptr = shared_ptr<PPFHashMapSearch>;
  using ConstPtr = shared_ptr<const PPFHashMapSearch>;

  /** \brief Constructor for the PPFHashMapSearch class which sets the two step
   * parameters for the enclosed data structure \param angle_discretization_step the
   * step value between each bin of the hash map for the angular values \param
   * distance_discretization_step the step value between each bin of the hash map for
   * the distance values
   */
  PPFHashMapSearch(float angle_discretization_step = 12.0f / 180.0f *
                                                     static_cast<float>(M_PI),
                   float distance_discretization_step = 0.01f)
  : feature_hash_map_(new FeatureHashMapType)
  , internals_initialized_(false)
  , angle_discretization_step_(angle_discretization_step)
  , distance_discretization_step_(distance_discretization_step)
  , max_dist_(-1.0f)
  {}

  /** \brief Method that sets the feature cloud to be inserted in the hash map
   * \param feature_cloud a const smart pointer to the PPFSignature feature cloud
   */
  void
  setInputFeatureCloud(PointCloud<PPFSignature>::ConstPtr feature_cloud);

  /** \brief Function for finding the nearest neighbors for the given feature inside the
   * discretized hash map \param f1 The 1st value describing the query PPFSignature
   * feature \param f2 The 2nd value describing the query PPFSignature feature \param f3
   * The 3rd value describing the query PPFSignature feature \param f4 The 4th value
   * describing the query PPFSignature feature \param indices a vector of pair indices
   * representing the feature pairs that have been found in the bin corresponding to the
   * query feature
   */
  void
  nearestNeighborSearch(float& f1,
                        float& f2,
                        float& f3,
                        float& f4,
                        std::vector<std::pair<std::size_t, std::size_t>>& indices);

  /** \brief Convenience method for returning a copy of the class instance as a
   * shared_ptr */
  Ptr
  makeShared()
  {
    return Ptr(new PPFHashMapSearch(*this));
  }

  /** \brief Returns the angle discretization step parameter (the step value between
   * each bin of the hash map for the angular values) */
  inline float
  getAngleDiscretizationStep() const
  {
    return angle_discretization_step_;
  }

  /** \brief Returns the distance discretization step parameter (the step value between
   * each bin of the hash map for the distance values) */
  inline float
  getDistanceDiscretizationStep() const
  {
    return distance_discretization_step_;
  }

  /** \brief Returns the maximum distance found between any feature pair in the given
   * input feature cloud */
  inline float
  getModelDiameter() const
  {
    return max_dist_;
  }

  std::vector<std::vector<float>> alpha_m_;

private:
  FeatureHashMapTypePtr feature_hash_map_;
  bool internals_initialized_;

  float angle_discretization_step_, distance_discretization_step_;
  float max_dist_;
};

/** \brief Class that registers two point clouds based on their sets of PPFSignatures.
 * Please refer to the following publication for more details:
 *    B. Drost, M. Ulrich, N. Navab, S. Ilic
 *    Model Globally, Match Locally: Efficient and Robust 3D Object Recognition
 *    2010 IEEE Conference on Computer Vision and Pattern Recognition (CVPR)
 *    13-18 June 2010, San Francisco, CA
 *
 * \note This class works in tandem with the PPFEstimation class
 *
 * \author Alexandru-Eugen Ichim
 */
template <typename PointSource, typename PointTarget>
class PPFRegistration : public Registration<PointSource, PointTarget> {
public:
  /** \brief Structure for storing a pose (represented as an Eigen::Affine3f) and an
   * integer for counting votes \note initially used std::pair<Eigen::Affine3f, unsigned
   * int>, but it proved problematic because of the Eigen structures alignment problems
   * - std::pair does not have a custom allocator
   */
  struct PoseWithVotes {
    PoseWithVotes(Eigen::Affine3f& a_pose, unsigned int& a_votes)
    : pose(a_pose), votes(a_votes)
    {}

    Eigen::Affine3f pose;
    unsigned int votes;
  };
  using PoseWithVotesList =
      std::vector<PoseWithVotes, Eigen::aligned_allocator<PoseWithVotes>>;

  /// input_ is the model cloud
  using Registration<PointSource, PointTarget>::input_;
  /// target_ is the scene cloud
  using Registration<PointSource, PointTarget>::target_;
  using Registration<PointSource, PointTarget>::converged_;
  using Registration<PointSource, PointTarget>::final_transformation_;
  using Registration<PointSource, PointTarget>::transformation_;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  /** \brief Empty constructor that initializes all the parameters of the algorithm with
   * default values */
  PPFRegistration()
  : Registration<PointSource, PointTarget>()
  , scene_reference_point_sampling_rate_(5)
  , clustering_position_diff_threshold_(0.01f)
  , clustering_rotation_diff_threshold_(20.0f / 180.0f * static_cast<float>(M_PI))
  {}

  /** \brief Method for setting the position difference clustering parameter
   * \param clustering_position_diff_threshold distance threshold below which two poses
   * are considered close enough to be in the same cluster (for the clustering phase of
   * the algorithm)
   */
  inline void
  setPositionClusteringThreshold(float clustering_position_diff_threshold)
  {
    clustering_position_diff_threshold_ = clustering_position_diff_threshold;
  }

  /** \brief Returns the parameter defining the position difference clustering parameter
   * - distance threshold below which two poses are considered close enough to be in the
   * same cluster (for the clustering phase of the algorithm)
   */
  inline float
  getPositionClusteringThreshold()
  {
    return clustering_position_diff_threshold_;
  }

  /** \brief Method for setting the rotation clustering parameter
   * \param clustering_rotation_diff_threshold rotation difference threshold below which
   * two poses are considered to be in the same cluster (for the clustering phase of the
   * algorithm)
   */
  inline void
  setRotationClusteringThreshold(float clustering_rotation_diff_threshold)
  {
    clustering_rotation_diff_threshold_ = clustering_rotation_diff_threshold;
  }

  /** \brief Returns the parameter defining the rotation clustering threshold
   */
  inline float
  getRotationClusteringThreshold()
  {
    return clustering_rotation_diff_threshold_;
  }

  /** \brief Method for setting the scene reference point sampling rate
   * \param scene_reference_point_sampling_rate sampling rate for the scene reference
   * point
   */
  inline void
  setSceneReferencePointSamplingRate(unsigned int scene_reference_point_sampling_rate)
  {
    scene_reference_point_sampling_rate_ = scene_reference_point_sampling_rate;
  }

  /** \brief Returns the parameter for the scene reference point sampling rate of the
   * algorithm */
  inline unsigned int
  getSceneReferencePointSamplingRate()
  {
    return scene_reference_point_sampling_rate_;
  }

  /** \brief Function that sets the search method for the algorithm
   * \note Right now, the only available method is the one initially proposed by
   * the authors - by using a hash map with discretized feature vectors
   * \param search_method smart pointer to the search method to be set
   */
  inline void
  setSearchMethod(PPFHashMapSearch::Ptr search_method)
  {
    search_method_ = search_method;
  }

  /** \brief Getter function for the search method of the class */
  inline PPFHashMapSearch::Ptr
  getSearchMethod()
  {
    return search_method_;
  }

  /** \brief Provide a pointer to the input target (e.g., the point cloud that we want
   * to align the input source to) \param cloud the input point cloud target
   */
  void
  setInputTarget(const PointCloudTargetConstPtr& cloud) override;

private:
  /** \brief Method that calculates the transformation between the input_ and target_
   * point clouds, based on the PPF features */
  void
  computeTransformation(PointCloudSource& output,
                        const Eigen::Matrix4f& guess) override;

  /** \brief the search method that is going to be used to find matching feature pairs
   */
  PPFHashMapSearch::Ptr search_method_;

  /** \brief parameter for the sampling rate of the scene reference points */
  uindex_t scene_reference_point_sampling_rate_;

  /** \brief position and rotation difference thresholds below which two
   * poses are considered to be in the same cluster (for the clustering phase of the
   * algorithm) */
  float clustering_position_diff_threshold_, clustering_rotation_diff_threshold_;

  /** \brief use a kd-tree with range searches of range max_dist to skip an O(N) pass
   * through the point cloud */
  typename pcl::KdTreeFLANN<PointTarget>::Ptr scene_search_tree_;

  /** \brief static method used for the std::sort function to order two PoseWithVotes
   * instances by their number of votes*/
  static bool
  poseWithVotesCompareFunction(const PoseWithVotes& a, const PoseWithVotes& b);

  /** \brief static method used for the std::sort function to order two pairs <index,
   * votes> by the number of votes (unsigned integer value) */
  static bool
  clusterVotesCompareFunction(const std::pair<std::size_t, unsigned int>& a,
                              const std::pair<std::size_t, unsigned int>& b);

  /** \brief Method that clusters a set of given poses by using the clustering
   * thresholds and their corresponding number of votes (see publication for more
   * details) */
  void
  clusterPoses(PoseWithVotesList& poses, PoseWithVotesList& result);

  /** \brief Method that checks whether two poses are close together - based on the
   * clustering threshold parameters of the class */
  bool
  posesWithinErrorBounds(Eigen::Affine3f& pose1, Eigen::Affine3f& pose2);
};
} // namespace pcl

#include <pcl/registration/impl/ppf_registration.hpp>
