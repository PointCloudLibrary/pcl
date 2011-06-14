/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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

#ifndef PCL_SURFLET_H_
#define PCL_SURFLET_H_

#include <pcl/features/feature.h>
#include <boost/unordered_map.hpp>

namespace pcl
{
  bool 
  resultsCompareFunction (std::pair <Eigen::Affine3f, float> a, std::pair <Eigen::Affine3f, float> b)
  {
    return (a.second > b.second);
  }


  /** \brief Estimate 3D Surflet features.
   *
   * paper...
   *
   * \author Alexandru-Eugen Ichim
   */
  template <typename PointInT, typename PointOutT>
  class SurfletEstimation : public Feature<PointInT, PointOutT>
  {
  public:
    typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudIn;
    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    /// slight hack to enable the usage of the boost::hash <pair <A, B> >
    struct HashKeyStruct : public std::pair <int, std::pair <int, std::pair <int, int> > >
    {
      HashKeyStruct(int a, int b, int c, int d)
      {
        this->first = a;
        this->second.first = b;
        this->second.second.first = c;
        this->second.second.second = d;
      }
    };

        /// this is needed to replace std::pair as it seems to have problems using Eigen structures in MsWindows
    struct PoseWithVotes
    {
      PoseWithVotes(Eigen::Affine3f &a_pose, unsigned int &a_votes)
      : pose (a_pose),
        votes (a_votes)
      {
      }

      Eigen::Affine3f pose;
      unsigned int votes;
    };

    typedef boost::unordered_multimap<HashKeyStruct, std::pair<size_t, size_t> > FeatureHashMapType;
    typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;
    typedef std::vector<PoseWithVotes, Eigen::aligned_allocator<PoseWithVotes> > PoseWithVotesList;


    SurfletEstimation (float a_angle_discretization_step = 12.0 / 180 * M_PI,
                       float a_distance_discretization_step = 0.01,
                       float a_clustering_position_diff_threshold = 0.01,
                       float a_clustering_rotation_diff_threshold = 20.0 / 180 * M_PI,
                       unsigned int a_scene_reference_point_sampling_rate = 5,
                       Eigen::Vector3f a_subsampling_leaf_size = Eigen::Vector3f (0.01, 0.01, 0.01),
                       float a_normal_estimation_search_radius = 0.05)
    :  angle_discretization_step (a_angle_discretization_step),
       distance_discretization_step (a_distance_discretization_step),
       clustering_position_diff_threshold (a_clustering_position_diff_threshold),
       clustering_rotation_diff_threshold (a_clustering_rotation_diff_threshold),
       scene_reference_point_sampling_rate (a_scene_reference_point_sampling_rate),
       subsampling_leaf_size (a_subsampling_leaf_size),
       normal_estimation_search_radius (a_normal_estimation_search_radius)
    {
    }

    FeatureHashMapTypePtr
    computeSurfletModel (const pcl::PointCloud<PointInT> &cloud /* output goes here */);

    PoseWithVotesList
    registerModelToScene (const pcl::PointCloud<PointInT> &cloud_model,
                          const pcl::PointCloud<PointOutT> &cloud_model_normals,
                          const pcl::PointCloud<PointInT> &cloud_scene,
                          FeatureHashMapTypePtr feature_hashmap_model);


    protected:
    void
    computeFeature (PointCloudOut &output);


    private:
    float angle_discretization_step, distance_discretization_step;
    float clustering_position_diff_threshold, clustering_rotation_diff_threshold;
    unsigned int scene_reference_point_sampling_rate;
    Eigen::Vector3f subsampling_leaf_size;
    float normal_estimation_search_radius;

    static bool
    poseWithVotesCompareFunction (const PoseWithVotes &a, const PoseWithVotes &b);

    static bool
    clusterVotesCompareFunction (const std::pair<size_t, unsigned int> &a, const std::pair<size_t, unsigned int> &b);

    void
    clusterPoses (PoseWithVotesList &poses, PoseWithVotesList &result);

    bool
    posesWithinErrorBounds (Eigen::Affine3f &pose1, Eigen::Affine3f &pose2);
  };
}

#endif /* PCL_SURFLET_H_ */
