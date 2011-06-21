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
 */


#ifndef PCL_PPF_REGISTRATION_H_
#define PCL_PPF_REGISTRATION_H_

#include "pcl/registration/registration.h"
#include <pcl/features/ppf.h>
#include <boost/unordered_map.hpp>

namespace pcl
{
  /// @TODO should be placed somewhere else
  class PPFHashMapSearch
  {
    public:
      /** \brief Data structure to hold the information for the key in the feature hash map
        * \note It uses multiple pair levels in order to enable the usage of the boost::hash function
        * which has the std::pair implementation (i.e., does not require a custom hash function)
        */
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
      typedef boost::unordered_multimap<HashKeyStruct, std::pair<size_t, size_t> > FeatureHashMapType;
      typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;
      typedef boost::shared_ptr<PPFHashMapSearch> Ptr;

      PPFHashMapSearch (float a_angle_discretization_step = 12.0 / 180 * M_PI,
                        float a_distance_discretization_step = 0.01)
      :  angle_discretization_step (a_angle_discretization_step),
         distance_discretization_step (a_distance_discretization_step)
      {
        feature_hash_map = FeatureHashMapTypePtr (new FeatureHashMapType);
        internals_initialized = false;
      }


      void
      setInputFeatureCloud (PointCloud<PPFSignature>::ConstPtr feature_cloud);

      void
      nearestNeighborSearch (float &f1, float &f2, float &f3, float &f4,
                             std::vector<std::pair<size_t, size_t> > &indices);

      Ptr
      makeShared()
      {
        return Ptr (new PPFHashMapSearch (*this));
      }

      std::vector <std::vector <float> > alpha_m;

      float
      getAngleDiscretizationStep () { return angle_discretization_step; }

      float
      getDistanceDiscretizationStep () { return distance_discretization_step; }

      private:
      FeatureHashMapTypePtr feature_hash_map;
      bool internals_initialized;

      /// parameters
    float angle_discretization_step, distance_discretization_step;
  };

  /** \brief 
    * \author Alex Ichim
    */
  template <typename PointT, typename PointNT>
  class PPFRegistration : public Registration<PointT, PointT>
  {
    public:
      /** \note initially used std::pair<Eigen::Affine3f, unsigned int>, but it proved problematic
        * because of the Eigen structures alignment problems - std::pair does not have a custom allocator
        */
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
      typedef std::vector<PoseWithVotes, Eigen::aligned_allocator<PoseWithVotes> > PoseWithVotesList;

      typedef typename Registration<PointT, PointNT>::PointCloudSource PointCloudInput;
      typedef typename PointCloudInput::ConstPtr PointCloudInputConstPtr;

      typedef typename Registration<PointT, PointNT>::PointCloudTarget PointCloudInputNormals;
      typedef typename PointCloudInputNormals::ConstPtr PointCloudInputNormalsConstPtr;

      using Registration<PointT, PointT>::input_;
      using Registration<PointT, PointT>::target_;
      using Registration<PointT, PointT>::converged_;
      using Registration<PointT, PointT>::final_transformation_;
      using Registration<PointT, PointT>::transformation_;

      /** \brief Constructor
        * \param a_scene_reference_point_sampling_rate
        * \param a_clustering_position_diff_threshold
        * \param a_clustering_rotation_diff_threshold
        */
      PPFRegistration (unsigned int a_scene_reference_point_sampling_rate = 5,
                       float a_clustering_position_diff_threshold = 0.01,
                       float a_clustering_rotation_diff_threshold = 20.0 / 180 * M_PI)
      :  scene_reference_point_sampling_rate (a_scene_reference_point_sampling_rate),
         clustering_position_diff_threshold (a_clustering_position_diff_threshold),
         clustering_rotation_diff_threshold (a_clustering_rotation_diff_threshold)
      {
        search_method_set = false;
        cloud_model_set = false;
        cloud_model_normals_set = false;
        cloud_scene_normals_set = false;
      }

      /** \brief
        * \param a_search_method
        */
      void
      setSearchMethod (PPFHashMapSearch::Ptr a_search_method);

      /** \brief */
      void
      setInputCloud (const PointCloudInputConstPtr &cloud)
      {
        PCL_WARN("PPFRegistration: setInputCloud(...) method disabled - use setSourceClouds instead.\n");
      }

      /** \brief */
      void
      getInputCloud ()
      {
        PCL_WARN("PPFRegistration: getInputCloud(...) method disabled - use getSourceClouds(...) instead.\n");
      }

      /** \brief */
      void
      setSourceClouds (const PointCloudInputConstPtr &cloud,
                       const PointCloudInputNormalsConstPtr &normals);

      /** \brief */
      void
      getSourceClouds (PointCloudInputConstPtr &out_cloud,
                       PointCloudInputNormalsConstPtr &out_normals);

      /** \brief */
      void
      setInputTargetNormals (const PointCloudInputNormalsConstPtr &target_normals);

      /*    void setSourceClouds (const PointCloudInputConstPtr &cloud,
                            const PointCloudInputNormalsConstPtr &normals,
                            std::string &key);

      void getSourceClouds (std::string &key,
                            boost::unordered_map<std::string, std::pair<PointCloudInputConstPtr, PointCloudInputNormalsConstPtr> > &out_cloud_model_map);
       */



    private:

      /** \brief */
      void
      computeTransformation (PointCloudInput &output);


      /** \brief */
      PPFHashMapSearch::Ptr search_method;
      /** \brief */
      unsigned int scene_reference_point_sampling_rate;
      /** \brief */
      float clustering_position_diff_threshold, clustering_rotation_diff_threshold;

      //boost::unordered_map<std::string, std::pair<PointCloudInputConstPtr, PointCloudInputNormalsConstPtr> > cloud_model_map;
      /** \brief */
      PointCloudInputConstPtr cloud_model;
      /** \brief */
      PointCloudInputNormalsConstPtr cloud_model_normals, cloud_scene_normals;

      /** \brief */
      bool search_method_set, cloud_model_set, cloud_model_normals_set, cloud_scene_normals_set;


      /** \brief */
      static bool
      poseWithVotesCompareFunction (const PoseWithVotes &a,
                                    const PoseWithVotes &b);

      /** \brief */
      static bool
      clusterVotesCompareFunction (const std::pair<size_t, unsigned int> &a,
                                   const std::pair<size_t, unsigned int> &b);

      /** \brief */
      void
      clusterPoses (PoseWithVotesList &poses,
                    PoseWithVotesList &result);

      /** \brief */
      bool
      posesWithinErrorBounds (Eigen::Affine3f &pose1,
                              Eigen::Affine3f &pose2);

  };
}

#include "pcl/registration/impl/ppf_registration.hpp"

#endif // PCL_PPF_REGISTRATION_H_
