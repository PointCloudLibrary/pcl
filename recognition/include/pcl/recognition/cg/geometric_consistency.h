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

#ifndef PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_H_
#define PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_H_

#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/point_cloud.h>

namespace pcl
{
 
  /** \brief Class implementing a 3D correspondence grouping enforcing geometric consistency among feature correspondences
    *
    * \author Federico Tombari, Tommaso Cavallari, Aitor Aldoma
    * \ingroup recognition
    */
  template<typename PointModelT, typename PointSceneT>
  class GeometricConsistencyGrouping : public CorrespondenceGrouping<PointModelT, PointSceneT>
  {
    public:
      typedef pcl::PointCloud<PointModelT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::SceneCloudConstPtr SceneCloudConstPtr;

      /** \brief Constructor */
      GeometricConsistencyGrouping () 
        : gc_threshold_ (3)
        , gc_size_ (1.0)
        , found_transformations_ ()
      {}

      
      /** \brief Sets the minimum cluster size
        * \param[in] threshold the minimum cluster size 
        */
      inline void
      setGCThreshold (int threshold)
      {
        gc_threshold_ = threshold;
      }

      /** \brief Gets the minimum cluster size.
        * 
        * \return the minimum cluster size used by GC.
        */
      inline int
      getGCThreshold () const
      {
        return (gc_threshold_);
      }

      /** \brief Sets the consensus set resolution. This should be in metric units.
        * 
        * \param[in] gc_size consensus set resolution.
        */
      inline void
      setGCSize (double gc_size)
      {
        gc_size_ = gc_size;
      }

      /** \brief Gets the consensus set resolution.
        * 
        * \return the consensus set resolution.
        */
      inline double
      getGCSize () const
      {
        return (gc_size_);
      }

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations);

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        * \param[out] clustered_corrs a vector containing the correspondences for each instance of the model found within the input data (the same output of clusterCorrespondences).
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, std::vector<pcl::Correspondences> &clustered_corrs);

    protected:
      using CorrespondenceGrouping<PointModelT, PointSceneT>::input_;
      using CorrespondenceGrouping<PointModelT, PointSceneT>::scene_;
      using CorrespondenceGrouping<PointModelT, PointSceneT>::model_scene_corrs_;

      /** \brief Minimum cluster size. It shouldn't be less than 3, since at least 3 correspondences are needed to compute the 6DOF pose */
      int gc_threshold_;

      /** \brief Resolution of the consensus set used to cluster correspondences together*/
      double gc_size_;

      /** \brief Transformations found by clusterCorrespondences method. */
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > found_transformations_;

      /** \brief Cluster the input correspondences in order to distinguish between different instances of the model into the scene.
        * 
        * \param[out] model_instances a vector containing the clustered correspondences for each model found on the scene.
        * \return true if the clustering had been successful or false if errors have occurred.
        */ 
      void
      clusterCorrespondences (std::vector<Correspondences> &model_instances);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/cg/geometric_consistency.hpp>
#endif

#endif // PCL_RECOGNITION_GEOMETRIC_CONSISTENCY_H_
