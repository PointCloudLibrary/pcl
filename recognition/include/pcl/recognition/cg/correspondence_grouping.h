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

#pragma once

#include <pcl/pcl_base.h>
#include <pcl/correspondence.h>
#include <pcl/console/print.h>

namespace pcl
{
  /** \brief Abstract base class for Correspondence Grouping algorithms.
    *
    * \author Tommaso Cavallari, Federico Tombari, Aitor Aldoma
    * \ingroup recognition
    */
  template <typename PointModelT, typename PointSceneT>
  class CorrespondenceGrouping : public PCLBase<PointModelT>
  {
    public:
      using SceneCloud = pcl::PointCloud<PointSceneT>;
      using SceneCloudPtr = typename SceneCloud::Ptr;
      using SceneCloudConstPtr = typename SceneCloud::ConstPtr;

      /** \brief Empty constructor. */
      CorrespondenceGrouping () : scene_ () {}

      /** \brief destructor. */
      ~CorrespondenceGrouping() override 
      {
        scene_.reset ();
        model_scene_corrs_.reset ();
      }

      /** \brief Provide a pointer to the scene dataset.
        * 
        * \param[in] scene the const boost shared pointer to a PointCloud message.
        */
      virtual inline void
      setSceneCloud (const SceneCloudConstPtr &scene)
      {
        scene_ = scene;
      }

      /** \brief Getter for the scene dataset.
        * 
        * \return the const boost shared pointer to a PointCloud message.
        */
      inline SceneCloudConstPtr
      getSceneCloud () const
      {
        return (scene_);
      }

      /** \brief Provide a pointer to the precomputed correspondences between points in the input dataset and 
        * points in the scene dataset. The correspondences are going to be clustered into different model hypotheses
        * by the algorithm.
        * 
        * \param[in] corrs the correspondences between the model and the scene.
        */
      virtual inline void
      setModelSceneCorrespondences (const CorrespondencesConstPtr &corrs)
      {
        model_scene_corrs_ = corrs;
      }

      /** \brief Getter for the precomputed correspondences between points in the input dataset and 
        * points in the scene dataset. 
        * 
        * \return the correspondences between the model and the scene.
        */
      inline CorrespondencesConstPtr
      getModelSceneCorrespondences () const
      {
        return (model_scene_corrs_);
      }

	   /** \brief Getter for the vector of characteristic scales associated to each cluster
        * 
        * \return the vector of characteristic scales (assuming scale = model / scene)
        */
      inline std::vector<double>
      getCharacteristicScales () const
      {
        return (corr_group_scale_);
      }

      /** \brief Clusters the input correspondences belonging to different model instances.
        *
        * \param[out] clustered_corrs a vector containing the correspondences for each instance of the model found within the input data.
        */
      void
      cluster (std::vector<Correspondences> &clustered_corrs);

    protected:
      /** \brief The scene cloud. */
      SceneCloudConstPtr scene_;

      using PCLBase<PointModelT>::input_;

      /** \brief The correspondences between points in the input and the scene datasets. */
      CorrespondencesConstPtr model_scene_corrs_;

	  /** \brief characteristic scale associated to each correspondence subset; 
		* if the cg algorithm can not handle scale invariance, the size of the vector will be 0. */
	  std::vector <double> corr_group_scale_;

      /** \brief The actual clustering method, should be implemented by each subclass.
        *
        * \param[out] clustered_corrs a vector containing the correspondences for each instance of the model found within the input data.
        */
      virtual void
      clusterCorrespondences (std::vector<Correspondences> &clustered_corrs) = 0;

      /** \brief This method should get called before starting the actual computation. 
        *
        * Internally, initCompute() does the following:
        *   - checks if an input dataset is given, and returns false otherwise
        *   - checks if a scene dataset is given, and returns false otherwise
        *   - checks if the model-scene correspondences have been given, and returns false otherwise
        */
      inline bool
      initCompute ()
      {
        if (!PCLBase<PointModelT>::initCompute ())
        {
          return (false);
        }

        if (!scene_)
        {
          PCL_ERROR ("[initCompute] Scene not set.\n");
          return (false);
        }

        if (!input_)
        {
          PCL_ERROR ("[initCompute] Input not set.\n");
          return (false);
        }

        if (!model_scene_corrs_)
        {
          PCL_ERROR ("[initCompute] Model-Scene Correspondences not set.\n");
          return (false);
        }

        return (true);
      }

      /** \brief This method should get called after finishing the actual computation. 
        *
        */
      inline bool
      deinitCompute ()
      {
        return (true);
      }

  };
}

#include <pcl/recognition/impl/cg/correspondence_grouping.hpp>
