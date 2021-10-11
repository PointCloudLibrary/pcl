/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/pcl_macros.h>
#include "pcl/recognition/hv/occlusion_reasoning.h"
#include "pcl/recognition/impl/hv/occlusion_reasoning.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

namespace pcl
{

  /**
   * \brief Abstract class for hypotheses verification methods
   * \author Aitor Aldoma, Federico Tombari
   */

  template<typename ModelT, typename SceneT>
  class PCL_EXPORTS HypothesisVerification
  {

  protected:
    /*
     * \brief Boolean vector indicating if a hypothesis is accepted/rejected (output of HV stage)
     */
    std::vector<bool> mask_;
    /*
     * \brief Scene point cloud
     */
    typename pcl::PointCloud<SceneT>::ConstPtr scene_cloud_;

    /*
     * \brief Scene point cloud
     */
    typename pcl::PointCloud<SceneT>::ConstPtr occlusion_cloud_;

    bool occlusion_cloud_set_;

    /*
     * \brief Downsampled scene point cloud
     */
     typename pcl::PointCloud<SceneT>::Ptr scene_cloud_downsampled_;

    /*
     * \brief Scene tree of the downsampled cloud
     */
    typename pcl::search::KdTree<SceneT>::Ptr scene_downsampled_tree_;

    /*
     * \brief Vector of point clouds representing the 3D models after occlusion reasoning
	 * the 3D models are pruned of occluded points, and only visible points are left. 
	 * the coordinate system is that of the scene cloud
     */
    typename std::vector<typename pcl::PointCloud<ModelT>::ConstPtr> visible_models_;

    std::vector<typename pcl::PointCloud<pcl::Normal>::ConstPtr> visible_normal_models_;
    /*
     * \brief Vector of point clouds representing the complete 3D model (in same coordinates as the scene cloud)
     */
    typename std::vector<typename pcl::PointCloud<ModelT>::ConstPtr> complete_models_;

    std::vector<typename pcl::PointCloud<pcl::Normal>::ConstPtr> complete_normal_models_;
    /*
     * \brief Resolutions in pixel for the depth scene buffer
     */
    int zbuffer_scene_resolution_;
    /*
     * \brief Resolutions in pixel for the depth model self-occlusion buffer
     */
    int zbuffer_self_occlusion_resolution_;
    /*
     * \brief The resolution of models and scene used to verify hypotheses (in meters)
     */
    float resolution_;

    /*
     * \brief Threshold for inliers
     */
    float inliers_threshold_;

    /*
     * \brief Threshold to consider a point being occluded
     */
    float occlusion_thres_;

    /*
     * \brief Whether the HV method requires normals or not, by default = false
     */
    bool requires_normals_;

    /*
     * \brief Whether the normals have been set
     */
    bool normals_set_;
  public:

    HypothesisVerification ()
    {
      zbuffer_scene_resolution_ = 100;
      zbuffer_self_occlusion_resolution_ = 150;
      resolution_ = 0.005f;
      inliers_threshold_ = static_cast<float>(resolution_);
      occlusion_cloud_set_ = false;
      occlusion_thres_ = 0.005f;
      normals_set_ = false;
      requires_normals_ = false;
    }

    virtual
    ~HypothesisVerification() = default;

    bool getRequiresNormals() {
      return requires_normals_;
    }

    /*
     *  \brief Sets the resolution of scene cloud and models used to verify hypotheses
     *  mask r resolution
     */
    void
    setResolution(float r) {
      resolution_ = r;
    }

    /*
     *  \brief Sets the occlusion threshold
     *  mask t threshold
     */
    void
    setOcclusionThreshold(float t) {
      occlusion_thres_ = t;
    }

    /*
     *  \brief Sets the resolution of scene cloud and models used to verify hypotheses
     *  mask r resolution
     */
    void
    setInlierThreshold(float r) {
      inliers_threshold_ = r;
    }

    /*
     *  \brief Returns a vector of booleans representing which hypotheses have been accepted/rejected (true/false)
     *  mask vector of booleans
     */

    void
    getMask (std::vector<bool> & mask)
    {
      mask = mask_;
    }

    /*
     *  \brief Sets the 3D complete models. NOTE: If addModels is called with occlusion_reasoning=true, then
     *  there is no need to call this function.
     *  mask models Vector of point clouds representing the models (in same coordinates as the scene_cloud_)
     */

    void
    addCompleteModels (std::vector<typename pcl::PointCloud<ModelT>::ConstPtr> & complete_models)
    {
      complete_models_ = complete_models;
    }

    /*
     *  \brief Sets the normals of the 3D complete models and sets normals_set_ to true.
     *  Normals need to be added before calling the addModels method.
     *  complete_models The normals of the models.
     */
    void
    addNormalsClouds (std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> & complete_models)
    {
      complete_normal_models_ = complete_models;
      normals_set_ = true;
    }

    /*
     *  \brief Sets the models (recognition hypotheses) - requires the scene_cloud_ to be set first if reasoning about occlusions
     *  mask models Vector of point clouds representing the models (in same coordinates as the scene_cloud_)
     */
    void
    addModels (std::vector<typename pcl::PointCloud<ModelT>::ConstPtr> & models, bool occlusion_reasoning = false)
    {

      mask_.clear();
      if(!occlusion_cloud_set_) {
        PCL_WARN("Occlusion cloud not set, using scene_cloud instead...\n");
        occlusion_cloud_ = scene_cloud_;
      }

      if (!occlusion_reasoning)
        visible_models_ = models;
      else
      {
        //we need to reason about occlusions before setting the model
        if (scene_cloud_ == nullptr)
        {
          PCL_ERROR("setSceneCloud should be called before adding the model if reasoning about occlusions...\n");
        }

        if (!occlusion_cloud_->isOrganized ())
        {
          PCL_WARN("Scene not organized... filtering using computed depth buffer\n");
        }

        pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT> zbuffer_scene (zbuffer_scene_resolution_, zbuffer_scene_resolution_, 1.f);
        if (!occlusion_cloud_->isOrganized ())
        {
          zbuffer_scene.computeDepthMap (occlusion_cloud_, true);
        }

        for (std::size_t i = 0; i < models.size (); i++)
        {

          //self-occlusions
          typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> ());
          typename pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT> zbuffer_self_occlusion (75, 75, 1.f);
          zbuffer_self_occlusion.computeDepthMap (models[i], true);
          pcl::Indices indices;
          zbuffer_self_occlusion.filter (models[i], indices, 0.005f);
          pcl::copyPointCloud (*models[i], indices, *filtered);

          if(normals_set_ && requires_normals_) {
            pcl::PointCloud<pcl::Normal>::Ptr filtered_normals (new pcl::PointCloud<pcl::Normal> ());
            pcl::copyPointCloud(*complete_normal_models_[i], indices, *filtered_normals);
            visible_normal_models_.push_back(filtered_normals);
          }

          typename pcl::PointCloud<ModelT>::ConstPtr const_filtered(new pcl::PointCloud<ModelT> (*filtered));
          //typename pcl::PointCloud<ModelT>::ConstPtr const_filtered(new pcl::PointCloud<ModelT> (*models[i]));
          //scene-occlusions
          if (occlusion_cloud_->isOrganized ())
          {
            filtered = pcl::occlusion_reasoning::filter<ModelT,SceneT> (occlusion_cloud_, const_filtered, 525.f, occlusion_thres_);
          }
          else
          {
            zbuffer_scene.filter (const_filtered, filtered, occlusion_thres_);
          }

          visible_models_.push_back (filtered);
        }

        complete_models_ = models;
      }

      occlusion_cloud_set_ = false;
      normals_set_ = false;
    }

    /*
     *  \brief Sets the scene cloud
     *  scene_cloud Point cloud representing the scene
     */

    void
    setSceneCloud (const typename pcl::PointCloud<SceneT>::Ptr & scene_cloud)
    {

      complete_models_.clear();
      visible_models_.clear();
      visible_normal_models_.clear();

      scene_cloud_ = scene_cloud;
      scene_cloud_downsampled_.reset(new pcl::PointCloud<SceneT>());

      pcl::VoxelGrid<SceneT> voxel_grid;
      voxel_grid.setInputCloud (scene_cloud);
      voxel_grid.setLeafSize (resolution_, resolution_, resolution_);
      voxel_grid.setDownsampleAllData(true);
      voxel_grid.filter (*scene_cloud_downsampled_);

      //initialize kdtree for search
      scene_downsampled_tree_.reset (new pcl::search::KdTree<SceneT>);
      scene_downsampled_tree_->setInputCloud(scene_cloud_downsampled_);
    }

    void setOcclusionCloud (const typename pcl::PointCloud<SceneT>::Ptr & occ_cloud)
    {
      occlusion_cloud_ = occ_cloud;
      occlusion_cloud_set_ = true;
    }

    /*
     *  \brief Function that performs the hypotheses verification, needs to be implemented in the subclasses
     *  This function modifies the values of mask_ and needs to be called after both scene and model have been added
     */

    virtual void
    verify ()=0;
  };

}
