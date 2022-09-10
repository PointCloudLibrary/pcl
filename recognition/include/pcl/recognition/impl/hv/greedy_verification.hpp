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
#include <pcl/recognition/hv/greedy_verification.h>

template<typename ModelT, typename SceneT>
  void
  pcl::GreedyVerification<ModelT, SceneT>::initialize ()
  {
    //clear stuff
    recognition_models_.clear ();
    points_explained_by_rm_.clear ();

    // initialize mask...
    mask_.resize (visible_models_.size ());
    for (std::size_t i = 0; i < visible_models_.size (); i++)
      mask_[i] = false;

    // initialize explained_by_RM
    points_explained_by_rm_.resize (scene_cloud_downsampled_->size ());

    // initialize model
    for (std::size_t m = 0; m < visible_models_.size (); m++)
    {
      RecognitionModelPtr recog_model (new RecognitionModel);
      // voxelize model cloud
      recog_model->cloud_.reset (new pcl::PointCloud<ModelT>);
      recog_model->id_ = static_cast<int> (m);

      pcl::VoxelGrid<ModelT> voxel_grid;
      voxel_grid.setInputCloud (visible_models_[m]);
      voxel_grid.setLeafSize (resolution_, resolution_, resolution_);
      voxel_grid.filter (*(recog_model->cloud_));

      std::vector<int> explained_indices;
      std::vector<int> outliers;
      pcl::Indices nn_indices;
      std::vector<float> nn_distances;

      for (std::size_t i = 0; i < recog_model->cloud_->size (); i++)
      {
        if (!scene_downsampled_tree_->radiusSearch ((*recog_model->cloud_)[i], inliers_threshold_, nn_indices, nn_distances,
                                                    std::numeric_limits<int>::max ()))
        {
          outliers.push_back (static_cast<int> (i));
        }
        else
        {
          for (std::size_t k = 0; k < nn_distances.size (); k++)
          {
            explained_indices.push_back (nn_indices[k]); //nn_indices[k] points to the scene
          }
        }
      }

      std::sort (explained_indices.begin (), explained_indices.end ());
      explained_indices.erase (std::unique (explained_indices.begin (), explained_indices.end ()), explained_indices.end ());

      recog_model->bad_information_ = static_cast<int> (outliers.size ());
      recog_model->explained_ = explained_indices;
      recog_model->good_information_ = static_cast<int> (explained_indices.size ());
      recog_model->regularizer_ = regularizer_;

      recognition_models_.push_back (recog_model);

      for (const int &explained_index : explained_indices)
      {
        points_explained_by_rm_[explained_index].push_back (recog_model);
      }
    }

    sortModels ();
  }

template<typename ModelT, typename SceneT>
  void
  pcl::GreedyVerification<ModelT, SceneT>::verify ()
  {
    initialize ();

    std::vector<bool> best_solution_;
    best_solution_.resize (recognition_models_.size ());

    for (std::size_t i = 0; i < recognition_models_.size (); i++)
    {
      if (static_cast<float> (recognition_models_[i]->good_information_) > (regularizer_
          * static_cast<float> (recognition_models_[i]->bad_information_)))
      {
        best_solution_[i] = true;
        updateGoodInformation (static_cast<int> (i));
      }
      else
        best_solution_[i] = false;
    }

    for (std::size_t i = 0; i < best_solution_.size (); i++)
    {
      if (best_solution_[i])
      {
        mask_[indices_models_[i].index_] = true;
      }
      else
      {
        mask_[indices_models_[i].index_] = false;
      }
    }
  }

#define PCL_INSTANTIATE_GreedyVerification(T1,T2) template class PCL_EXPORTS pcl::GreedyVerification<T1,T2>;
