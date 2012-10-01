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
#include <pcl/recognition/hv/hv_papazov.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
  void
  pcl::PapazovHV<ModelT, SceneT>::initialize ()
  {

    //clear stuff
    recognition_models_.clear ();
    graph_id_model_map_.clear ();
    conflict_graph_.clear ();
    explained_by_RM_.clear ();
    points_explained_by_rm_.clear ();

    // initialize mask...
    mask_.resize (complete_models_.size ());
    for (size_t i = 0; i < complete_models_.size (); i++)
      mask_[i] = true;

    // initialize explained_by_RM
    explained_by_RM_.resize (scene_cloud_downsampled_->points.size ());
    points_explained_by_rm_.resize (scene_cloud_downsampled_->points.size ());

    // initalize model
    for (size_t m = 0; m < complete_models_.size (); m++)
    {
      boost::shared_ptr < RecognitionModel > recog_model (new RecognitionModel);
      // voxelize model cloud
      recog_model->cloud_.reset (new pcl::PointCloud<ModelT>);
      recog_model->complete_cloud_.reset (new pcl::PointCloud<ModelT>);
      recog_model->id_ = static_cast<int> (m);

      pcl::VoxelGrid<ModelT> voxel_grid;
      voxel_grid.setInputCloud (visible_models_[m]);
      voxel_grid.setLeafSize (resolution_, resolution_, resolution_);
      voxel_grid.filter (*(recog_model->cloud_));

      pcl::VoxelGrid<ModelT> voxel_grid_complete;
      voxel_grid_complete.setInputCloud (complete_models_[m]);
      voxel_grid_complete.setLeafSize (resolution_, resolution_, resolution_);
      voxel_grid_complete.filter (*(recog_model->complete_cloud_));

      std::vector<int> explained_indices;
      std::vector<int> outliers;
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;

      for (size_t i = 0; i < recog_model->cloud_->points.size (); i++)
      {
        if (!scene_downsampled_tree_->radiusSearch (recog_model->cloud_->points[i], inliers_threshold_, nn_indices, nn_distances,
                                                    std::numeric_limits<int>::max ()))
        {
          outliers.push_back (static_cast<int> (i));
        }
        else
        {
          for (size_t k = 0; k < nn_distances.size (); k++)
          {
            explained_indices.push_back (nn_indices[k]); //nn_indices[k] points to the scene
          }
        }
      }

      std::sort (explained_indices.begin (), explained_indices.end ());
      explained_indices.erase (std::unique (explained_indices.begin (), explained_indices.end ()), explained_indices.end ());

      recog_model->bad_information_ = static_cast<int> (outliers.size ());

      if ((static_cast<float> (recog_model->bad_information_) / static_cast<float> (recog_model->complete_cloud_->points.size ()))
          <= penalty_threshold_ && (static_cast<float> (explained_indices.size ())
          / static_cast<float> (recog_model->complete_cloud_->points.size ())) >= support_threshold_)
      {
        recog_model->explained_ = explained_indices;
        recognition_models_.push_back (recog_model);

        // update explained_by_RM_, add 1
        for (size_t i = 0; i < explained_indices.size (); i++)
        {
          explained_by_RM_[explained_indices[i]]++;
          points_explained_by_rm_[explained_indices[i]].push_back (recog_model);
        }
      }
      else
      {
        mask_[m] = false; // the model didnt survive the sequential check...
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
  void
  pcl::PapazovHV<ModelT, SceneT>::nonMaximaSuppresion ()
  {
    // iterate over all vertices of the graph and check if they have a better neighbour, then remove that vertex
    typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIterator;
    VertexIterator vi, vi_end, next;
    boost::tie (vi, vi_end) = boost::vertices (conflict_graph_);

    for (next = vi; next != vi_end; next++)
    {
      const typename Graph::vertex_descriptor v = boost::vertex (*next, conflict_graph_);
      typename boost::graph_traits<Graph>::adjacency_iterator ai;
      typename boost::graph_traits<Graph>::adjacency_iterator ai_end;

      boost::shared_ptr < RecognitionModel > current = static_cast<boost::shared_ptr<RecognitionModel> > (graph_id_model_map_[int (v)]);

      bool a_better_one = false;
      for (boost::tie (ai, ai_end) = boost::adjacent_vertices (v, conflict_graph_); (ai != ai_end) && !a_better_one; ++ai)
      {
        boost::shared_ptr < RecognitionModel > neighbour = static_cast<boost::shared_ptr<RecognitionModel> > (graph_id_model_map_[int (*ai)]);
        if ((neighbour->explained_.size () >= current->explained_.size ()) && mask_[neighbour->id_])
        {
          a_better_one = true;
        }
      }

      if (a_better_one)
      {
        mask_[current->id_] = false;
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
  void
  pcl::PapazovHV<ModelT, SceneT>::buildConflictGraph ()
  {
    // create vertices for the graph
    for (size_t i = 0; i < (recognition_models_.size ()); i++)
    {
      const typename Graph::vertex_descriptor v = boost::add_vertex (recognition_models_[i], conflict_graph_);
      graph_id_model_map_[int (v)] = static_cast<boost::shared_ptr<RecognitionModel> > (recognition_models_[i]);
    }

    // iterate over the remaining models and check for each one if there is a conflict with another one
    for (size_t i = 0; i < recognition_models_.size (); i++)
    {
      for (size_t j = i; j < recognition_models_.size (); j++)
      {
        if (i != j)
        {
          float n_conflicts = 0.f;
          // count scene points explained by both models
          for (size_t k = 0; k < explained_by_RM_.size (); k++)
          {
            if (explained_by_RM_[k] > 1)
            {
              // this point could be a conflict
              bool i_found = false;
              bool j_found = false;
              bool both_found = false;
              for (size_t kk = 0; (kk < points_explained_by_rm_[k].size ()) && !both_found; kk++)
              {
                if (points_explained_by_rm_[k][kk]->id_ == recognition_models_[i]->id_)
                  i_found = true;

                if (points_explained_by_rm_[k][kk]->id_ == recognition_models_[j]->id_)
                  j_found = true;

                if (i_found && j_found)
                  both_found = true;
              }

              if (both_found)
                n_conflicts += 1.f;
            }
          }

          // check if number of points is big enough to create a conflict
          bool add_conflict = false;
          add_conflict = ((n_conflicts / static_cast<float> (recognition_models_[i]->complete_cloud_->points.size ())) > conflict_threshold_size_)
              || ((n_conflicts / static_cast<float> (recognition_models_[j]->complete_cloud_->points.size ())) > conflict_threshold_size_);

          if (add_conflict)
          {
            boost::add_edge (i, j, conflict_graph_);
          }
        }
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
  void
  pcl::PapazovHV<ModelT, SceneT>::verify ()
  {
    initialize ();
    buildConflictGraph ();
    nonMaximaSuppresion ();
  }

#define PCL_INSTANTIATE_PapazovHV(T1,T2) template class PCL_EXPORTS pcl::PapazovHV<T1,T2>;
