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
#include <pcl/recognition/hv/hypotheses_verification.h>
#include <boost/graph/adjacency_list.hpp>

#include <memory>

namespace pcl
{

  /** \brief A hypothesis verification method proposed in
    * "An Efficient RANSAC for 3D Object Recognition in Noisy and Occluded Scenes", C. Papazov and D. Burschka, ACCV 2010
    * \author Aitor Aldoma, Federico Tombari
    * \ingroup recognition
    */

  template<typename ModelT, typename SceneT>
  class PCL_EXPORTS PapazovHV : public HypothesisVerification<ModelT, SceneT>
  {
    using HypothesisVerification<ModelT, SceneT>::mask_;
    using HypothesisVerification<ModelT, SceneT>::scene_cloud_downsampled_;
    using HypothesisVerification<ModelT, SceneT>::scene_downsampled_tree_;
    using HypothesisVerification<ModelT, SceneT>::visible_models_;
    using HypothesisVerification<ModelT, SceneT>::complete_models_;
    using HypothesisVerification<ModelT, SceneT>::resolution_;
    using HypothesisVerification<ModelT, SceneT>::inliers_threshold_;

    float conflict_threshold_size_;
    float penalty_threshold_;
    float support_threshold_;

    class RecognitionModel
    {
      public:
        std::vector<int> explained_; //indices vector referencing explained_by_RM_
        typename pcl::PointCloud<ModelT>::Ptr cloud_;
        typename pcl::PointCloud<ModelT>::Ptr complete_cloud_;
        int bad_information_;
        int id_;
    };

    using RecognitionModelPtr = std::shared_ptr<RecognitionModel>;

    std::vector<int> explained_by_RM_; //represents the points of scene_cloud_ that are explained by the recognition models
    std::vector<RecognitionModelPtr> recognition_models_;
    std::vector<std::vector<RecognitionModelPtr>> points_explained_by_rm_; //if inner size > 1, conflict
    std::map<int, RecognitionModelPtr> graph_id_model_map_;

    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, RecognitionModelPtr>;
    Graph conflict_graph_;

    //builds the conflict_graph
    void buildConflictGraph();
    //non-maxima suppresion on the conflict graph
    void nonMaximaSuppresion();
    //create recognition models
    void initialize();

    public:
      PapazovHV() : HypothesisVerification<ModelT,SceneT>() {
        support_threshold_ = 0.1f;
        penalty_threshold_ = 0.1f;
        conflict_threshold_size_ = 0.02f;
      }

      void setConflictThreshold(float t) {
        conflict_threshold_size_ = t;
      }

      void setSupportThreshold(float t) {
        support_threshold_ = t;
      }

      void setPenaltyThreshold(float t) {
        penalty_threshold_ = t;
      }

      //build conflict graph
      //non-maxima supression
      void verify() override;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/hv/hv_papazov.hpp>
#endif
