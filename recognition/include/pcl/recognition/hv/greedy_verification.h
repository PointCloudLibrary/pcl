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

#ifndef PCL_RECOGNITION_HV_GREEDY_H_
#define PCL_RECOGNITION_HV_GREEDY_H_

#include <pcl/pcl_macros.h>
#include <pcl/recognition/hv/hypotheses_verification.h>
#include <pcl/common/common.h>

namespace pcl
{

  /**
   * \brief A greedy hypothesis verification method
   * \author Aitor Aldoma
   */

  template<typename ModelT, typename SceneT>
    class PCL_EXPORTS GreedyVerification : public HypothesisVerification<ModelT, SceneT>
    {
      using HypothesisVerification<ModelT, SceneT>::mask_;
      using HypothesisVerification<ModelT, SceneT>::scene_cloud_downsampled_;
      using HypothesisVerification<ModelT, SceneT>::scene_downsampled_tree_;
      using HypothesisVerification<ModelT, SceneT>::visible_models_;
      using HypothesisVerification<ModelT, SceneT>::resolution_;
      using HypothesisVerification<ModelT, SceneT>::inliers_threshold_;

      /*
       * \brief Recognition model using during the verification
       */
      class RecognitionModel
      {
      public:
        std::vector<int> explained_;
        typename pcl::PointCloud<ModelT>::Ptr cloud_;
        int bad_information_;
        int good_information_;
        int id_;
        float regularizer_;
      };

      /*
       * \brief Sorts recognition models based on the number of explained scene points and visible outliers
       */
      struct sortModelsClass
      {
        bool
        operator() (const boost::shared_ptr<RecognitionModel> & n1, const boost::shared_ptr<RecognitionModel> & n2)
        {
          float val1 = static_cast<float>(n1->good_information_) - static_cast<float>(n1->bad_information_) * n1->regularizer_;
          float val2 = static_cast<float>(n2->good_information_) - static_cast<float>(n2->bad_information_) * n2->regularizer_;
          return val1 > val2;
        }
      } sortModelsOp;


      /*
       * \brief Recognition model indices to keep track of the sorted recognition hypotheses
       */
      struct modelIndices
      {
        int index_;
        boost::shared_ptr<RecognitionModel> model_;
      };

      /*
       * \brief Sorts model indices similar to sortModelsClass
       */
      struct sortModelIndicesClass
      {
        bool
        operator() (const modelIndices & n1, const modelIndices & n2)
        {
          float val1 = static_cast<float>(n1.model_->good_information_) - static_cast<float>(n1.model_->bad_information_) * n1.model_->regularizer_;
          float val2 = static_cast<float>(n2.model_->good_information_) - static_cast<float>(n2.model_->bad_information_) * n2.model_->regularizer_;
          return val1 > val2;
        }
      } sortModelsIndicesOp;

      /** \brief Recognition model and indices */
      std::vector<modelIndices> indices_models_;

      /** \brief Recognition models (hypotheses to be verified) */
      std::vector<boost::shared_ptr<RecognitionModel> > recognition_models_;

      /** \brief Recognition models that explain a scene points. */
      std::vector<std::vector<boost::shared_ptr<RecognitionModel> > > points_explained_by_rm_;

      /** \brief Weighting for outliers */
      float regularizer_;

      /** \brief Initialize the data structures */
      void
      initialize ();

      /** \brief Sorts the hypotheses for the greedy approach */
      void
      sortModels ()
      {
        indices_models_.clear ();
        for (size_t i = 0; i < recognition_models_.size (); i++)
        {
          modelIndices mi;
          mi.index_ = static_cast<int> (i);
          mi.model_ = recognition_models_[i];
          indices_models_.push_back (mi);
        }

        std::sort (indices_models_.begin (), indices_models_.end (), sortModelsIndicesOp);
        //sort also recognition models
        std::sort (recognition_models_.begin (), recognition_models_.end (), sortModelsOp);
      }

      /** \brief Updates conflicting recognition hypotheses when a hypothesis is accepted */
      void
      updateGoodInformation (int i)
      {
        for (size_t k = 0; k < recognition_models_[i]->explained_.size (); k++)
        {
          //update good_information_ for all hypotheses that were explaining the same points as hypothesis i
          for (size_t kk = 0; kk < points_explained_by_rm_[recognition_models_[i]->explained_[k]].size (); kk++)
          {
            (points_explained_by_rm_[recognition_models_[i]->explained_[k]])[kk]->good_information_--;
            (points_explained_by_rm_[recognition_models_[i]->explained_[k]])[kk]->bad_information_++;
          }
        }
      }

    public:

      /** \brief Constructor
       * \param[in] reg Regularizer value
       **/
      GreedyVerification (float reg = 1.5f) :
        HypothesisVerification<ModelT, SceneT> ()
      {
        regularizer_ = reg;
      }

      /** \brief Starts verification */
      void
      verify ();
    };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/hv/greedy_verification.hpp>
#endif

#endif /* PCL_RECOGNITION_HV_GREEDY_H_ */
