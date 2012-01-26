/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifdef _WIN32
# define NOMINMAX
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif

#include "pcl/cuda/sample_consensus/ransac.h"
#include "pcl/cuda/time_gpu.h"
#include <stdio.h>

namespace pcl
{
  namespace cuda
  {
    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    RandomSampleConsensus<Storage>::computeModel (int debug_verbosity_level)
    {
      // Warn and exit if no threshold was set
      if (threshold_ == DBL_MAX)
      {
        std::cerr << "[pcl::cuda::RandomSampleConsensus::computeModel] No threshold set!" << std::endl;
        return (false);
      }

      iterations_ = 0;
      int n_best_inliers_count = -INT_MAX;
      float k = 1.0;

      Indices inliers;
      typename SampleConsensusModel<Storage>::IndicesPtr inl_stencil;
      Indices selection;
      Coefficients model_coefficients;
      int good_coeff = -1;

      int n_inliers_count = 0;

#if 1
      Hypotheses h;
      sac_model_->generateModelHypotheses (h, max_iterations_);
#endif
      // Iterate
      while (iterations_ < k)
      {
#if 0
        // Get X samples which satisfy the model criteria
        sac_model_->getSamples (iterations_, selection);

        if (selection.empty ()) 
        {
          std::cerr << "[pcl::cuda::RandomSampleConsensus::computeModel] No samples could be selected!" << std::endl;
          break;
        }

        // Search for inliers in the point cloud for the current plane model M
        if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
        {
          //++iterations_;
          continue;
        }

        // Select the inliers that are within threshold_ from the model
        n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);
#endif

#if 1
        float3 centroid;
        n_inliers_count = sac_model_->selectWithinDistance (h, iterations_, threshold_, inl_stencil, centroid);
        //n_inliers_count = sac_model_->countWithinDistance (h, iterations_, threshold_);
#endif
        //if (inliers.empty () && k > 1.0)
        //  continue;

        //n_inliers_count = inliers.size ();

        // Better match ?
        if (n_inliers_count > n_best_inliers_count)
        {
          n_best_inliers_count = n_inliers_count;

          // Save the current model/inlier/coefficients selection as being the best so far
          //inliers_            = inliers;
          //inliers_stencil_    = inliers_stencil;
#if 0
          model_              = selection;
          model_coefficients_ = model_coefficients;
#endif

#if 1
          good_coeff = iterations_;
#endif

          // Compute the k parameter (k=log(z)/log(1-w^n))
          float w = (float)((float)n_best_inliers_count / (float)sac_model_->getIndices ()->size ());
    //      float p_no_outliers = 1.0 - pow (w, (float)selection.size ());
          float p_no_outliers = 1.0f - pow (w, (float)1);
          p_no_outliers = (std::max) (std::numeric_limits<float>::epsilon (), p_no_outliers);       // Avoid division by -Inf
          p_no_outliers = (std::min) (1.0f - std::numeric_limits<float>::epsilon (), p_no_outliers);   // Avoid division by 0.
          if (p_no_outliers == 1.0f)
            k++;
          else
            k = log (1.0f - probability_) / log (p_no_outliers);
        }

        ++iterations_;
        if (debug_verbosity_level > 1)
          fprintf (stderr, "[pcl::cuda::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
        if (iterations_ > max_iterations_)
        {
          if (debug_verbosity_level > 0)
            std::cerr << "[pcl::cuda::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials." << std::endl;
          break;
        }
      }

      if (debug_verbosity_level > 0)
        fprintf (stderr, "[pcl::cuda::RandomSampleConsensus::computeModel] Model: %lu size, %d inliers.\n", (unsigned long) model_.size (), n_best_inliers_count);

    //  if (model_.empty ())
    //  {
    //    inliers_.clear ();
    //    return (false);
    //  }

#if 1
      if (good_coeff == -1)
        return (false);
#endif

    /*  model_coefficients_.resize (4);
    */
      // Get the set of inliers that correspond to the best model found so far
#if 0
      sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_, inliers_stencil_);
#endif

#if 1
      float3 centroid;
      sac_model_->selectWithinDistance (h, good_coeff, threshold_, inliers_stencil_, centroid);//inliers_stencil_);
      std::cerr << "selected model " << good_coeff << std::endl;
#endif
      return (true);
    }

    template class RandomSampleConsensus<Device>;
    template class RandomSampleConsensus<Host>;

  } // namespace
} // namespace

