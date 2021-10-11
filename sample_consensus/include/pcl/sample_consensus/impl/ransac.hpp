/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_

#include <pcl/sample_consensus/ransac.h>
#ifdef _OPENMP
#include <omp.h>
#endif

#if defined _OPENMP && _OPENMP >= 201107 // We need OpenMP 3.1 for the atomic constructs
#define OPENMP_AVAILABLE_RANSAC true
#else
#define OPENMP_AVAILABLE_RANSAC false
#endif

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RandomSampleConsensus<PointT>::computeModel (int)
{
  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max())
  {
    PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  std::size_t n_best_inliers_count = 0;
  double k = std::numeric_limits<double>::max();

  Indices selection;
  Eigen::VectorXf model_coefficients (sac_model_->getModelSize ());

  const double log_probability  = std::log (1.0 - probability_);
  const double one_over_indices = 1.0 / static_cast<double> (sac_model_->getIndices ()->size ());

  unsigned skipped_count = 0;

  // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;

  int threads = threads_;
  if (threads >= 0)
  {
#if OPENMP_AVAILABLE_RANSAC
    if (threads == 0)
    {
      threads = omp_get_num_procs();
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Automatic number of threads requested, choosing %i threads.\n", threads);
    }
#else
    // Parallelization desired, but not available
    PCL_WARN ("[pcl::RandomSampleConsensus::computeModel] Parallelization is requested, but OpenMP 3.1 is not available! Continuing without parallelization.\n");
    threads = -1;
#endif
  }

#if OPENMP_AVAILABLE_RANSAC
#pragma omp parallel if(threads > 0) num_threads(threads) shared(k, skipped_count, n_best_inliers_count) firstprivate(selection, model_coefficients) // would be nice to have a default(none)-clause here, but then some compilers complain about the shared const variables
#endif
  {
#if OPENMP_AVAILABLE_RANSAC
    if (omp_in_parallel())
#pragma omp master
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Computing in parallel with up to %i threads.\n", omp_get_num_threads());
    else
#endif
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Computing not parallel.\n");

    // Iterate
    while (true) // infinite loop with four possible breaks
    {
      // Get X samples which satisfy the model criteria
#if OPENMP_AVAILABLE_RANSAC
#pragma omp critical(samples)
#endif
      {
        sac_model_->getSamples (iterations_, selection); // The random number generator used when choosing the samples should not be called in parallel
      }

      if (selection.empty ())
      {
        PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
        break;
      }

      // Search for inliers in the point cloud for the current plane model M
      if (!sac_model_->computeModelCoefficients (selection, model_coefficients)) // This function has to be thread-safe
      {
        //++iterations_;
        unsigned skipped_count_tmp;
#if OPENMP_AVAILABLE_RANSAC
#pragma omp atomic capture
#endif
        skipped_count_tmp = ++skipped_count;
        if (skipped_count_tmp < max_skip)
          continue;
        else
          break;
      }

      // Select the inliers that are within threshold_ from the model
      //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
      //if (inliers.empty () && k > 1.0)
      //  continue;

      std::size_t n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_); // This functions has to be thread-safe. Most work is done here

      std::size_t n_best_inliers_count_tmp;
#if OPENMP_AVAILABLE_RANSAC
#pragma omp atomic read
#endif
      n_best_inliers_count_tmp = n_best_inliers_count;

      if (n_inliers_count > n_best_inliers_count_tmp) // This condition is false most of the time, and the critical region is not entered, hopefully leading to more efficient concurrency
      {
#if OPENMP_AVAILABLE_RANSAC
#pragma omp critical(update) // n_best_inliers_count, model_, model_coefficients_, k are shared and read/write must be protected
#endif
        {
          // Better match ?
          if (n_inliers_count > n_best_inliers_count)
          {
            n_best_inliers_count = n_inliers_count; // This write and the previous read of n_best_inliers_count must be consecutive and must not be interrupted!
            n_best_inliers_count_tmp = n_best_inliers_count;

            // Save the current model/inlier/coefficients selection as being the best so far
            model_              = selection;
            model_coefficients_ = model_coefficients;

            // Compute the k parameter (k=std::log(z)/std::log(1-w^n))
            const double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
            double p_no_outliers = 1.0 - std::pow (w, static_cast<double> (selection.size ()));
            p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
            p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
            k = log_probability / std::log (p_no_outliers);
          }
        } // omp critical
      }

      int iterations_tmp;
      double k_tmp;
#if OPENMP_AVAILABLE_RANSAC
#pragma omp atomic capture
#endif
      iterations_tmp = ++iterations_;
#if OPENMP_AVAILABLE_RANSAC
#pragma omp atomic read
#endif
      k_tmp = k;
#if OPENMP_AVAILABLE_RANSAC
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %u inliers (best is: %u so far) (thread %d).\n", iterations_tmp, k_tmp, n_inliers_count, n_best_inliers_count_tmp, omp_get_thread_num());
#else
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %u inliers (best is: %u so far).\n", iterations_tmp, k_tmp, n_inliers_count, n_best_inliers_count_tmp);
#endif
      if (iterations_tmp > k_tmp)
        break;
      if (iterations_tmp > max_iterations_)
      {
        PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
        break;
      }
    } // while
  } // omp parallel

  PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Model: %lu size, %u inliers.\n", model_.size (), n_best_inliers_count);

  if (model_.empty ())
  {
    PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] RANSAC found no model.\n");
    inliers_.clear ();
    return (false);
  }

  // Get the set of inliers that correspond to the best model found so far
  sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
  return (true);
}

#define PCL_INSTANTIATE_RandomSampleConsensus(T) template class PCL_EXPORTS pcl::RandomSampleConsensus<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_

