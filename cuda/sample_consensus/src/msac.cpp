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

#include <pcl_cuda/sample_consensus/multi_ransac.h>
#include <pcl_cuda/time_gpu.h>
#include <stdio.h>
//CUPRINTF #include "cuPrintf.cu"

int min_nr_in_shape = 5000;

//////////////////////////////////////////////////////////////////////////
template <template <typename> class Storage> bool
pcl_cuda::MultiRandomSampleConsensus<Storage>::computeModel (int debug_verbosity_level)
{
  // Warn and exit if no threshold was set
  if (threshold_ == DBL_MAX)
  {
    std::cerr << "[pcl_cuda::MultiRandomSampleConsensus::computeModel] No threshold set!" << std::endl;
    return (false);
  }

  // compute number of points
  int nr_points = sac_model_->getIndices ()->size ();
  int nr_remaining_points = nr_points;
  //std::cerr << "nr_points = " << nr_points << std::endl;
  // number of total iterations
  unsigned int cur_iteration = 0;
  // number of valid iterations
  int valid_iterations = 0;
  // each batch has a vector of plane coefficients (float4)
  std::vector<Hypotheses> h(max_batches_);
  std::vector<typename Storage<int>::type> h_samples (max_batches_);
  std::vector<float3> centroids (max_batches_ * iterations_per_batch_);
  // current batch number
  int cur_batch = 0;
  //// stencil vector that holds the current inliers
  std::vector<IndicesPtr> hypotheses_inliers_stencils (max_batches_ * iterations_per_batch_);
  std::vector<int> hypotheses_inlier_count (max_batches_ * iterations_per_batch_);
  // initialize some things
  all_inliers_.clear ();
  all_model_coefficients_.clear ();
  all_model_centroids_.clear ();
  int n_inliers_count = 0;
  int n_best_inliers_count = 0;
  int good_coeff = -1;
  float k = max_batches_ * iterations_per_batch_;

    //thrust::host_vector<float3> host_points = sac_model_->getInputCloud()->points;
    //std::cerr << "Input Points:" << std::endl;
    //for (unsigned int print_iter = 0; print_iter < nr_points; ++print_iter)
    //{
    //  std::cerr << print_iter << " : [ " 
    //    << host_points[print_iter].x << ", "
    //    << host_points[print_iter].y << ", "
    //    << host_points[print_iter].z << " ]" << std::endl;
    //}

  ScopeTime t ("ALLLLLLLLLLL");
  do  // multiple models ..
  {
    thrust::host_vector<int> host_samples;
    thrust::host_vector<float4> host_coeffs;
    // make sure that sac_model_->indices_ only contains remaining point indices
    sac_model_->getIndices ();

    // generate a new batch of hypotheses
    {
      ScopeTime t ("generateModelHypotheses");
      sac_model_->generateModelHypotheses (h[cur_batch], h_samples[cur_batch], iterations_per_batch_);
    }
    host_samples = h_samples[cur_batch];
    host_coeffs = h[cur_batch];

    if (debug_verbosity_level > 1)
    {
      std::cerr << "Samples / Coefficients:" << std::endl;
      for (unsigned int print_iter = 0; print_iter < iterations_per_batch_; ++print_iter)
      {
        std::cerr << host_samples[print_iter] << " : [ " 
          << host_coeffs[print_iter].x << ", "
          << host_coeffs[print_iter].y << ", "
          << host_coeffs[print_iter].z << ", "
          << host_coeffs[print_iter].w << std::endl;
      }
    }

    std::vector<bool> hypothesis_valid (max_batches_ * iterations_per_batch_, true);

    // evaluate each hypothesis in this batch
    {
      ScopeTime t ("evaluate");
    for (unsigned int i = 0; i < iterations_per_batch_; i++, cur_iteration ++, valid_iterations ++)
    {
      // hypothesis could be invalid because it's initial sample point was inlier previously
      if (!hypothesis_valid[cur_batch * iterations_per_batch_ + i])
      {
        hypotheses_inlier_count[cur_iteration] = 0;
        valid_iterations --;
        continue;
      }

      // compute inliers for each model
      IndicesPtr inl_stencil;
      {
        ScopeTime t ("selectWithinDistance");
		int d_cur_penalty = 0;
        n_inliers_count = sac_model_->selectWithinDistance (h[cur_batch], i, threshold_, inl_stencil, centroids[cur_iteration], d_cur_penalty);
      }
      // store inliers and inlier count
      if (n_inliers_count < min_nr_in_shape)
      {
        n_inliers_count = 0;
        inl_stencil.reset (); // release stencil
        hypothesis_valid[cur_batch * iterations_per_batch_ + i] = false;
      }
      
      hypotheses_inliers_stencils[cur_iteration] = inl_stencil;
      hypotheses_inlier_count[cur_iteration] = n_inliers_count;
      
      // Better match ?
      if (n_inliers_count > n_best_inliers_count)
      {
        n_best_inliers_count = n_inliers_count;
        good_coeff = cur_iteration;

        // Compute the k parameter (k=std::log(z)/std::log(1-w^n))
        float w = (float)((float)n_best_inliers_count / (float)nr_remaining_points);
        float p_no_outliers = 1.0f - w;
        p_no_outliers = (std::max) (std::numeric_limits<float>::epsilon (), p_no_outliers);       // Avoid division by -Inf
        p_no_outliers = (std::min) (1.0f - std::numeric_limits<float>::epsilon (), p_no_outliers);   // Avoid division by 0.
        if (p_no_outliers == 1.0f)
          k++;
        else
          k = std::log (1.0f - probability_) / std::log (p_no_outliers);
      }

      //fprintf (stderr, "[pcl_cuda::MultiRandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n",
          //cur_iteration, k, n_inliers_count, n_best_inliers_count);
      // check if we found a valid model

      {
        ScopeTime t("extracmodel");

      if (valid_iterations >= k)
      {
        unsigned int extracted_model = good_coeff;
        //int nr_remaining_points_before_delete = nr_remaining_points;
        bool find_no_better = false;
        nr_remaining_points = sac_model_->deleteIndices (hypotheses_inliers_stencils[extracted_model]);
        //if (nr_remaining_points != nr_remaining_points_before_delete)
        {

          // Compute the k parameter (k=std::log(z)/std::log(1-w^n))
          float w = (float)((float)min_nr_in_shape / (float)nr_remaining_points);
          float p_no_outliers = 1.0f - w;
          p_no_outliers = (std::max) (std::numeric_limits<float>::epsilon (), p_no_outliers);       // Avoid division by -Inf
          p_no_outliers = (std::min) (1.0f - std::numeric_limits<float>::epsilon (), p_no_outliers);   // Avoid division by 0.
          if (p_no_outliers != 1.0f)
          {
            if (std::log (1.0f - probability_) / std::log (p_no_outliers) < valid_iterations) // we won't find a model with min_nr_in_shape points anymore...
              find_no_better = true;
            else
              if (debug_verbosity_level > 1)
                std::cerr << "------->" << std::log (1.0f - probability_) / std::log (p_no_outliers) << "  -vs-  " << valid_iterations << std::endl;
          }
        }

        std::cerr << "found model: " << n_best_inliers_count << ", points remaining: " << nr_remaining_points << " after " << valid_iterations << " / " << cur_iteration << " iterations" << std::endl;

        all_inliers_.push_back (hypotheses_inliers_stencils[extracted_model]);
        all_inlier_counts_.push_back (n_best_inliers_count);
        all_model_centroids_.push_back (centroids [extracted_model]);
        thrust::host_vector<float4> host_coeffs_extracted_model = h [extracted_model / iterations_per_batch_];
        all_model_coefficients_.push_back (host_coeffs_extracted_model [extracted_model % iterations_per_batch_]);
       
        //float4 model_coeff = h[extracted_model/iterations_per_batch_][extracted_model%iterations_per_batch_];
        //std::cerr << "MODEL COEFFICIENTS: " << model_coeff.x << " " << model_coeff.y << " " << model_coeff.z << " " << model_coeff.w << std::endl;

        // so we only get it once:
        hypothesis_valid[extracted_model] = false;

        if (nr_remaining_points < (1.0-min_coverage_percent_) * nr_points)
        {
          std::cerr << "batches: " << cur_batch << ", valid iterations: " << valid_iterations << ", remaining points:" << nr_remaining_points << std::endl;
          return true;
        }
        if (find_no_better)
        {
          std::cerr << "will find no better model! batches: " << cur_batch << ", valid iterations: " << valid_iterations << ", remaining points:" << nr_remaining_points << std::endl;
          return true;
        }

        n_best_inliers_count = 0;
        k = max_batches_ * iterations_per_batch_;
        // go through all other models, invalidating those whose samples are inliers to the best one

        //for (unsigned int b = 0; b <= cur_batch; b++)
        unsigned int b = cur_batch;  
        for (unsigned int j = 0; j < iterations_per_batch_; j++)
        {
          // todo: precheck for very similar models
          // if (h[best_coeff] - h[]) // <---- copies from device! slow!
          //   continue;
          if (!hypothesis_valid[b * iterations_per_batch_ + j])
          {
            //std::cerr << "model " << j << " in batch " << b <<" is an invalid model" << std::endl;
            continue;
          }
          if ((b*iterations_per_batch_ + j) == extracted_model)
          {
            if (debug_verbosity_level > 1)
              std::cerr << "model " << j << " in batch " << b <<" is being extracted..." << std::endl;
            continue;
          }
          if (sac_model_->isSampleInlier (hypotheses_inliers_stencils[extracted_model], h_samples[b], j))
          {
            //std::cerr << "sample point for model " << j << " in batch " << b <<" is inlier to best model " << extracted_model << std::endl;
            hypothesis_valid[b * iterations_per_batch_ + j] = false;
            hypotheses_inlier_count[b*iterations_per_batch_ + j] = 0;
            if (j <= i)
              valid_iterations --;
          }
          else if (j <= i) // if it is not inlier to the best model, we subtract the inliers of the extracted model
          {
            //todo: recompute inliers... / deleteindices
            // ... determine best remaining model
            int old_score = hypotheses_inlier_count[b*iterations_per_batch_ + j]; 
            if (old_score != 0)
            {
      //std::cerr << "inliers for model " << b*iterations_per_batch_ + j << " : " << old_score;
              n_inliers_count = sac_model_->deleteIndices (h[b], j,
                  hypotheses_inliers_stencils[b*iterations_per_batch_ + j], hypotheses_inliers_stencils[extracted_model]);
              hypotheses_inlier_count[b*iterations_per_batch_ + j] = n_inliers_count;
      //std::cerr << " ---> " << hypotheses_inlier_count[b * iterations_per_batch_ + j] << std::endl;
            }
      
            // Better match ?
            if (n_inliers_count > n_best_inliers_count)
            {
              n_best_inliers_count = n_inliers_count;
              good_coeff = b * iterations_per_batch_ + j;

              // Compute the k parameter (k=std::log(z)/std::log(1-w^n))
              float w = (float)((float)n_best_inliers_count / (float)nr_remaining_points);
              float p_no_outliers = 1.0f - w;
              p_no_outliers = (std::max) (std::numeric_limits<float>::epsilon (), p_no_outliers);       // Avoid division by -Inf
              p_no_outliers = (std::min) (1.0f - std::numeric_limits<float>::epsilon (), p_no_outliers);   // Avoid division by 0.
              if (p_no_outliers == 1.0f)
                k++;
              else
                k = std::log (1.0f - probability_) / std::log (p_no_outliers);
            }
            
          }
        }
      }
      }
    }
    }

    // one batch done, go to next
    cur_batch ++;

  } while (cur_iteration < max_batches_ * iterations_per_batch_);

  return (false);
}

template class pcl_cuda::MultiRandomSampleConsensus<pcl_cuda::Device>;
template class pcl_cuda::MultiRandomSampleConsensus<pcl_cuda::Host>;

