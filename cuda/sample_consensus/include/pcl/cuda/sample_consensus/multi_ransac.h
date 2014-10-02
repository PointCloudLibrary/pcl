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

#ifndef PCL_CUDA_SAMPLE_CONSENSUS_RANSAC_H_
#define PCL_CUDA_SAMPLE_CONSENSUS_RANSAC_H_

#include <pcl/cuda/sample_consensus/sac.h>
#include <pcl/cuda/sample_consensus/sac_model.h>

namespace pcl
{
  namespace cuda
  {
    /** \brief @b RandomSampleConsensus represents an implementation of the
      * RANSAC (RAndom SAmple Consensus) algorithm, as described in: "Random
      * Sample Consensus: A Paradigm for Model Fitting with Applications to Image
      * Analysis and Automated Cartography", Martin A. Fischler and Robert C. Bolles, 
      * Comm. Of the ACM 24: 381–395, June 1981.
      * \author Radu Bogdan Rusu
      */
    template <template <typename> class Storage>
    class MultiRandomSampleConsensus : public SampleConsensus<Storage>
    {
      using SampleConsensus<Storage>::max_iterations_;
      using SampleConsensus<Storage>::threshold_;
      using SampleConsensus<Storage>::iterations_;
      using SampleConsensus<Storage>::sac_model_;
      using SampleConsensus<Storage>::model_;
      using SampleConsensus<Storage>::model_coefficients_;
      using SampleConsensus<Storage>::inliers_;
      using SampleConsensus<Storage>::inliers_stencil_;
      using SampleConsensus<Storage>::probability_;

      typedef typename SampleConsensusModel<Storage>::Ptr SampleConsensusModelPtr;
      typedef typename SampleConsensusModel<Storage>::Coefficients Coefficients;
      typedef typename SampleConsensusModel<Storage>::Hypotheses Hypotheses;

      typedef typename SampleConsensusModel<Storage>::Indices Indices;
      typedef typename SampleConsensusModel<Storage>::IndicesPtr IndicesPtr;
      typedef typename SampleConsensusModel<Storage>::IndicesConstPtr IndicesConstPtr;

      public:
        /** \brief RANSAC (RAndom SAmple Consensus) main constructor
          * \param model a Sample Consensus model
          */
        MultiRandomSampleConsensus (const SampleConsensusModelPtr &model) : 
          SampleConsensus<Storage> (model),
          min_coverage_percent_ (0.9),
          max_batches_ (5),
          iterations_per_batch_ (1000)
        {
          // Maximum number of trials before we give up.
          max_iterations_ = 10000;
        }

        /** \brief RANSAC (RAndom SAmple Consensus) main constructor
          * \param model a Sample Consensus model
          * \param threshold distance to model threshold
          */
        MultiRandomSampleConsensus (const SampleConsensusModelPtr &model, double threshold) : 
          SampleConsensus<Storage> (model, threshold)
        {
          // Maximum number of trials before we give up.
          max_iterations_ = 10000;
        }

        /** \brief Compute the actual model and find the inliers
          * \param debug_verbosity_level enable/disable on-screen debug
          * information and set the verbosity level
          */
        bool 
        computeModel (int debug_verbosity_level = 0);

        /** \brief how much (in percent) of the point cloud should be covered?
         *  If it is not possible to find enough planes, it will stop according to the regular ransac criteria
         */
        void
        setMinimumCoverage (float percent)
        {
          min_coverage_percent_ = percent;
        }
          
        /** \brief Sets the maximum number of batches that should be processed.
         *  Every Batch computes up to iterations_per_batch_ models and verifies them.
         *  If planes with a sufficiently high total inlier count are found earlier, the
         *  actual number of batch runs might be lower.
         */
        void
        setMaximumBatches (int max_batches)
        {
          max_batches_ = max_batches_;
        }

        /** \brief Sets the maximum number of batches that should be processed.
         *  Every Batch computes up to max_iterations_ models and verifies them.
         *  If planes with a sufficiently high total inlier count are found earlier, the
         *  actual number of batch runs might be lower.
         */
        void
        setIerationsPerBatch(int iterations_per_batch)
        {
          iterations_per_batch_ = iterations_per_batch;
        }

        inline std::vector<IndicesPtr>
        getAllInliers () { return all_inliers_; }

        inline std::vector<int>
        getAllInlierCounts () { return all_inlier_counts_; }

        /** \brief Return the model coefficients of the best model found so far. 
          */
        inline std::vector<float4>
        getAllModelCoefficients () 
        { 
          return all_model_coefficients_; 
        }

        /** \brief Return the model coefficients of the best model found so far. 
          */
        inline std::vector<float3>
        getAllModelCentroids () 
        { 
          return all_model_centroids_; 
        }

      private:
        float min_coverage_percent_;
        unsigned int max_batches_;
        unsigned int iterations_per_batch_;

        /** \brief The vector of the centroids of our models computed directly from the models found. */
        std::vector<float3> all_model_centroids_;

        /** \brief The vector of coefficients of our models computed directly from the models found. */
        std::vector<float4> all_model_coefficients_;

        std::vector<IndicesPtr> all_inliers_;
        std::vector<int> all_inlier_counts_;
    };

  } // namespace
} // namespace

#endif  //#ifndef PCL_CUDA_SAMPLE_CONSENSUS_RANSAC_H_

