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

#pragma once

#include <pcl_cuda/sample_consensus/sac.h>
#include <pcl_cuda/sample_consensus/sac_model.h>

namespace pcl_cuda
{

  template <template <typename> class Storage>
  class MEstimatorSampleConsensus : public SampleConsensus<Storage>
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

    using SampleConsensusModelPtr = typename SampleConsensusModel<Storage>::Ptr;
    using Coefficients = typename SampleConsensusModel<Storage>::Coefficients;
    using Indices = typename SampleConsensusModel<Storage>::Indices;
    using Hypotheses = typename SampleConsensusModel<Storage>::Hypotheses;

    public:
      /** \brief MEstimatorSampleConsensus main constructor
        * \param model a Sample Consensus model
        */
      MEstimatorSampleConsensus (const SampleConsensusModelPtr &model) : 
        SampleConsensus<Storage> (model)
      {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
      }

      /** \brief RANSAC (RAndom SAmple Consensus) main constructor
        * \param model a Sample Consensus model
        * \param threshold distance to model threshold
        */
      MEstimatorSampleConsensus (const SampleConsensusModelPtr &model, float threshold) : 
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
  };
}
