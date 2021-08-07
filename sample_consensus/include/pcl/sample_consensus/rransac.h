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

#pragma once

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>

namespace pcl
{
  /** \brief @b RandomizedRandomSampleConsensus represents an implementation of the RRANSAC (Randomized RANdom SAmple 
    * Consensus), as described in "Randomized RANSAC with Td,d test", O. Chum and J. Matas, Proc. British Machine Vision 
    * Conf. (BMVC '02), vol. 2, BMVA, pp. 448-457, 2002.
    * 
    * The algorithm works similar to RANSAC, with one addition: after computing the model coefficients, randomly select a fraction
    * of points. If any of these points do not belong to the model (given a threshold), continue with the next iteration instead
    * of checking all points. This may speed up the finding of the model if the fraction of points to pre-test is chosen well.
    * \note RRANSAC is useful in situations where most of the data samples belong to the model, and a fast outlier rejection algorithm is needed.
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class RandomizedRandomSampleConsensus : public SampleConsensus<PointT>
  {
    using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;

    public:
      using Ptr = shared_ptr<RandomizedRandomSampleConsensus<PointT> >;
      using ConstPtr = shared_ptr<const RandomizedRandomSampleConsensus<PointT> >;

      using SampleConsensus<PointT>::max_iterations_;
      using SampleConsensus<PointT>::threshold_;
      using SampleConsensus<PointT>::iterations_;
      using SampleConsensus<PointT>::sac_model_;
      using SampleConsensus<PointT>::model_;
      using SampleConsensus<PointT>::model_coefficients_;
      using SampleConsensus<PointT>::inliers_;
      using SampleConsensus<PointT>::probability_;

      /** \brief RRANSAC (Randomized RANdom SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        */
      RandomizedRandomSampleConsensus (const SampleConsensusModelPtr &model) 
        : SampleConsensus<PointT> (model)
        , fraction_nr_pretest_ (10.0) // Number of samples to try randomly in percents
      {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
      }

      /** \brief RRANSAC (Randomized RANdom SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        * \param[in] threshold distance to model threshold
        */
      RandomizedRandomSampleConsensus (const SampleConsensusModelPtr &model, double threshold) 
        : SampleConsensus<PointT> (model, threshold)
        , fraction_nr_pretest_ (10.0) // Number of samples to try randomly in percents
      {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
      }

      /** \brief Compute the actual model and find the inliers
        * \param[in] debug_verbosity_level enable/disable on-screen debug information and set the verbosity level
        */
      bool 
      computeModel (int debug_verbosity_level = 0) override;

      /** \brief Set the percentage of points to pre-test.
        * \param[in] nr_pretest percentage of points to pre-test
        */
      inline void 
      setFractionNrPretest (double nr_pretest) { fraction_nr_pretest_ = nr_pretest; }

      /** \brief Get the percentage of points to pre-test. */
      inline double 
      getFractionNrPretest () const { return (fraction_nr_pretest_); }

    private:
      /** \brief Number of samples to randomly pre-test, in percents. */
      double fraction_nr_pretest_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/rransac.hpp>
#endif
