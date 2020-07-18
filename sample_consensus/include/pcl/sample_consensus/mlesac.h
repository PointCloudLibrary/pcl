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
#include <pcl/pcl_base.h>

namespace pcl
{
  /** \brief @b MaximumLikelihoodSampleConsensus represents an implementation of the MLESAC (Maximum Likelihood 
    * Estimator SAmple Consensus) algorithm, as described in: "MLESAC: A new robust estimator with application to 
    * estimating image geometry", P.H.S. Torr and A. Zisserman, Computer Vision and Image Understanding, vol 78, 2000.
    * \note MLESAC is useful in situations where most of the data samples belong to the model, and a fast outlier rejection algorithm is needed.
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class MaximumLikelihoodSampleConsensus : public SampleConsensus<PointT>
  {
    using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;
    using PointCloudConstPtr = typename SampleConsensusModel<PointT>::PointCloudConstPtr; 

    public:
      using Ptr = shared_ptr<MaximumLikelihoodSampleConsensus<PointT> >;
      using ConstPtr = shared_ptr<const MaximumLikelihoodSampleConsensus<PointT> >;

      using SampleConsensus<PointT>::max_iterations_;
      using SampleConsensus<PointT>::threshold_;
      using SampleConsensus<PointT>::iterations_;
      using SampleConsensus<PointT>::sac_model_;
      using SampleConsensus<PointT>::model_;
      using SampleConsensus<PointT>::model_coefficients_;
      using SampleConsensus<PointT>::inliers_;
      using SampleConsensus<PointT>::probability_;

      /** \brief MLESAC (Maximum Likelihood Estimator SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        */
      MaximumLikelihoodSampleConsensus (const SampleConsensusModelPtr &model) : 
        SampleConsensus<PointT> (model),
        iterations_EM_ (3),      // Max number of EM (Expectation Maximization) iterations
        sigma_ (0)
      {
        max_iterations_ = 10000; // Maximum number of trials before we give up.
      }

      /** \brief MLESAC (Maximum Likelihood Estimator SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        * \param[in] threshold distance to model threshold
        */
      MaximumLikelihoodSampleConsensus (const SampleConsensusModelPtr &model, double threshold) : 
        SampleConsensus<PointT> (model, threshold),
        iterations_EM_ (3),      // Max number of EM (Expectation Maximization) iterations
        sigma_ (0)
      {
        max_iterations_ = 10000; // Maximum number of trials before we give up.
      }

      /** \brief Compute the actual model and find the inliers
        * \param[in] debug_verbosity_level enable/disable on-screen debug information and set the verbosity level
        */
      bool 
      computeModel (int debug_verbosity_level = 0) override;

      /** \brief Set the number of EM iterations.
        * \param[in] iterations the number of EM iterations
        */
      inline void 
      setEMIterations (int iterations) { iterations_EM_ = iterations; }

      /** \brief Get the number of EM iterations. */
      inline int 
      getEMIterations () const { return (iterations_EM_); }


    protected:
      /** \brief Compute the median absolute deviation:
        * \f[
        * MAD = \sigma * median_i (| Xi - median_j(Xj) |)
        * \f]
        * \note Sigma needs to be chosen carefully (a good starting sigma value is 1.4826)
        * \param[in] cloud the point cloud data message
        * \param[in] indices the set of point indices to use
        * \param[in] sigma the sigma value
        */
      double 
      computeMedianAbsoluteDeviation (const PointCloudConstPtr &cloud, 
                                      const IndicesPtr &indices,
                                      double sigma) const;

      /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given set of points
        * \param[in] cloud the point cloud message
        * \param[in] indices the set of point indices to use
        * \param[out] min_p the resultant minimum bounding box coordinates
        * \param[out] max_p the resultant maximum bounding box coordinates
        */
      void 
      getMinMax (const PointCloudConstPtr &cloud, 
                 const IndicesPtr &indices,
                 Eigen::Vector4f &min_p, 
                 Eigen::Vector4f &max_p) const;

      /** \brief Compute the median value of a 3D point cloud using a given set point indices and return it as a Point32.
        * \param[in] cloud the point cloud data message
        * \param[in] indices the point indices
        * \param[out] median the resultant median value
        */
      void 
      computeMedian (const PointCloudConstPtr &cloud, 
                     const IndicesPtr &indices,
                     Eigen::Vector4f &median) const;

    private:
      /** \brief Maximum number of EM (Expectation Maximization) iterations. */
      int iterations_EM_;
      /** \brief The MLESAC sigma parameter. */
      double sigma_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/mlesac.hpp>
#endif
