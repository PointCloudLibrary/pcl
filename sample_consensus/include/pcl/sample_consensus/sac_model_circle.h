/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifdef __SSE__
#include <xmmintrin.h> // for __m128
#endif // ifdef __SSE__
#ifdef __AVX__
#include <immintrin.h> // for __m256
#endif // ifdef __AVX__

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

namespace pcl
{
  /** \brief SampleConsensusModelCircle2D defines a model for 2D circle segmentation on the X-Y plane.
    *
    * The model coefficients are defined as:
    *   - \b center.x : the X coordinate of the circle's center
    *   - \b center.y : the Y coordinate of the circle's center
    *   - \b radius   : the circle's radius
    * 
    * \author Radu B. Rusu
    * \ingroup sample_consensus
   */
  template <typename PointT>
  class SampleConsensusModelCircle2D : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::radius_min_;
      using SampleConsensusModel<PointT>::radius_max_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;

      using PointCloud = typename SampleConsensusModel<PointT>::PointCloud;
      using PointCloudPtr = typename SampleConsensusModel<PointT>::PointCloudPtr;
      using PointCloudConstPtr = typename SampleConsensusModel<PointT>::PointCloudConstPtr;

      using Ptr = shared_ptr<SampleConsensusModelCircle2D<PointT> >;
      using ConstPtr = shared_ptr<const SampleConsensusModelCircle2D<PointT>>;

      /** \brief Constructor for base SampleConsensusModelCircle2D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCircle2D (const PointCloudConstPtr &cloud, bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random)
      {
        model_name_ = "SampleConsensusModelCircle2D";
        sample_size_ = 3;
        model_size_ = 3;
      }

      /** \brief Constructor for base SampleConsensusModelCircle2D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCircle2D (const PointCloudConstPtr &cloud, 
                                    const Indices &indices,
                                    bool random = false)
        : SampleConsensusModel<PointT> (cloud, indices, random)
      {
        model_name_ = "SampleConsensusModelCircle2D";
        sample_size_ = 3;
        model_size_ = 3;
      }

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      SampleConsensusModelCircle2D (const SampleConsensusModelCircle2D &source) :
        SampleConsensusModel<PointT> ()
      {
        *this = source;
        model_name_ = "SampleConsensusModelCircle2D";
      }
      
      /** \brief Empty destructor */
      ~SampleConsensusModelCircle2D () override = default;

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      inline SampleConsensusModelCircle2D&
      operator = (const SampleConsensusModelCircle2D &source)
      {
        SampleConsensusModel<PointT>::operator=(source);
        return (*this);
      }

      /** \brief Check whether the given index samples can form a valid 2D circle model, compute the model coefficients
        * from these samples and store them in model_coefficients. The circle coefficients are: x, y, R.
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const Indices &samples,
                                Eigen::VectorXf &model_coefficients) const override;

      /** \brief Compute all distances from the cloud data to a given 2D circle model.
        * \param[in] model_coefficients the coefficients of a 2D circle model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const override;

      /** \brief Compute all distances from the cloud data to a given 2D circle model.
        * \param[in] model_coefficients the coefficients of a 2D circle model that we need to compute distances to
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void 
      selectWithinDistance (const Eigen::VectorXf &model_coefficients, 
                            const double threshold, 
                            Indices &inliers) override;

      /** \brief Count all the points which respect the given model coefficients as inliers. 
        * 
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      std::size_t
      countWithinDistance (const Eigen::VectorXf &model_coefficients,
                           const double threshold) const override;

       /** \brief Recompute the 2d circle coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the 2d circle model after refinement (e.g. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void
      optimizeModelCoefficients (const Indices &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients) const override;

      /** \brief Create a new point cloud with inliers projected onto the 2d circle model.
        * \param[in] inliers the data inliers that we want to project on the 2d circle model
        * \param[in] model_coefficients the coefficients of a 2d circle model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        */
      void
      projectPoints (const Indices &inliers,
                     const Eigen::VectorXf &model_coefficients,
                     PointCloud &projected_points,
                     bool copy_data_fields = true) const override;

      /** \brief Verify whether a subset of indices verifies the given 2d circle model coefficients.
        * \param[in] indices the data indices that need to be tested against the 2d circle model
        * \param[in] model_coefficients the 2d circle model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool
      doSamplesVerifyModel (const std::set<index_t> &indices,
                            const Eigen::VectorXf &model_coefficients,
                            const double threshold) const override;

      /** \brief Return a unique id for this model (SACMODEL_CIRCLE2D). */
      inline pcl::SacModel 
      getModelType () const override { return (SACMODEL_CIRCLE2D); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      bool
      isModelValid (const Eigen::VectorXf &model_coefficients) const override;

      /** \brief Check if a sample of indices results in a good sample of points indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood(const Indices &samples) const override;

      /** This implementation uses no SIMD instructions. It is not intended for normal use.
        * See countWithinDistance which automatically uses the fastest implementation.
        */
      std::size_t
      countWithinDistanceStandard (const Eigen::VectorXf &model_coefficients,
                                   const double threshold,
                                   std::size_t i = 0) const;

#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
      /** This implementation uses SSE, SSE2, and SSE4.1 instructions. It is not intended for normal use.
        * See countWithinDistance which automatically uses the fastest implementation.
        */
      std::size_t
      countWithinDistanceSSE (const Eigen::VectorXf &model_coefficients,
                              const double threshold,
                              std::size_t i = 0) const;
#endif

#if defined (__AVX__) && defined (__AVX2__)
      /** This implementation uses AVX and AVX2 instructions. It is not intended for normal use.
        * See countWithinDistance which automatically uses the fastest implementation.
        */
      std::size_t
      countWithinDistanceAVX (const Eigen::VectorXf &model_coefficients,
                              const double threshold,
                              std::size_t i = 0) const;
#endif

    private:
      /** \brief Functor for the optimization function */
      struct OptimizationFunctor : pcl::Functor<float>
      {
        /** \brief Functor constructor
          * \param[in] indices the indices of data points to evaluate
          * \param[in] estimator pointer to the estimator object
          */
        OptimizationFunctor (const pcl::SampleConsensusModelCircle2D<PointT> *model, const Indices& indices) :
          pcl::Functor<float> (indices.size ()), model_ (model), indices_ (indices) {}

        /** Cost function to be minimized
          * \param[in] x the variables array
          * \param[out] fvec the resultant functions evaluations
          * \return 0
          */
        int 
        operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
          for (int i = 0; i < values (); ++i)
          {
            // Compute the difference between the center of the circle and the datapoint X_i
            float xt = (*model_->input_)[indices_[i]].x - x[0];
            float yt = (*model_->input_)[indices_[i]].y - x[1];

            // g = sqrt ((x-a)^2 + (y-b)^2) - R
            fvec[i] = std::sqrt (xt * xt + yt * yt) - x[2];
          }
          return (0);
        }

        const pcl::SampleConsensusModelCircle2D<PointT> *model_;
        const Indices &indices_;
      };

#ifdef __AVX__
      inline __m256 sqr_dist8 (const std::size_t i, const __m256 a_vec, const __m256 b_vec) const;
#endif

#ifdef __SSE__
      inline __m128 sqr_dist4 (const std::size_t i, const __m128 a_vec, const __m128 b_vec) const;
#endif
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#endif
