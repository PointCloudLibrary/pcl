/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *
 */

#ifndef PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_H_
#define PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_H_

#include <pcl/registration/eigen.h>
#include <pcl/correspondence.h>
#include <pcl/registration/convergence_criteria.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b DefaultConvergenceCriteria represents an instantiation of 
      * ConvergenceCriteria, and implements the following criteria for registration loop
      * evaluation:
      * 
      *  * a maximum number of iterations has been reached
      *  * the transformation (R, t) cannot be further updated (the difference between current and previous is smaller than a threshold)
      *  * the Mean Squared Error (MSE) between the current set of correspondences and the previous one is smaller than some threshold (both relative and absolute tests)
      *
      * \note Convergence is considered reached if ANY of the above criteria are met.
      *
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename Scalar = float>
    class DefaultConvergenceCriteria : public ConvergenceCriteria
    {
      public:
        typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

        /** \brief Empty constructor. 
          * Sets:
          *  * the maximum number of iterations to 1000
          *  * the rotation threshold to 0.256 degrees (0.99999)
          *  * the translation threshold to 0.0003 meters (3e-4^2)
          *  * the MSE relative / absolute thresholds to 0.001% and 1e-12
          *
          * \param[in] iterations a reference to the number of iterations the loop has ran so far
          * \param[in] transform a reference to the current transformation obtained by the transformation evaluation
          * \param[in] correspondences a reference to the current set of point correspondences between source and target
          */
        DefaultConvergenceCriteria (const int &iterations, const Matrix4 &transform, const pcl::Correspondences &correspondences)
          : iterations_ (iterations)
          , transformation_ (transform)
          , correspondences_ (correspondences)
          , correspondences_prev_mse_ (std::numeric_limits<double>::max ())
          , correspondences_cur_mse_ (std::numeric_limits<double>::max ())
          , max_iterations_ (100)                 // 100 iterations
          , rotation_threshold_ (0.99999)         // 0.256 degrees
          , translation_threshold_ (3e-4 * 3e-4)  // 0.0003 meters
          , mse_threshold_relative_ (0.00001)     // 0.001% of the previous MSE (relative error)
          , mse_threshold_absolute_ (1e-12)       // MSE (absolute error)
        {
        }

        /** \brief Set the maximum number of iterations the internal optimization should run for.
          * \param[in] nr_iterations the maximum number of iterations the internal optimization should run for
          */
        inline void 
        setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }

        /** \brief Get the maximum number of iterations the internal optimization should run for, as set by the user. */
        inline int 
        getMaximumIterations () { return (max_iterations_); }

        /** \brief Set the rotation threshold cosine angle (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
          * \param[in] threshold the rotation threshold in order for an optimization to be considered as having converged to the final solution.
          */
        inline void 
        setRotationThreshold (double threshold) { rotation_threshold_ = threshold; }

        /** \brief Get the rotation threshold cosine angle (maximum allowable difference between two consecutive transformations) as set by the user.
          */
        inline double 
        getRotationThreshold () { return (rotation_threshold_); }

        /** \brief Set the translation threshold (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
          * \param[in] threshold the translation threshold in order for an optimization to be considered as having converged to the final solution.
          */
        inline void 
        setTranslationThreshold (double threshold) { translation_threshold_ = threshold; }

        /** \brief Get the rotation threshold cosine angle (maximum allowable difference between two consecutive transformations) as set by the user.
          */
        inline double 
        getTranslationThreshold () { return (translation_threshold_); }

        /** \brief Set the relative MSE between two consecutive sets of correspondences.
          * \param[in] mse_relative the relative MSE threshold
          */
        inline void 
        setRelativeMSE (double mse_relative) { mse_threshold_relative_ = mse_relative; }

        /** \brief Get the relative MSE between two consecutive sets of correspondences. */
        inline double 
        getRelativeMSE () { return (mse_threshold_relative_); }

        /** \brief Set the absolute MSE between two consecutive sets of correspondences.
          * \param[in] mse_absolute the relative MSE threshold
          */
        inline void 
        setAbsoluteMSE (double mse_absolute) { mse_threshold_absolute_ = mse_absolute; }

        /** \brief Get the absolute MSE between two consecutive sets of correspondences. */
        inline double
        getAbsoluteMSE () { return (mse_threshold_absolute_); }


        /** \brief Check if convergence has been reached. */
        virtual bool
        hasConverged ();

      protected:

        /** \brief Calculate the mean squared error (MSE) of the distance for a given set of correspondences. 
          * \param[in] correspondences the given set of correspondences 
          */
        inline double
        calculateMSE (const pcl::Correspondences &correspondences)
        {
          double mse = 0;
          for (size_t i = 0; i < correspondences.size (); ++i)
            mse += correspondences[i].distance;
          mse /= double (correspondences.size ());
          return (mse);
        }

        /** \brief The number of iterations done by the registration loop so far. */
        const int &iterations_;

        /** \brief The current transformation obtained by the transformation estimation method. */
        const Matrix4 &transformation_;

        /** \brief The current set of point correspondences between the source and the target. */
        const pcl::Correspondences &correspondences_;

        /** \brief The MSE for the previous set of correspondences. */
        double correspondences_prev_mse_;
        
        /** \brief The MSE for the current set of correspondences. */
        double correspondences_cur_mse_;

        /** \brief The maximum number of iterations that the registration loop is to be executed. */
        int max_iterations_;

        /** \brief The rotation threshold is the relative rotation between two iterations (as angle cosine). */
        double rotation_threshold_;

        /** \brief The translation threshold is the relative translation between two iterations (0 if no translation). */
        double translation_threshold_;

        /** \brief The relative change from the previous MSE for the current set of correspondences, e.g. .1 means 10% change. */
        double mse_threshold_relative_;

        /** \brief The absolute change from the previous MSE for the current set of correspondences. */
        double mse_threshold_absolute_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     };
  }
}

#include <pcl/registration/impl/default_convergence_criteria.hpp>

#endif    // PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_H_

