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

#include <pcl/sample_consensus/sac_model_line.h>

namespace pcl
{
  /** \brief SampleConsensusModelParallelLine defines a model for 3D line segmentation using additional angular
    * constraints.
    *
    * Checking for inliers will not only involve a "distance to model" criterion, but also an additional "maximum
    * angular deviation" between the line's direction and a user-specified axis.
    *
    * The model coefficients are defined as:
    *   - \b point_on_line.x  : the X coordinate of a point on the line
    *   - \b point_on_line.y  : the Y coordinate of a point on the line
    *   - \b point_on_line.z  : the Z coordinate of a point on the line
    *   - \b line_direction.x : the X coordinate of a line's direction
    *   - \b line_direction.y : the Y coordinate of a line's direction
    *   - \b line_direction.z : the Z coordinate of a line's direction
    *
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelParallelLine : public SampleConsensusModelLine<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;

      using PointCloud = typename SampleConsensusModelLine<PointT>::PointCloud;
      using PointCloudPtr = typename SampleConsensusModelLine<PointT>::PointCloudPtr;
      using PointCloudConstPtr = typename SampleConsensusModelLine<PointT>::PointCloudConstPtr;

      using Ptr = shared_ptr<SampleConsensusModelParallelLine<PointT> >;
      using ConstPtr = shared_ptr<const SampleConsensusModelParallelLine<PointT>>;

      /** \brief Constructor for base SampleConsensusModelParallelLine.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelParallelLine (const PointCloudConstPtr &cloud,
                                        bool random = false)
        : SampleConsensusModelLine<PointT> (cloud, random)
        , axis_ (Eigen::Vector3f::Zero ())
        , eps_angle_ (0.0)
      {
        model_name_ = "SampleConsensusModelParallelLine";
        sample_size_ = 2;
        model_size_ = 6;
      }

      /** \brief Constructor for base SampleConsensusModelParallelLine.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelParallelLine (const PointCloudConstPtr &cloud,
                                        const Indices &indices,
                                        bool random = false)
        : SampleConsensusModelLine<PointT> (cloud, indices, random)
        , axis_ (Eigen::Vector3f::Zero ())
        , eps_angle_ (0.0)
      {
        model_name_ = "SampleConsensusModelParallelLine";
        sample_size_ = 2;
        model_size_ = 6;
      }

      /** \brief Empty destructor */
      ~SampleConsensusModelParallelLine () override = default;

      /** \brief Set the axis along which we need to search for a line.
        * \param[in] ax the axis along which we need to search for a line
        */
      inline void
      setAxis (const Eigen::Vector3f &ax) { axis_ = ax; axis_.normalize (); }

      /** \brief Get the axis along which we need to search for a line. */
      inline Eigen::Vector3f
      getAxis () const { return (axis_); }

      /** \brief Set the angle epsilon (delta) threshold.
        * \param[in] ea the maximum allowed difference between the line direction and the given axis (in radians).
        */
      inline void
      setEpsAngle (const double ea) { eps_angle_ = ea; }

      /** \brief Get the angle epsilon (delta) threshold (in radians). */
      inline double getEpsAngle () const { return (eps_angle_); }

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the coefficients of a line model that we need to compute distances to
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

      /** \brief Compute all squared distances from the cloud data to a given line model.
        * \param[in] model_coefficients the coefficients of a line model that we need to compute distances to
        * \param[out] distances the resultant estimated squared distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const override;

      /** \brief Return a unique id for this model (SACMODEL_PARALLEL_LINE). */
      inline pcl::SacModel
      getModelType () const override { return (SACMODEL_PARALLEL_LINE); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      bool
      isModelValid (const Eigen::VectorXf &model_coefficients) const override;

      /** \brief The axis along which we need to search for a line. */
      Eigen::Vector3f axis_;

      /** \brief The maximum allowed difference between the line direction and the given axis. */
      double eps_angle_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#endif
