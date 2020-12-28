/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/registration/registration.h>
#include <pcl/memory.h>

namespace pcl {
/** \brief @b NormalDistributionsTransform2D provides an implementation of the
 * Normal Distributions Transform algorithm for scan matching.
 *
 * This implementation is intended to match the definition:
 * Peter Biber and Wolfgang Straßer. The normal distributions transform: A
 * new approach to laser scan matching. In Proceedings of the IEEE In-
 * ternational Conference on Intelligent Robots and Systems (IROS), pages
 * 2743–2748, Las Vegas, USA, October 2003.
 *
 * \author James Crosby
 */
template <typename PointSource, typename PointTarget>
class NormalDistributionsTransform2D : public Registration<PointSource, PointTarget> {
  using PointCloudSource =
      typename Registration<PointSource, PointTarget>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget =
      typename Registration<PointSource, PointTarget>::PointCloudTarget;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

public:
  using Ptr = shared_ptr<NormalDistributionsTransform2D<PointSource, PointTarget>>;
  using ConstPtr =
      shared_ptr<const NormalDistributionsTransform2D<PointSource, PointTarget>>;

  /** \brief Empty constructor. */
  NormalDistributionsTransform2D()
  : Registration<PointSource, PointTarget>()
  , grid_centre_(0, 0)
  , grid_step_(1, 1)
  , grid_extent_(20, 20)
  , newton_lambda_(1, 1, 1)
  {
    reg_name_ = "NormalDistributionsTransform2D";
  }

  /** \brief Empty destructor */
  ~NormalDistributionsTransform2D() override = default;

  /** \brief centre of the ndt grid (target coordinate system)
   * \param centre value to set
   */
  virtual void
  setGridCentre(const Eigen::Vector2f& centre)
  {
    grid_centre_ = centre;
  }

  /** \brief Grid spacing (step) of the NDT grid
   * \param[in] step value to set
   */
  virtual void
  setGridStep(const Eigen::Vector2f& step)
  {
    grid_step_ = step;
  }

  /** \brief NDT Grid extent (in either direction from the grid centre)
   * \param[in] extent value to set
   */
  virtual void
  setGridExtent(const Eigen::Vector2f& extent)
  {
    grid_extent_ = extent;
  }

  /** \brief NDT Newton optimisation step size parameter
   * \param[in] lambda step size: 1 is simple newton optimisation, smaller values may
   * improve convergence
   */
  virtual void
  setOptimizationStepSize(const double& lambda)
  {
    newton_lambda_ = Eigen::Vector3d(lambda, lambda, lambda);
  }

  /** \brief NDT Newton optimisation step size parameter
   * \param[in] lambda step size: (1,1,1) is simple newton optimisation,
   * smaller values may improve convergence, or elements may be set to
   * zero to prevent optimisation over some parameters
   *
   * This overload allows control of updates to the individual (x, y,
   * theta) free parameters in the optimisation. If, for example, theta is
   * believed to be close to the correct value a small value of lambda[2]
   * should be used.
   */
  virtual void
  setOptimizationStepSize(const Eigen::Vector3d& lambda)
  {
    newton_lambda_ = lambda;
  }

protected:
  /** \brief Rigid transformation computation method with initial guess.
   * \param[out] output the transformed input point cloud dataset using the rigid
   * transformation found \param[in] guess the initial guess of the transformation to
   * compute
   */
  void
  computeTransformation(PointCloudSource& output,
                        const Eigen::Matrix4f& guess) override;

  using Registration<PointSource, PointTarget>::reg_name_;
  using Registration<PointSource, PointTarget>::target_;
  using Registration<PointSource, PointTarget>::converged_;
  using Registration<PointSource, PointTarget>::nr_iterations_;
  using Registration<PointSource, PointTarget>::max_iterations_;
  using Registration<PointSource, PointTarget>::transformation_epsilon_;
  using Registration<PointSource, PointTarget>::transformation_rotation_epsilon_;
  using Registration<PointSource, PointTarget>::transformation_;
  using Registration<PointSource, PointTarget>::previous_transformation_;
  using Registration<PointSource, PointTarget>::final_transformation_;
  using Registration<PointSource, PointTarget>::update_visualizer_;
  using Registration<PointSource, PointTarget>::indices_;

  Eigen::Vector2f grid_centre_;
  Eigen::Vector2f grid_step_;
  Eigen::Vector2f grid_extent_;
  Eigen::Vector3d newton_lambda_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

#include <pcl/registration/impl/ndt_2d.hpp>
