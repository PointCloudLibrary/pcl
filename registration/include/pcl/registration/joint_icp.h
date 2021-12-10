/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

// PCL includes
#include <pcl/registration/icp.h>
namespace pcl {
/** \brief @b JointIterativeClosestPoint extends ICP to multiple frames which
 *  share the same transform. This is particularly useful when solving for
 *  camera extrinsics using multiple observations. When given a single pair of
 *  clouds, this reduces to vanilla ICP.
 *
 * \author Stephen Miller
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class JointIterativeClosestPoint
: public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
  using PointCloudSource = typename IterativeClosestPoint<PointSource,
                                                          PointTarget,
                                                          Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename IterativeClosestPoint<PointSource,
                                                          PointTarget,
                                                          Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using KdTree = pcl::search::KdTree<PointTarget>;
  using KdTreePtr = typename KdTree::Ptr;

  using KdTreeReciprocal = pcl::search::KdTree<PointSource>;
  using KdTreeReciprocalPtr = typename KdTree::Ptr;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using Ptr = shared_ptr<JointIterativeClosestPoint<PointSource, PointTarget, Scalar>>;
  using ConstPtr =
      shared_ptr<const JointIterativeClosestPoint<PointSource, PointTarget, Scalar>>;

  using CorrespondenceEstimation =
      pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>;
  using CorrespondenceEstimationPtr = typename CorrespondenceEstimation::Ptr;
  using CorrespondenceEstimationConstPtr = typename CorrespondenceEstimation::ConstPtr;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::getClassName;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::input_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::indices_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::nr_iterations_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::max_iterations_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      previous_transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::final_transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      transformation_epsilon_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::converged_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::inlier_threshold_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      min_number_correspondences_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::update_visualizer_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      euclidean_fitness_epsilon_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondences_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      transformation_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      correspondence_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      correspondence_rejectors_;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      use_reciprocal_correspondence_;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::convergence_criteria_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::source_has_normals_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_has_normals_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_source_blob_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_target_blob_;

  using Matrix4 =
      typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Empty constructor. */
  JointIterativeClosestPoint()
  {
    IterativeClosestPoint<PointSource, PointTarget, Scalar>();
    reg_name_ = "JointIterativeClosestPoint";
  };

  /** \brief Empty destructor */
  ~JointIterativeClosestPoint() {}

  /** \brief Provide a pointer to the input source
   * (e.g., the point cloud that we want to align to the target)
   */
  void
  setInputSource(const PointCloudSourceConstPtr& /*cloud*/) override
  {
    PCL_WARN("[pcl::%s::setInputSource] Warning; JointIterativeClosestPoint expects "
             "multiple clouds. Please use addInputSource.\n",
             getClassName().c_str());
    return;
  }

  /** \brief Add a source cloud to the joint solver
   *
   * \param[in] cloud source cloud
   */
  inline void
  addInputSource(const PointCloudSourceConstPtr& cloud)
  {
    // Set the parent InputSource, just to get all cached values (e.g. the existence of
    // normals).
    if (sources_.empty())
      IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource(cloud);
    sources_.push_back(cloud);
  }

  /** \brief Provide a pointer to the input target
   * (e.g., the point cloud that we want to align to the target)
   */
  void
  setInputTarget(const PointCloudTargetConstPtr& /*cloud*/) override
  {
    PCL_WARN("[pcl::%s::setInputTarget] Warning; JointIterativeClosestPoint expects "
             "multiple clouds. Please use addInputTarget.\n",
             getClassName().c_str());
    return;
  }

  /** \brief Add a target cloud to the joint solver
   *
   * \param[in] cloud target cloud
   */
  inline void
  addInputTarget(const PointCloudTargetConstPtr& cloud)
  {
    // Set the parent InputTarget, just to get all cached values (e.g. the existence of
    // normals).
    if (targets_.empty())
      IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
    targets_.push_back(cloud);
  }

  /** \brief Add a manual correspondence estimator
   * If you choose to do this, you must add one for each
   * input source / target pair. They do not need to have trees
   * or input clouds set ahead of time.
   *
   * \param[in] ce Correspondence estimation
   */
  inline void
  addCorrespondenceEstimation(CorrespondenceEstimationPtr ce)
  {
    correspondence_estimations_.push_back(ce);
  }

  /** \brief Reset my list of input sources
   */
  inline void
  clearInputSources()
  {
    sources_.clear();
  }

  /** \brief Reset my list of input targets
   */
  inline void
  clearInputTargets()
  {
    targets_.clear();
  }

  /** \brief Reset my list of correspondence estimation methods.
   */
  inline void
  clearCorrespondenceEstimations()
  {
    correspondence_estimations_.clear();
  }

protected:
  /** \brief Rigid transformation computation method  with initial guess.
   * \param output the transformed input point cloud dataset using the rigid
   * transformation found \param guess the initial guess of the transformation to
   * compute
   */
  void
  computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  /** \brief Looks at the Estimators and Rejectors and determines whether their
   * blob-setter methods need to be called */
  void
  determineRequiredBlobData() override;

  std::vector<PointCloudSourceConstPtr> sources_;
  std::vector<PointCloudTargetConstPtr> targets_;
  std::vector<CorrespondenceEstimationPtr> correspondence_estimations_;
};

} // namespace pcl

#include <pcl/registration/impl/joint_icp.hpp>
