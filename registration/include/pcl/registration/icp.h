/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

// PCL includes
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <pcl/memory.h> // for dynamic_pointer_cast, pcl::make_shared, shared_ptr

namespace pcl {
/** \brief @b IterativeClosestPoint provides a base implementation of the Iterative
 * Closest Point algorithm. The transformation is estimated based on Singular Value
 * Decomposition (SVD).
 *
 * The algorithm has several termination criteria:
 *
 * <ol>
 * <li>Number of iterations has reached the maximum user imposed number of iterations
 * (via \ref setMaximumIterations)</li> <li>The epsilon (difference) between the
 * previous transformation and the current estimated transformation is smaller than an
 * user imposed value (via \ref setTransformationEpsilon)</li> <li>The sum of Euclidean
 * squared errors is smaller than a user defined threshold (via \ref
 * setEuclideanFitnessEpsilon)</li>
 * </ol>
 *
 *
 * Usage example:
 * \code
 * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
 * // Set the input source and target
 * icp.setInputSource (cloud_source);
 * icp.setInputTarget (cloud_target);
 *
 * // Set the max correspondence distance to 5cm (e.g., correspondences with higher
 * // distances will be ignored)
 * icp.setMaxCorrespondenceDistance (0.05);
 * // Set the maximum number of iterations (criterion 1)
 * icp.setMaximumIterations (50);
 * // Set the transformation epsilon (criterion 2)
 * icp.setTransformationEpsilon (1e-8);
 * // Set the euclidean distance difference epsilon (criterion 3)
 * icp.setEuclideanFitnessEpsilon (1);
 *
 * // Perform the alignment
 * icp.align (cloud_source_registered);
 *
 * // Obtain the transformation that aligned cloud_source to cloud_source_registered
 * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
 * \endcode
 *
 * \author Radu B. Rusu, Michael Dixon
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPoint : public Registration<PointSource, PointTarget, Scalar> {
public:
  using PointCloudSource =
      typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget =
      typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using Ptr = shared_ptr<IterativeClosestPoint<PointSource, PointTarget, Scalar>>;
  using ConstPtr =
      shared_ptr<const IterativeClosestPoint<PointSource, PointTarget, Scalar>>;

  using Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using Registration<PointSource, PointTarget, Scalar>::getClassName;
  using Registration<PointSource, PointTarget, Scalar>::input_;
  using Registration<PointSource, PointTarget, Scalar>::indices_;
  using Registration<PointSource, PointTarget, Scalar>::target_;
  using Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using Registration<PointSource, PointTarget, Scalar>::max_iterations_;
  using Registration<PointSource, PointTarget, Scalar>::previous_transformation_;
  using Registration<PointSource, PointTarget, Scalar>::final_transformation_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
  using Registration<PointSource, PointTarget, Scalar>::
      transformation_rotation_epsilon_;
  using Registration<PointSource, PointTarget, Scalar>::converged_;
  using Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
  using Registration<PointSource, PointTarget, Scalar>::inlier_threshold_;
  using Registration<PointSource, PointTarget, Scalar>::min_number_correspondences_;
  using Registration<PointSource, PointTarget, Scalar>::update_visualizer_;
  using Registration<PointSource, PointTarget, Scalar>::euclidean_fitness_epsilon_;
  using Registration<PointSource, PointTarget, Scalar>::correspondences_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_estimation_;
  using Registration<PointSource, PointTarget, Scalar>::correspondence_estimation_;
  using Registration<PointSource, PointTarget, Scalar>::correspondence_rejectors_;

  typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr
      convergence_criteria_;
  using Matrix4 = typename Registration<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Empty constructor. */
  IterativeClosestPoint()
  : x_idx_offset_(0)
  , y_idx_offset_(0)
  , z_idx_offset_(0)
  , nx_idx_offset_(0)
  , ny_idx_offset_(0)
  , nz_idx_offset_(0)
  , use_reciprocal_correspondence_(false)
  , source_has_normals_(false)
  , target_has_normals_(false)
  {
    reg_name_ = "IterativeClosestPoint";
    transformation_estimation_.reset(
        new pcl::registration::
            TransformationEstimationSVD<PointSource, PointTarget, Scalar>());
    correspondence_estimation_.reset(
        new pcl::registration::
            CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
    convergence_criteria_.reset(
        new pcl::registration::DefaultConvergenceCriteria<Scalar>(
            nr_iterations_, transformation_, *correspondences_));
  };

  /**
   * \brief Due to `convergence_criteria_` holding references to the class members,
   * it is tricky to correctly implement its copy and move operations correctly. This
   * can result in subtle bugs and to prevent them, these operations for ICP have
   * been disabled.
   *
   * \todo: remove deleted ctors and assignments operations after resolving the issue
   */
  IterativeClosestPoint(const IterativeClosestPoint&) = delete;
  IterativeClosestPoint(IterativeClosestPoint&&) = delete;
  IterativeClosestPoint&
  operator=(const IterativeClosestPoint&) = delete;
  IterativeClosestPoint&
  operator=(IterativeClosestPoint&&) = delete;

  /** \brief Empty destructor */
  ~IterativeClosestPoint() override = default;

  /** \brief Returns a pointer to the DefaultConvergenceCriteria used by the
   * IterativeClosestPoint class. This allows to check the convergence state after the
   * align() method as well as to configure DefaultConvergenceCriteria's parameters not
   * available through the ICP API before the align() method is called. Please note that
   * the align method sets max_iterations_, euclidean_fitness_epsilon_ and
   * transformation_epsilon_ and therefore overrides the default / set values of the
   * DefaultConvergenceCriteria instance. \return Pointer to the IterativeClosestPoint's
   * DefaultConvergenceCriteria.
   */
  inline typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr
  getConvergeCriteria()
  {
    return convergence_criteria_;
  }

  /** \brief Provide a pointer to the input source
   * (e.g., the point cloud that we want to align to the target)
   *
   * \param[in] cloud the input point cloud source
   */
  void
  setInputSource(const PointCloudSourceConstPtr& cloud) override
  {
    Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
    const auto fields = pcl::getFields<PointSource>();
    source_has_normals_ = false;
    for (const auto& field : fields) {
      if (field.name == "x")
        x_idx_offset_ = field.offset;
      else if (field.name == "y")
        y_idx_offset_ = field.offset;
      else if (field.name == "z")
        z_idx_offset_ = field.offset;
      else if (field.name == "normal_x") {
        source_has_normals_ = true;
        nx_idx_offset_ = field.offset;
      }
      else if (field.name == "normal_y") {
        source_has_normals_ = true;
        ny_idx_offset_ = field.offset;
      }
      else if (field.name == "normal_z") {
        source_has_normals_ = true;
        nz_idx_offset_ = field.offset;
      }
    }
  }

  /** \brief Provide a pointer to the input target
   * (e.g., the point cloud that we want to align the input source to)
   *
   * \param[in] cloud the input point cloud target
   */
  void
  setInputTarget(const PointCloudTargetConstPtr& cloud) override
  {
    Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
    const auto fields = pcl::getFields<PointSource>();
    target_has_normals_ = false;
    for (const auto& field : fields) {
      if (field.name == "normal_x" || field.name == "normal_y" ||
          field.name == "normal_z") {
        target_has_normals_ = true;
        break;
      }
    }
  }

  /** \brief Set whether to use reciprocal correspondence or not
   *
   * \param[in] use_reciprocal_correspondence whether to use reciprocal correspondence
   * or not
   */
  inline void
  setUseReciprocalCorrespondences(bool use_reciprocal_correspondence)
  {
    use_reciprocal_correspondence_ = use_reciprocal_correspondence;
  }

  /** \brief Obtain whether reciprocal correspondence are used or not */
  inline bool
  getUseReciprocalCorrespondences() const
  {
    return (use_reciprocal_correspondence_);
  }

protected:
  /** \brief Apply a rigid transform to a given dataset. Here we check whether
   * the dataset has surface normals in addition to XYZ, and rotate normals as well.
   * \param[in] input the input point cloud
   * \param[out] output the resultant output point cloud
   * \param[in] transform a 4x4 rigid transformation
   * \note Can be used with cloud_in equal to cloud_out
   */
  virtual void
  transformCloud(const PointCloudSource& input,
                 PointCloudSource& output,
                 const Matrix4& transform);

  /** \brief Rigid transformation computation method  with initial guess.
   * \param output the transformed input point cloud dataset using the rigid
   * transformation found \param guess the initial guess of the transformation to
   * compute
   */
  void
  computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  /** \brief Looks at the Estimators and Rejectors and determines whether their
   * blob-setter methods need to be called */
  virtual void
  determineRequiredBlobData();

  /** \brief XYZ fields offset. */
  std::size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

  /** \brief Normal fields offset. */
  std::size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

  /** \brief The correspondence type used for correspondence estimation. */
  bool use_reciprocal_correspondence_;

  /** \brief Internal check whether source dataset has normals or not. */
  bool source_has_normals_;
  /** \brief Internal check whether target dataset has normals or not. */
  bool target_has_normals_;

  /** \brief Checks for whether estimators and rejectors need various data */
  bool need_source_blob_, need_target_blob_;
};

/** \brief @b IterativeClosestPointWithNormals is a special case of
 * IterativeClosestPoint, that uses a transformation estimated based on
 * Point to Plane distances by default.
 *
 * By default, this implementation uses the traditional point to plane objective
 * and computes point to plane distances using the normals of the target point
 * cloud. It also provides the option (through setUseSymmetricObjective) of
 * using the symmetric objective function of [Rusinkiewicz 2019]. This objective
 * uses the normals of both the source and target point cloud and has a similar
 * computational cost to the traditional point to plane objective while also
 * offering improved convergence speed and a wider basin of convergence.
 *
 * Note that this implementation not demean the point clouds which can lead
 * to increased numerical error. If desired, a user can demean the point cloud,
 * run iterative closest point, and composite the resulting ICP transformation
 * with the translations from demeaning to obtain a transformation between
 * the original point clouds.
 *
 * \author Radu B. Rusu, Matthew Cong
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointWithNormals
: public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
  using PointCloudSource = typename IterativeClosestPoint<PointSource,
                                                          PointTarget,
                                                          Scalar>::PointCloudSource;
  using PointCloudTarget = typename IterativeClosestPoint<PointSource,
                                                          PointTarget,
                                                          Scalar>::PointCloudTarget;
  using Matrix4 =
      typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      transformation_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::
      correspondence_rejectors_;

  using Ptr =
      shared_ptr<IterativeClosestPointWithNormals<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const IterativeClosestPointWithNormals<PointSource, PointTarget, Scalar>>;

  /** \brief Empty constructor. */
  IterativeClosestPointWithNormals()
  {
    reg_name_ = "IterativeClosestPointWithNormals";
    setUseSymmetricObjective(false);
    setEnforceSameDirectionNormals(true);
    // correspondence_rejectors_.add
  };

  /** \brief Empty destructor */
  ~IterativeClosestPointWithNormals() override = default;

  /** \brief Set whether to use a symmetric objective function or not
   *
   * \param[in] use_symmetric_objective whether to use a symmetric objective function or
   * not
   */
  inline void
  setUseSymmetricObjective(bool use_symmetric_objective)
  {
    use_symmetric_objective_ = use_symmetric_objective;
    if (use_symmetric_objective_) {
      auto symmetric_transformation_estimation = pcl::make_shared<
          pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<
              PointSource,
              PointTarget,
              Scalar>>();
      symmetric_transformation_estimation->setEnforceSameDirectionNormals(
          enforce_same_direction_normals_);
      transformation_estimation_ = symmetric_transformation_estimation;
    }
    else {
      transformation_estimation_.reset(
          new pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource,
                                                                         PointTarget,
                                                                         Scalar>());
    }
  }

  /** \brief Obtain whether a symmetric objective is used or not */
  inline bool
  getUseSymmetricObjective() const
  {
    return use_symmetric_objective_;
  }

  /** \brief Set whether or not to negate source or target normals on a per-point basis
   * such that they point in the same direction. Only applicable to the symmetric
   * objective function.
   *
   * \param[in] enforce_same_direction_normals whether to negate source or target
   * normals on a per-point basis such that they point in the same direction.
   */
  inline void
  setEnforceSameDirectionNormals(bool enforce_same_direction_normals)
  {
    enforce_same_direction_normals_ = enforce_same_direction_normals;
    auto symmetric_transformation_estimation = dynamic_pointer_cast<
        pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource,
                                                                            PointTarget,
                                                                            Scalar>>(
        transformation_estimation_);
    if (symmetric_transformation_estimation)
      symmetric_transformation_estimation->setEnforceSameDirectionNormals(
          enforce_same_direction_normals_);
  }

  /** \brief Obtain whether source or target normals are negated on a per-point basis
   * such that they point in the same direction or not */
  inline bool
  getEnforceSameDirectionNormals() const
  {
    return enforce_same_direction_normals_;
  }

protected:
  /** \brief Apply a rigid transform to a given dataset
   * \param[in] input the input point cloud
   * \param[out] output the resultant output point cloud
   * \param[in] transform a 4x4 rigid transformation
   * \note Can be used with cloud_in equal to cloud_out
   */
  void
  transformCloud(const PointCloudSource& input,
                 PointCloudSource& output,
                 const Matrix4& transform) override;

  /** \brief Type of objective function (asymmetric vs. symmetric) used for transform
   * estimation */
  bool use_symmetric_objective_;
  /** \brief Whether or not to negate source and/or target normals such that they point
   * in the same direction in the symmetric objective function */
  bool enforce_same_direction_normals_;
};

} // namespace pcl

#include <pcl/registration/impl/icp.hpp>
