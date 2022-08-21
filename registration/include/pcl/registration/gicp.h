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

#include <pcl/registration/bfgs.h>
#include <pcl/registration/icp.h>

namespace pcl {
/** \brief GeneralizedIterativeClosestPoint is an ICP variant that implements the
 * generalized iterative closest point algorithm as described by Alex Segal et al. in
 * http://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf
 * The approach is based on using anisotropic cost functions to optimize the alignment
 * after closest point assignments have been made.
 * The original code uses GSL and ANN while in ours we use an eigen mapped BFGS and
 * FLANN.
 * \author Nizar Sallem
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class GeneralizedIterativeClosestPoint
: public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::getClassName;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::indices_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::input_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::tree_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::tree_reciprocal_;
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

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using MatricesVector =
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
  using MatricesVectorPtr = shared_ptr<MatricesVector>;
  using MatricesVectorConstPtr = shared_ptr<const MatricesVector>;

  using InputKdTree = typename Registration<PointSource, PointTarget, Scalar>::KdTree;
  using InputKdTreePtr =
      typename Registration<PointSource, PointTarget, Scalar>::KdTreePtr;

  using Ptr =
      shared_ptr<GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>>;

  using Vector3 = typename Eigen::Matrix<Scalar, 3, 1>;
  using Vector4 = typename Eigen::Matrix<Scalar, 4, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix3 = typename Eigen::Matrix<Scalar, 3, 3>;
  using Matrix4 =
      typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4;
  using AngleAxis = typename Eigen::AngleAxis<Scalar>;

  /** \brief Empty constructor. */
  GeneralizedIterativeClosestPoint()
  : k_correspondences_(20)
  , gicp_epsilon_(0.001)
  , rotation_epsilon_(2e-3)
  , mahalanobis_(0)
  , max_inner_iterations_(20)
  , translation_gradient_tolerance_(1e-2)
  , rotation_gradient_tolerance_(1e-2)
  {
    min_number_correspondences_ = 4;
    reg_name_ = "GeneralizedIterativeClosestPoint";
    max_iterations_ = 200;
    transformation_epsilon_ = 5e-4;
    corr_dist_threshold_ = 5.;
    rigid_transformation_estimation_ = [this](const PointCloudSource& cloud_src,
                                              const pcl::Indices& indices_src,
                                              const PointCloudTarget& cloud_tgt,
                                              const pcl::Indices& indices_tgt,
                                              Matrix4& transformation_matrix) {
      estimateRigidTransformationBFGS(
          cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
    };
  }

  /** \brief Provide a pointer to the input dataset
   * \param cloud the const boost shared pointer to a PointCloud message
   */
  inline void
  setInputSource(const PointCloudSourceConstPtr& cloud) override
  {

    if (cloud->points.empty()) {
      PCL_ERROR(
          "[pcl::%s::setInputSource] Invalid or empty point cloud dataset given!\n",
          getClassName().c_str());
      return;
    }
    PointCloudSource input = *cloud;
    // Set all the point.data[3] values to 1 to aid the rigid transformation
    for (std::size_t i = 0; i < input.size(); ++i)
      input[i].data[3] = 1.0;

    pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource(cloud);
    input_covariances_.reset();
  }

  /** \brief Provide a pointer to the covariances of the input source (if computed
   * externally!). If not set, GeneralizedIterativeClosestPoint will compute the
   * covariances itself. Make sure to set the covariances AFTER setting the input source
   * point cloud (setting the input source point cloud will reset the covariances).
   * \param[in] covariances the input source covariances
   */
  inline void
  setSourceCovariances(const MatricesVectorPtr& covariances)
  {
    input_covariances_ = covariances;
  }

  /** \brief Provide a pointer to the input target (e.g., the point cloud that we want
   * to align the input source to) \param[in] target the input point cloud target
   */
  inline void
  setInputTarget(const PointCloudTargetConstPtr& target) override
  {
    pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputTarget(
        target);
    target_covariances_.reset();
  }

  /** \brief Provide a pointer to the covariances of the input target (if computed
   * externally!). If not set, GeneralizedIterativeClosestPoint will compute the
   * covariances itself. Make sure to set the covariances AFTER setting the input source
   * point cloud (setting the input source point cloud will reset the covariances).
   * \param[in] covariances the input target covariances
   */
  inline void
  setTargetCovariances(const MatricesVectorPtr& covariances)
  {
    target_covariances_ = covariances;
  }

  /** \brief Estimate a rigid rotation transformation between a source and a target
   * point cloud using an iterative non-linear BFGS approach.
   * \param[in] cloud_src the source point cloud dataset
   * \param[in] indices_src the vector of indices describing
   * the points of interest in \a cloud_src
   * \param[in] cloud_tgt the target point cloud dataset
   * \param[in] indices_tgt the vector of indices describing
   * the correspondences of the interest points from \a indices_src
   * \param[in,out] transformation_matrix the resultant transformation matrix
   */
  void
  estimateRigidTransformationBFGS(const PointCloudSource& cloud_src,
                                  const pcl::Indices& indices_src,
                                  const PointCloudTarget& cloud_tgt,
                                  const pcl::Indices& indices_tgt,
                                  Matrix4& transformation_matrix);

  /** \brief \return Mahalanobis distance matrix for the given point index */
  inline const Eigen::Matrix3d&
  mahalanobis(std::size_t index) const
  {
    assert(index < mahalanobis_.size());
    return mahalanobis_[index];
  }

  /** \brief Computes the derivative of the cost function w.r.t rotation angles.
   * rotation matrix is obtainded from rotation angles x[3], x[4] and x[5]
   * \return d/d_Phi, d/d_Theta, d/d_Psi respectively in g[3], g[4] and g[5]
   * \param[in] x array representing 3D transformation
   * \param[in] dCost_dR_T the transpose of the derivative of the cost function w.r.t
   * rotation matrix
   * \param[out] g gradient vector
   */
  void
  computeRDerivative(const Vector6d& x,
                     const Eigen::Matrix3d& dCost_dR_T,
                     Vector6d& g) const;

  /** \brief Set the rotation epsilon (maximum allowable difference between two
   * consecutive rotations) in order for an optimization to be considered as having
   * converged to the final solution.
   * \param epsilon the rotation epsilon
   */
  inline void
  setRotationEpsilon(double epsilon)
  {
    rotation_epsilon_ = epsilon;
  }

  /** \brief Get the rotation epsilon (maximum allowable difference between two
   * consecutive rotations) as set by the user.
   */
  inline double
  getRotationEpsilon() const
  {
    return rotation_epsilon_;
  }

  /** \brief Set the number of neighbors used when selecting a point neighbourhood
   * to compute covariances.
   * A higher value will bring more accurate covariance matrix but will make
   * covariances computation slower.
   * \param k the number of neighbors to use when computing covariances
   */
  void
  setCorrespondenceRandomness(int k)
  {
    k_correspondences_ = k;
  }

  /** \brief Get the number of neighbors used when computing covariances as set by
   * the user
   */
  int
  getCorrespondenceRandomness() const
  {
    return k_correspondences_;
  }

  /** \brief Set maximum number of iterations at the optimization step
   * \param[in] max maximum number of iterations for the optimizer
   */
  void
  setMaximumOptimizerIterations(int max)
  {
    max_inner_iterations_ = max;
  }

  /** \brief Return maximum number of iterations at the optimization step
   */
  int
  getMaximumOptimizerIterations() const
  {
    return max_inner_iterations_;
  }

  /** \brief Set the minimal translation gradient threshold for early optimization stop
   * \param[in] tolerance translation gradient threshold in meters
   */
  void
  setTranslationGradientTolerance(double tolerance)
  {
    translation_gradient_tolerance_ = tolerance;
  }

  /** \brief Return the minimal translation gradient threshold for early optimization
   * stop
   */
  double
  getTranslationGradientTolerance() const
  {
    return translation_gradient_tolerance_;
  }

  /** \brief Set the minimal rotation gradient threshold for early optimization stop
   * \param[in] tolerance rotation gradient threshold in radians
   */
  void
  setRotationGradientTolerance(double tolerance)
  {
    rotation_gradient_tolerance_ = tolerance;
  }

  /** \brief Return the minimal rotation gradient threshold for early optimization stop
   */
  double
  getRotationGradientTolerance() const
  {
    return rotation_gradient_tolerance_;
  }

protected:
  /** \brief The number of neighbors used for covariances computation.
   * default: 20
   */
  int k_correspondences_;

  /** \brief The epsilon constant for gicp paper; this is NOT the convergence
   * tolerance
   * default: 0.001
   */
  double gicp_epsilon_;

  /** The epsilon constant for rotation error. (In GICP the transformation epsilon
   * is split in rotation part and translation part).
   * default: 2e-3
   */
  double rotation_epsilon_;

  /** \brief base transformation */
  Matrix4 base_transformation_;

  /** \brief Temporary pointer to the source dataset. */
  const PointCloudSource* tmp_src_;

  /** \brief Temporary pointer to the target dataset. */
  const PointCloudTarget* tmp_tgt_;

  /** \brief Temporary pointer to the source dataset indices. */
  const pcl::Indices* tmp_idx_src_;

  /** \brief Temporary pointer to the target dataset indices. */
  const pcl::Indices* tmp_idx_tgt_;

  /** \brief Input cloud points covariances. */
  MatricesVectorPtr input_covariances_;

  /** \brief Target cloud points covariances. */
  MatricesVectorPtr target_covariances_;

  /** \brief Mahalanobis matrices holder. */
  std::vector<Eigen::Matrix3d> mahalanobis_;

  /** \brief maximum number of optimizations */
  int max_inner_iterations_;

  /** \brief minimal translation gradient for early optimization stop */
  double translation_gradient_tolerance_;

  /** \brief minimal rotation gradient for early optimization stop */
  double rotation_gradient_tolerance_;

  /** \brief compute points covariances matrices according to the K nearest
   * neighbors. K is set via setCorrespondenceRandomness() method.
   * \param cloud pointer to point cloud
   * \param tree KD tree performer for nearest neighbors search
   * \param[out] cloud_covariances covariances matrices for each point in the cloud
   */
  template <typename PointT>
  void
  computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     const typename pcl::search::KdTree<PointT>::Ptr tree,
                     MatricesVector& cloud_covariances);

  /** \return trace of mat1 . mat2
   * \param mat1 matrix of dimension nxm
   * \param mat2 matrix of dimension mxp
   */
  inline double
  matricesInnerProd(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2) const
  {
    if (mat1.cols() != mat2.rows()) {
      PCL_THROW_EXCEPTION(PCLException,
                          "The two matrices' shapes don't match. "
                          "They are ("
                              << mat1.rows() << ", " << mat1.cols() << ") and ("
                              << mat2.rows() << ", " << mat2.cols() << ")");
    }
    return (mat1 * mat2).trace();
  }

  /** \brief Rigid transformation computation method  with initial guess.
   * \param output the transformed input point cloud dataset using the rigid
   * transformation found \param guess the initial guess of the transformation to
   * compute
   */
  void
  computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  /** \brief Search for the closest nearest neighbor of a given point.
   * \param query the point to search a nearest neighbour for
   * \param index vector of size 1 to store the index of the nearest neighbour found
   * \param distance vector of size 1 to store the distance to nearest neighbour found
   */
  inline bool
  searchForNeighbors(const PointSource& query,
                     pcl::Indices& index,
                     std::vector<float>& distance)
  {
    int k = tree_->nearestKSearch(query, 1, index, distance);
    if (k == 0)
      return (false);
    return (true);
  }

  /// \brief compute transformation matrix from transformation matrix
  void
  applyState(Matrix4& t, const Vector6d& x) const;

  /// \brief optimization functor structure
  struct OptimizationFunctorWithIndices : public BFGSDummyFunctor<double, 6> {
    OptimizationFunctorWithIndices(const GeneralizedIterativeClosestPoint* gicp)
    : BFGSDummyFunctor<double, 6>(), gicp_(gicp)
    {}
    double
    operator()(const Vector6d& x) override;
    void
    df(const Vector6d& x, Vector6d& df) override;
    void
    fdf(const Vector6d& x, double& f, Vector6d& df) override;
    BFGSSpace::Status
    checkGradient(const Vector6d& g) override;

    const GeneralizedIterativeClosestPoint* gicp_;
  };

  std::function<void(const pcl::PointCloud<PointSource>& cloud_src,
                     const pcl::Indices& src_indices,
                     const pcl::PointCloud<PointTarget>& cloud_tgt,
                     const pcl::Indices& tgt_indices,
                     Matrix4& transformation_matrix)>
      rigid_transformation_estimation_;
};
} // namespace pcl

#include <pcl/registration/impl/gicp.hpp>
