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

#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>

namespace pcl {

/** \brief @b SampleConsensusModelTorus defines a model for 3D torus segmentation.
 * The model coefficients are defined as:
 *   - \b radii.inner          : the torus's inner radius
 *   - \b radii.outer          : the torus's outer radius
 *   - \b torus_center_point.x  : the X coordinate of the center of the torus
 *   - \b torus_center_point.y  : the Y coordinate of the center of the torus
 *   - \b torus_center_point.z  : the Z coordinate of the center of the torus
 *   - \b torus_normal.x  : the X coordinate of the normal of the torus
 *   - \b torus_normal.y  : the Y coordinate of the normal of the torus
 *   - \b torus_normal.z  : the Z coordinate of the normal of the torus
 *
 * \author lasdasdas
 * \ingroup sample_consensus
 */
template <typename PointT, typename PointNT>
class SampleConsensusModelTorus
: public SampleConsensusModel<PointT>,
  public SampleConsensusModelFromNormals<PointT, PointNT> {
  using SampleConsensusModel<PointT>::model_name_;
  using SampleConsensusModel<PointT>::input_;
  using SampleConsensusModel<PointT>::indices_;
  using SampleConsensusModel<PointT>::radius_min_;
  using SampleConsensusModel<PointT>::radius_max_;
  using SampleConsensusModelFromNormals<PointT, PointNT>::normals_;
  using SampleConsensusModelFromNormals<PointT, PointNT>::normal_distance_weight_;
  using SampleConsensusModel<PointT>::error_sqr_dists_;

  using PointCloud = typename SampleConsensusModel<PointT>::PointCloud;
  using PointCloudPtr = typename SampleConsensusModel<PointT>::PointCloudPtr;
  using PointCloudConstPtr = typename SampleConsensusModel<PointT>::PointCloudConstPtr;

public:
  using Ptr = shared_ptr<SampleConsensusModelTorus<PointT, PointNT>>;
  using ConstPtr = shared_ptr<const SampleConsensusModelTorus<PointT, PointNT>>;

  /** \brief Constructor for base SampleConsensusModelTorus.
   * \param[in] cloud the input point cloud dataset
   * \param[in] random if true set the random seed to the current time, else set to
   * 12345 (default: false)
   */
  SampleConsensusModelTorus(const PointCloudConstPtr& cloud, bool random = false)
  : SampleConsensusModel<PointT>(cloud, random)
  , SampleConsensusModelFromNormals<PointT, PointNT>()
  {
    model_name_ = "SampleConsensusModelTorus";
    sample_size_ = 4;
    model_size_ = 8;
  }

  /** \brief Constructor for base SampleConsensusModelTorus.
   * \param[in] cloud the input point cloud dataset
   * \param[in] indices a vector of point indices to be used from \a cloud
   * \param[in] random if true set the random seed to the current time, else set to
   * 12345 (default: false)
   */
  SampleConsensusModelTorus(const PointCloudConstPtr& cloud,
                            const Indices& indices,
                            bool random = false)
  : SampleConsensusModel<PointT>(cloud, indices, random)
  , SampleConsensusModelFromNormals<PointT, PointNT>()
  {
    model_name_ = "SampleConsensusModelTorus";
    sample_size_ = 4;
    model_size_ = 8;
  }

  /** \brief Copy constructor.
   * \param[in] source the model to copy into this
   */
  SampleConsensusModelTorus(const SampleConsensusModelTorus& source)
  : SampleConsensusModel<PointT>(), SampleConsensusModelFromNormals<PointT, PointNT>()
  {
    *this = source;
    model_name_ = "SampleConsensusModelTorus";
  }

  /** \brief Empty destructor */
  ~SampleConsensusModelTorus() override = default;

  /** \brief Copy constructor.
   * \param[in] source the model to copy into this
   */
  inline SampleConsensusModelTorus&
  operator=(const SampleConsensusModelTorus& source)
  {
    SampleConsensusModelFromNormals<PointT, PointNT>::operator=(source);
    return (*this);
  }
  /** \brief Check whether the given index samples can form a valid torus model, compute
   * the model coefficients from these samples and store them in model_coefficients. The
   * torus coefficients are: radii, torus_center_point, torus_normal.
   * \param[in] samples the point indices found as possible good candidates for creating a valid model
   * \param[out] model_coefficients the resultant model coefficients
   */
  bool
  computeModelCoefficients(const Indices& samples,
                           Eigen::VectorXf& model_coefficients) const override;

  /** \brief Compute all distances from the cloud data to a given torus model.
   * \param[in] model_coefficients the coefficients of a torus model that we need to compute distances to
   * \param[out] distances the resultant estimated distances
   */
  void
  getDistancesToModel(const Eigen::VectorXf& model_coefficients,
                      std::vector<double>& distances) const override;

  /** \brief Select all the points which respect the given model coefficients as
   * inliers.
   * \param[in] model_coefficients the coefficients of a torus model that we need to compute distances to
   * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
   * \param[out] inliers the
   * resultant model inliers
   */
  void
  selectWithinDistance(const Eigen::VectorXf& model_coefficients,
                       const double threshold,
                       Indices& inliers) override;

  /** \brief Count all the points which respect the given model coefficients as inliers.
   *
   * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
   * \param[in] threshold maximum admissible distance threshold for
   * determining the inliers from the outliers \return the resultant number of inliers
   */
  std::size_t
  countWithinDistance(const Eigen::VectorXf& model_coefficients,
                      const double threshold) const override;

  /** \brief Recompute the torus coefficients using the given inlier set and return them
   * to the user.
   * \param[in] inliers the data inliers found as supporting the model
   * \param[in] model_coefficients the initial guess for the optimization
   * \param[out] optimized_coefficients the resultant recomputed coefficients after
   * non-linear optimization
   */
  void
  optimizeModelCoefficients(const Indices& inliers,
                            const Eigen::VectorXf& model_coefficients,
                            Eigen::VectorXf& optimized_coefficients) const override;

  /** \brief Create a new point cloud with inliers projected onto the torus model.
   * \param[in] inliers the data inliers that we want to project on the torus model
   * \param[in] model_coefficients the coefficients of a torus model
   * \param[out] projected_points the resultant projected points
   * \param[in] copy_data_fields set to true if we need to copy the other data fields
   */
  void
  projectPoints(const Indices& inliers,
                const Eigen::VectorXf& model_coefficients,
                PointCloud& projected_points,
                bool copy_data_fields = true) const override;

  /** \brief Verify whether a subset of indices verifies the given torus model
   * coefficients.
   * \param[in] indices the data indices that need to be tested against the torus model
   * \param[in] model_coefficients the torus model coefficients
   * \param[in] threshold a maximum admissible distance threshold for determining the
   * inliers from the outliers
   */
  bool
  doSamplesVerifyModel(const std::set<index_t>& indices,
                       const Eigen::VectorXf& model_coefficients,
                       const double threshold) const override;

  /** \brief Return a unique id for this model (SACMODEL_TORUS). */
  inline pcl::SacModel
  getModelType() const override
  {
    return (SACMODEL_TORUS);
  }

protected:
  using SampleConsensusModel<PointT>::sample_size_;
  using SampleConsensusModel<PointT>::model_size_;

  /** \brief Project a point onto a torus given by its model coefficients (radii,
   * torus_center_point, torus_normal)
   * \param[in] pt the input point to project
   * \param[in] model_coefficients the coefficients of the torus (radii, torus_center_point, torus_normal)
   * \param[out] pt_proj the resultant projected point
   */
  void
  projectPointToTorus(const Eigen::Vector3f& pt,
                      const Eigen::Vector3f& pt_n,
                      const Eigen::VectorXf& model_coefficients,
                      Eigen::Vector3f& pt_proj) const;

  /** \brief Check whether a model is valid given the user constraints.
   * \param[in] model_coefficients the set of model coefficients
   */
  bool
  isModelValid(const Eigen::VectorXf& model_coefficients) const override;

  /** \brief Check if a sample of indices results in a good sample of points
   * indices. Pure virtual.
   * \param[in] samples the resultant index samples
   */
  bool
  isSampleGood(const Indices& samples) const override;

private:
  struct OptimizationFunctor : pcl::Functor<double> {
    /** Functor constructor
     * \param[in] indices the indices of data points to evaluate
     * \param[in] estimator pointer to the estimator object
     */
    OptimizationFunctor(const pcl::SampleConsensusModelTorus<PointT, PointNT>* model,
                        const Indices& indices)
    : pcl::Functor<double>(indices.size()), model_(model), indices_(indices)
    {}

    /** Cost function to be minimized
     * \param[in] x the variables array
     * \param[out] fvec the resultant functions evaluations
     * \return 0
     */
    int
    operator()(const Eigen::VectorXd& xs, Eigen::VectorXd& fvec) const
    {
      // Getting constants from state vector
      const double& R = xs[0];
      const double& r = xs[1];

      const double& x0 = xs[2];
      const double& y0 = xs[3];
      const double& z0 = xs[4];

      const Eigen::Vector3d centroid{x0, y0, z0};

      const double& nx = xs[5];
      const double& ny = xs[6];
      const double& nz = xs[7];

      const Eigen::Vector3d n1{0.0, 0.0, 1.0};
      const Eigen::Vector3d n2 = Eigen::Vector3d{nx, ny, nz}.normalized();

      for (size_t j = 0; j < indices_.size(); j++) {

        size_t i = indices_[j];
        const Eigen::Vector3d pt =
            (*model_->input_)[i].getVector3fMap().template cast<double>();

        Eigen::Vector3d pte{pt - centroid};

        // Transposition is inversion
        // Using Quaternions instead of Rodrigues
        pte = Eigen::Quaterniond()
                  .setFromTwoVectors(n1, n2)
                  .toRotationMatrix()
                  .transpose() *
              pte;

        const double& x = pte[0];
        const double& y = pte[1];
        const double& z = pte[2];

        fvec[j] = (std::pow(sqrt(x * x + y * y) - R, 2) + z * z - r * r);
      }
      return 0;
    }

    const pcl::SampleConsensusModelTorus<PointT, PointNT>* model_;
    const Indices& indices_;
  };
};
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_torus.hpp>
#endif
