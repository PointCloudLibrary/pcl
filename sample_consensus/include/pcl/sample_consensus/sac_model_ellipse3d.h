/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/pcl_exports.h>

namespace pcl
{
  /** \brief SampleConsensusModelEllipse3D defines a model for 3D ellipse segmentation.
    *
    * The model coefficients are defined as:
    *   - \b center.x : the X coordinate of the ellipse's center
    *   - \b center.y : the Y coordinate of the ellipse's center
    *   - \b center.z : the Z coordinate of the ellipse's center 
    *   - \b semi_axis.u : semi-major axis length along the local u-axis of the ellipse
    *   - \b semi_axis.v : semi-minor axis length along the local v-axis of the ellipse
    *   - \b normal.x : the X coordinate of the normal's direction 
    *   - \b normal.y : the Y coordinate of the normal's direction 
    *   - \b normal.z : the Z coordinate of the normal's direction
    *   - \b u.x : the X coordinate of the local u-axis of the ellipse 
    *   - \b u.y : the Y coordinate of the local u-axis of the ellipse 
    *   - \b u.z : the Z coordinate of the local u-axis of the ellipse 
    *
    * For more details please refer to the following manuscript:
    * "Semi-autonomous Prosthesis Control Using Minimal Depth Information and Vibrotactile Feedback",
    * Miguel N. Castro & Strahinja Dosen. IEEE Transactions on Human-Machine Systems [under review]. arXiv:2210.00541.
    * (@ github.com/mnobrecastro/pcl-ellipse-fitting)
    * 
    * \author Miguel Nobre Castro (mnobrecastro@gmail.com)
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class PCL_EXPORTS SampleConsensusModelEllipse3D : public SampleConsensusModel<PointT>
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

      using Ptr = shared_ptr<SampleConsensusModelEllipse3D<PointT> >;
      using ConstPtr = shared_ptr<const SampleConsensusModelEllipse3D<PointT> >;

      /** \brief Constructor for base SampleConsensusModelEllipse3D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelEllipse3D (const PointCloudConstPtr &cloud, bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random)
      {
        model_name_ = "SampleConsensusModelEllipse3D";
        sample_size_ = 6;
        model_size_ = 11;
      }

      /** \brief Constructor for base SampleConsensusModelEllipse3D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelEllipse3D (const PointCloudConstPtr &cloud, 
                                    const Indices &indices,
                                    bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random)
      {
        model_name_ = "SampleConsensusModelEllipse3D";
        sample_size_ = 6;
        model_size_ = 11;
      }
      
      /** \brief Empty destructor */
      ~SampleConsensusModelEllipse3D () override = default;

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      SampleConsensusModelEllipse3D (const SampleConsensusModelEllipse3D &source) :
        SampleConsensusModel<PointT> ()
      {
        *this = source;
        model_name_ = "SampleConsensusModelEllipse3D";
      }

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      inline SampleConsensusModelEllipse3D&
      operator = (const SampleConsensusModelEllipse3D &source)
      {
        SampleConsensusModel<PointT>::operator=(source);
        return *this;
      }

      /** \brief Check whether the given index samples can form a valid 3D ellipse model, compute the model coefficients
        * from these samples and store them in model_coefficients. The ellipse coefficients are: x, y, R.
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const Indices &samples,
                                Eigen::VectorXf &model_coefficients) const override;

      /** \brief Compute all distances from the cloud data to a given 3D ellipse model.
        * \param[in] model_coefficients the coefficients of a 3D ellipse model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const override;

      /** \brief Compute all distances from the cloud data to a given 3D ellipse model.
        * \param[in] model_coefficients the coefficients of a 3D ellipse model that we need to compute distances to
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

       /** \brief Recompute the 3d ellipse coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the 3d ellipse model after refinement (e.g. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void
      optimizeModelCoefficients (const Indices &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients) const override;

      /** \brief Create a new point cloud with inliers projected onto the 3d ellipse model.
        * \param[in] inliers the data inliers that we want to project on the 3d ellipse model
        * \param[in] model_coefficients the coefficients of a 3d ellipse model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        */
      void
      projectPoints (const Indices &inliers,
                     const Eigen::VectorXf &model_coefficients,
                     PointCloud &projected_points,
                     bool copy_data_fields = true) const override;

      /** \brief Verify whether a subset of indices verifies the given 3d ellipse model coefficients.
        * \param[in] indices the data indices that need to be tested against the 3d ellipse model
        * \param[in] model_coefficients the 3d ellipse model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool
      doSamplesVerifyModel (const std::set<index_t> &indices,
                            const Eigen::VectorXf &model_coefficients,
                            const double threshold) const override;

      /** \brief Return a unique id for this model (SACMODEL_ELLIPSE3D). */
      inline pcl::SacModel
      getModelType () const override { return (SACMODEL_ELLIPSE3D); }

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

      struct OptimizationFunctor;
  };

  namespace internal
  {
    /** \brief Internal function to compute ellipse coefficients from 6 points. */
    PCL_EXPORTS bool
    computeModelCoefficientsEllipse3D (const Eigen::Matrix<float, 6, 3> &points,
                                       Eigen::VectorXf &model_coefficients);

    /** \brief Internal function to compute distances to an ellipse model. */
    PCL_EXPORTS void
    getDistancesToModelEllipse3D (const Eigen::VectorXf &model_coefficients,
                                  const Eigen::ArrayXf &x,
                                  const Eigen::ArrayXf &y,
                                  const Eigen::ArrayXf &z,
                                  std::vector<double> &distances);

    /** \brief Internal function to optimize ellipse coefficients. */
    PCL_EXPORTS void
    optimizeModelCoefficientsEllipse3D (const Eigen::ArrayXf &x,
                                        const Eigen::ArrayXf &y,
                                        const Eigen::ArrayXf &z,
                                        const Eigen::VectorXf &model_coefficients,
                                        Eigen::VectorXf &optimized_coefficients);

    /** \brief Internal function to compute ellipse point from parametric coefficients and angle.
      * \param[in] par the parametric coefficients (a, b, h, k, slant)
      * \param[in] th the angle (in radians)
      * \param[out] x the resultant X coordinate in local frame
      * \param[out] y the resultant Y coordinate in local frame
      */
    PCL_EXPORTS void
    getEllipsePoint (const Eigen::VectorXf& par, float th, float& x, float& y);

    /** \brief Internal function to compute the shortest distance vector from a point to an ellipse.
      * \param[in] par the ellipse coefficients (a, b, 0, 0, 0)
      * \param[in] u point X coordinate in local frame
      * \param[in] v point Y coordinate in local frame
      * \param[out] th_opt the resultant optimal angle on the ellipse
      * \return the distance vector from the point to its projection on the ellipse
      */
    PCL_EXPORTS Eigen::Vector2f
    dVec2Ellipse (const Eigen::VectorXf& par, float u, float v, float& th_opt);

    /** \brief Internal function to find the optimal angle using Golden Section Search.
      * \param[in] par the ellipse coefficients (a, b, 0, 0, 0)
      * \param[in] u point X coordinate in local frame
      * \param[in] v point Y coordinate in local frame
      * \param[in] th_min search interval lower bound
      * \param[in] th_max search interval upper bound
      * \param[in] epsilon search convergence tolerance
      * \return the optimal angle (in radians)
      */
    PCL_EXPORTS float
    goldenSectionSearch (const Eigen::VectorXf& par,
                         float u,
                         float v,
                         float th_min,
                         float th_max,
                         float epsilon);
  }
}

#define PCL_INSTANTIATE_SampleConsensusModelEllipse3D(T) template class PCL_EXPORTS pcl::SampleConsensusModelEllipse3D<T>;

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_ellipse3d.hpp>
#endif
