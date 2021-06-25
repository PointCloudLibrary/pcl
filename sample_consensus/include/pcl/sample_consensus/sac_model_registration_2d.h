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

#pragma once

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  /** \brief SampleConsensusModelRegistration2D defines a model for Point-To-Point registration outlier rejection using distances between 2D pixels
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelRegistration2D : public pcl::SampleConsensusModelRegistration<PointT>
  {
    public:
      using pcl::SampleConsensusModelRegistration<PointT>::model_name_;
      using pcl::SampleConsensusModelRegistration<PointT>::input_;
      using pcl::SampleConsensusModelRegistration<PointT>::target_;
      using pcl::SampleConsensusModelRegistration<PointT>::indices_;
      using pcl::SampleConsensusModelRegistration<PointT>::indices_tgt_;
      using pcl::SampleConsensusModelRegistration<PointT>::error_sqr_dists_;
      using pcl::SampleConsensusModelRegistration<PointT>::correspondences_;
      using pcl::SampleConsensusModelRegistration<PointT>::sample_dist_thresh_;
      using pcl::SampleConsensusModelRegistration<PointT>::computeOriginalIndexMapping;
      using pcl::SampleConsensusModel<PointT>::isModelValid;

      using PointCloud = typename pcl::SampleConsensusModel<PointT>::PointCloud;
      using PointCloudPtr = typename pcl::SampleConsensusModel<PointT>::PointCloudPtr;
      using PointCloudConstPtr = typename pcl::SampleConsensusModel<PointT>::PointCloudConstPtr;

      using Ptr = shared_ptr<SampleConsensusModelRegistration2D<PointT> >;
      using ConstPtr = shared_ptr<const SampleConsensusModelRegistration2D<PointT> >;

      /** \brief Constructor for base SampleConsensusModelRegistration2D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistration2D (const PointCloudConstPtr &cloud,
                                          bool random = false) 
        : pcl::SampleConsensusModelRegistration<PointT> (cloud, random)
        , projection_matrix_ (Eigen::Matrix3f::Identity ())
      {
        // Call our own setInputCloud
        setInputCloud (cloud);
        model_name_ = "SampleConsensusModelRegistration2D";
        sample_size_ = 3;
        model_size_ = 16;
      }

      /** \brief Constructor for base SampleConsensusModelRegistration2D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistration2D (const PointCloudConstPtr &cloud,
                                          const Indices &indices,
                                          bool random = false)
        : pcl::SampleConsensusModelRegistration<PointT> (cloud, indices, random)
        , projection_matrix_ (Eigen::Matrix3f::Identity ())
      {
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud, indices);
        model_name_ = "SampleConsensusModelRegistration2D";
        sample_size_ = 3;
        model_size_ = 16;
      }
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelRegistration2D () = default;

      /** \brief Compute all distances from the transformed points to their correspondences
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const;

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void
      selectWithinDistance (const Eigen::VectorXf &model_coefficients,
                            const double threshold,
                            Indices &inliers);

      /** \brief Count all the points which respect the given model coefficients as inliers.
        *
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      virtual std::size_t
      countWithinDistance (const Eigen::VectorXf &model_coefficients,
                           const double threshold) const;

      /** \brief Set the camera projection matrix. 
        * \param[in] projection_matrix the camera projection matrix 
        */
      inline void
      setProjectionMatrix (const Eigen::Matrix3f &projection_matrix)
      { projection_matrix_ = projection_matrix; }

      /** \brief Get the camera projection matrix. */
      inline Eigen::Matrix3f
      getProjectionMatrix () const
      { return (projection_matrix_); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood (const Indices &samples) const;

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr&)
      {
        //// Compute the principal directions via PCA
        //Eigen::Vector4f xyz_centroid;
        //Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero ();

        //computeMeanAndCovarianceMatrix (*cloud, covariance_matrix, xyz_centroid);

        //// Check if the covariance matrix is finite or not.
        //for (int i = 0; i < 3; ++i)
        //  for (int j = 0; j < 3; ++j)
        //    if (!std::isfinite (covariance_matrix.coeffRef (i, j)))
        //      PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        //Eigen::Vector3f eigen_values;
        //pcl::eigen33 (covariance_matrix, eigen_values);

        //// Compute the distance threshold for sample selection
        //sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        //sample_dist_thresh_ *= sample_dist_thresh_;
        //PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr&,
                                      const Indices&)
      {
        //// Compute the principal directions via PCA
        //Eigen::Vector4f xyz_centroid;
        //Eigen::Matrix3f covariance_matrix;
        //computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        //// Check if the covariance matrix is finite or not.
        //for (int i = 0; i < 3; ++i)
        //  for (int j = 0; j < 3; ++j)
        //    if (!std::isfinite (covariance_matrix.coeffRef (i, j)))
        //      PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        //Eigen::Vector3f eigen_values;
        //pcl::eigen33 (covariance_matrix, eigen_values);

        //// Compute the distance threshold for sample selection
        //sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        //sample_dist_thresh_ *= sample_dist_thresh_;
        //PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

    private:
      /** \brief Camera projection matrix. */
      Eigen::Matrix3f projection_matrix_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <pcl/sample_consensus/impl/sac_model_registration_2d.hpp>
