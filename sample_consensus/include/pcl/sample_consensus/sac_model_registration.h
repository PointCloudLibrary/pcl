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

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_H_

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <map>

namespace pcl
{
  /** \brief SampleConsensusModelRegistration defines a model for Point-To-Point registration outlier rejection.
    * \author Radu Bogdan Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelRegistration : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;
      using SampleConsensusModel<PointT>::isModelValid;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelRegistration> Ptr;

      /** \brief Constructor for base SampleConsensusModelRegistration.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistration (const PointCloudConstPtr &cloud, 
                                        bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random)
        , target_ ()
        , indices_tgt_ ()
        , correspondences_ ()
        , sample_dist_thresh_ (0)
      {
        // Call our own setInputCloud
        setInputCloud (cloud);
        model_name_ = "SampleConsensusModelRegistration";
        sample_size_ = 3;
        model_size_ = 16;
      }

      /** \brief Constructor for base SampleConsensusModelRegistration.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistration (const PointCloudConstPtr &cloud,
                                        const std::vector<int> &indices,
                                        bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random)
        , target_ ()
        , indices_tgt_ ()
        , correspondences_ ()
        , sample_dist_thresh_ (0)
      {
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud, indices);
        model_name_ = "SampleConsensusModelRegistration";
        sample_size_ = 3;
        model_size_ = 16;
      }
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelRegistration () {}

      /** \brief Provide a pointer to the input dataset
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      inline virtual void
      setInputCloud (const PointCloudConstPtr &cloud)
      {
        SampleConsensusModel<PointT>::setInputCloud (cloud);
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud);
      }

      /** \brief Set the input point cloud target.
        * \param[in] target the input point cloud target
        */
      inline void
      setInputTarget (const PointCloudConstPtr &target)
      {
        target_ = target;
        indices_tgt_.reset (new std::vector<int>);
        // Cache the size and fill the target indices
        int target_size = static_cast<int> (target->size ());
        indices_tgt_->resize (target_size);

        for (int i = 0; i < target_size; ++i)
          (*indices_tgt_)[i] = i;
        computeOriginalIndexMapping ();
      }

      /** \brief Set the input point cloud target.
        * \param[in] target the input point cloud target
        * \param[in] indices_tgt a vector of point indices to be used from \a target
        */
      inline void
      setInputTarget (const PointCloudConstPtr &target, const std::vector<int> &indices_tgt)
      {
        target_ = target;
        indices_tgt_.reset (new std::vector<int> (indices_tgt));
        computeOriginalIndexMapping ();
      }

      /** \brief Compute a 4x4 rigid transformation matrix from the samples given
        * \param[in] samples the indices found as good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the transformed points to their correspondences
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the 4x4 transformation matrix
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void
      selectWithinDistance (const Eigen::VectorXf &model_coefficients,
                            const double threshold,
                            std::vector<int> &inliers);

      /** \brief Count all the points which respect the given model coefficients as inliers.
        *
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      virtual int
      countWithinDistance (const Eigen::VectorXf &model_coefficients,
                           const double threshold);

      /** \brief Recompute the 4x4 transformation using the given inlier set
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed transformation
        */
      void
      optimizeModelCoefficients (const std::vector<int> &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients);

      void
      projectPoints (const std::vector<int> &,
                     const Eigen::VectorXf &,
                     PointCloud &, bool = true)
      {
      };

      bool
      doSamplesVerifyModel (const std::set<int> &,
                            const Eigen::VectorXf &,
                            const double)
      {
        return (false);
      }

      /** \brief Return an unique id for this model (SACMODEL_REGISTRATION). */
      inline pcl::SacModel
      getModelType () const { return (SACMODEL_REGISTRATION); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      virtual bool
      isSampleGood (const std::vector<int> &samples) const;

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero ();

        computeMeanAndCovarianceMatrix (*cloud, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param indices
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

    /** \brief Estimate a rigid transformation between a source and a target point cloud using an SVD closed-form
      * solution of absolute orientation using unit quaternions
      * \param[in] cloud_src the source point cloud dataset
      * \param[in] indices_src the vector of indices describing the points of interest in cloud_src
      * \param[in] cloud_tgt the target point cloud dataset
      * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
      * indices_src
      * \param[out] transform the resultant transformation matrix (as model coefficients)
      *
      * This method is an implementation of: Horn, B. “Closed-Form Solution of Absolute Orientation Using Unit Quaternions,” JOSA A, Vol. 4, No. 4, 1987
      */
      void
      estimateRigidTransformationSVD (const pcl::PointCloud<PointT> &cloud_src,
                                      const std::vector<int> &indices_src,
                                      const pcl::PointCloud<PointT> &cloud_tgt,
                                      const std::vector<int> &indices_tgt,
                                      Eigen::VectorXf &transform);

      /** \brief Compute mappings between original indices of the input_/target_ clouds. */
      void
      computeOriginalIndexMapping ()
      {
        if (!indices_tgt_ || !indices_ || indices_->empty () || indices_->size () != indices_tgt_->size ())
          return;
        for (size_t i = 0; i < indices_->size (); ++i)
          correspondences_[(*indices_)[i]] = (*indices_tgt_)[i];
      }

      /** \brief A boost shared pointer to the target point cloud data array. */
      PointCloudConstPtr target_;

      /** \brief A pointer to the vector of target point indices to use. */
      boost::shared_ptr <std::vector<int> > indices_tgt_;

      /** \brief Given the index in the original point cloud, give the matching original index in the target cloud */
      std::map<int, int> correspondences_;

      /** \brief Internal distance threshold used for the sample selection step. */
      double sample_dist_thresh_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <pcl/sample_consensus/impl/sac_model_registration.hpp>

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_H_
