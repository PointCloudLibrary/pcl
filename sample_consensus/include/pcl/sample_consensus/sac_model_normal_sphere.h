/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id: sac_model_normal_sphere.h schrandt $
 *
 */

#pragma once

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  /** \brief @b SampleConsensusModelNormalSphere defines a model for 3D sphere
    * segmentation using additional surface normal constraints. Basically this
    * means that checking for inliers will not only involve a "distance to
    * model" criterion, but also an additional "maximum angular deviation"
    * between the sphere's normal and the inlier points normals.
    *
    * The model coefficients are defined as:
    * <ul>
    * <li><b>center.x</b> : the X coordinate of the sphere's center
    * <li><b>center.y</b> : the Y coordinate of the sphere's center
    * <li><b>center.z</b> : the Z coordinate of the sphere's center
    * <li><b>radius</b> : radius of the sphere
    * </ul>
    *
    * \author Stefan Schrandt
    * \ingroup sample_consensus
    */
  template <typename PointT, typename PointNT>
  class SampleConsensusModelNormalSphere : public SampleConsensusModelSphere<PointT>, public SampleConsensusModelFromNormals<PointT, PointNT>
  {
    public:
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

      using PointCloudNPtr = typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNPtr;
      using PointCloudNConstPtr = typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNConstPtr;

      using Ptr = shared_ptr<SampleConsensusModelNormalSphere<PointT, PointNT> >;
      using ConstPtr = shared_ptr<const SampleConsensusModelNormalSphere<PointT, PointNT>>;

      /** \brief Constructor for base SampleConsensusModelNormalSphere.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelNormalSphere (const PointCloudConstPtr &cloud, 
                                        bool random = false) 
        : SampleConsensusModelSphere<PointT> (cloud, random)
        , SampleConsensusModelFromNormals<PointT, PointNT> ()
      {
        model_name_ = "SampleConsensusModelNormalSphere";
        sample_size_ = 4;
        model_size_ = 4;
      }

      /** \brief Constructor for base SampleConsensusModelNormalSphere.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelNormalSphere (const PointCloudConstPtr &cloud, 
                                        const Indices &indices,
                                        bool random = false) 
        : SampleConsensusModelSphere<PointT> (cloud, indices, random)
        , SampleConsensusModelFromNormals<PointT, PointNT> ()
      {
        model_name_ = "SampleConsensusModelNormalSphere";
        sample_size_ = 4;
        model_size_ = 4;
      }
      
      /** \brief Empty destructor */
      ~SampleConsensusModelNormalSphere () {}

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the coefficients of a sphere model that we need to compute distances to
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void 
      selectWithinDistance (const Eigen::VectorXf &model_coefficients, 
                            const double threshold, 
                            Indices &inliers) override;

      /** \brief Count all the points which respect the given model coefficients as inliers. 
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      std::size_t
      countWithinDistance (const Eigen::VectorXf &model_coefficients,
                           const double threshold) const override;

      /** \brief Compute all distances from the cloud data to a given sphere model.
        * \param[in] model_coefficients the coefficients of a sphere model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const override;

      /** \brief Return a unique id for this model (SACMODEL_NORMAL_SPHERE). */
      inline pcl::SacModel 
      getModelType () const override { return (SACMODEL_NORMAL_SPHERE); }

    	PCL_MAKE_ALIGNED_OPERATOR_NEW

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;
      using SampleConsensusModelSphere<PointT>::isModelValid;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#endif
