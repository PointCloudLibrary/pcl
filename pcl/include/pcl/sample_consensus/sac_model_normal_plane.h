/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: sac_model_normal_plane.h 36021 2011-02-17 03:44:01Z vrabaud $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_NORMALPLANE_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_NORMALPLANE_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SampleConsensusModelNormalPlane defines a model for 3D plane segmentation using additional surface normal
    * constraints.
    * \author Radu Bogdan Rusu and Jared Glover
    */
  template <typename PointT, typename PointNT>
  class SampleConsensusModelNormalPlane : public SampleConsensusModelPlane<PointT>, public SampleConsensusModelFromNormals<PointT, PointNT>
  {
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModelFromNormals<PointT, PointNT>::normals_;
    using SampleConsensusModelFromNormals<PointT, PointNT>::normal_distance_weight_;

    public:

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNPtr PointCloudNPtr;
      typedef typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNConstPtr PointCloudNConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelNormalPlane> Ptr;

      /** \brief Constructor for base SampleConsensusModelNormalPlane.
        * \param cloud the input point cloud dataset
        */
      SampleConsensusModelNormalPlane (const PointCloudConstPtr &cloud) : SampleConsensusModelPlane<PointT> (cloud),
                                                                          eps_angle_ (0.0), eps_dist_ (0.0)
      {
        axis_.setZero ();
      }

      /** \brief Constructor for base SampleConsensusModelNormalPlane.
        * \param cloud the input point cloud dataset
        * \param indices a vector of point indices to be used from \a cloud
        */
      SampleConsensusModelNormalPlane (const PointCloudConstPtr &cloud, const std::vector<int> &indices) : SampleConsensusModelPlane<PointT> (cloud, indices),
                                                                                                           eps_angle_ (0.0), eps_dist_ (0.0)
      {
        axis_.setZero ();
      }

      /** \brief Set the axis along which we need to search for a plane perpendicular to.
        * \param ax the axis along which we need to search for a plane perpendicular to
        */
      inline void setAxis (const Eigen::Vector3f &ax) { axis_ = ax; }

      /** \brief Get the axis along which we need to search for a plane perpendicular to. */
      inline Eigen::Vector3f getAxis () { return (axis_); }

      /** \brief Set the angle epsilon (delta) threshold.
        * \param ea the maximum allowed difference between the plane normal and the given axis.
        */
      inline void setEpsAngle (double ea) { eps_angle_ = ea; }

      /** \brief Get the angle epsilon (delta) threshold. */
      inline double getEpsAngle () { return (eps_angle_); }

      /** \brief Set the distance we expect the plane to be from the origin
        * \param d distance from the template plane to the origin
        */
      inline void setDistanceFromOrigin (double d) { distance_from_origin_ = d; }

      /** \brief Get the distance of the plane from the origin. */
      inline double getDistanceFromOrigin () { return (distance_from_origin_); }

      /** \brief Set the distance epsilon (delta) threshold.
        * \param delta the maximum allowed deviation from the template distance from the origin
        */
      inline void setEpsDist (double delta) { eps_dist_ = delta; }

      /** \brief Get the distance epsilon (delta) threshold. */
      inline double getEpsDist () { return (eps_dist_); }

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param model_coefficients the coefficients of a plane model that we need to compute distances to
        * \param inliers the resultant model inliers
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers);

      /** \brief Compute all distances from the cloud data to a given plane model.
        * \param model_coefficients the coefficients of a plane model that we need to compute distances to
        * \param distances the resultant estimated distances
        */
      void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);

      /** \brief Return an unique id for this model (SACMODEL_NORMAL_PLANE). */
      inline pcl::SacModel getModelType () const { return (SACMODEL_NORMAL_PLANE); }

    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param model_coefficients the set of model coefficients
        */
      bool isModelValid (const Eigen::VectorXf &model_coefficients);

   private:
      /** \brief The axis along which we need to search for a plane perpendicular to. */
      Eigen::Vector3f axis_;

      /** \brief The distance from the template plane to the origin. */
      double distance_from_origin_;

      /** \brief The maximum allowed difference between the plane normal and the given axis. */
      double eps_angle_;

      /** \brief The maximum allowed deviation from the template distance from the origin. */
      double eps_dist_;
  };
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_NORMALPLANE_H_
