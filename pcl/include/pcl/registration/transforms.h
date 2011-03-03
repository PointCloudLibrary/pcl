/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_TRANSFORMS_H_
#define PCL_TRANSFORMS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pcl
{
  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param cloud_in the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    */
  template <typename PointT> void 
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                    const Eigen::Vector4f &centroid, 
                    pcl::PointCloud<PointT> &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    */
  template <typename PointT> void 
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                    const std::vector<int> &indices, 
                    const Eigen::Vector4f &centroid, 
                    pcl::PointCloud<PointT> &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param cloud_in the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    */
  template <typename PointT> void 
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                    const Eigen::Vector4f &centroid, 
                    Eigen::MatrixXf &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    */
  template <typename PointT> void 
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                    const std::vector<int> &indices, 
                    const Eigen::Vector4f &centroid, 
                    Eigen::MatrixXf &cloud_out);

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param transform an affine transformation (typically a rigid transformation)
    * \note The density of the point cloud is lost, since density implies that
    * the origin is the point of view
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform);

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param transform an affine transformation (typically a rigid transformation)
    * \note The density of the point cloud is lost, since density implies that
    * the origin is the point of view
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform);

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param transform an affine transformation (typically a rigid transformation)
    * \note The density of the point cloud is lost, since density implies that
    * the origin is the point of view
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform);

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param transform an affine transformation (typically a rigid transformation)
    * \note The density of the point cloud is lost, since density implies that
    * the origin is the point of view
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform);

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param transform an affine transformation (typically a rigid transformation)
    * \note The density of the point cloud is lost, since density implies that
    * the origin is the point of view
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform);

  /** \brief Apply a rigid transform defined by a 3D offset and a quaternion
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param offset the translation component of the rigid transformation
    * \param rotation the rotation component of the rigid transformation
    * \note density of the point cloud is lost, since density implies that the
    * origin is the point of view
    */
  template <typename PointT> inline void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Vector3f &offset, 
                       const Eigen::Quaternionf &rotation);

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param cloud_in the input point cloud
    * \param cloud_out the resultant output point cloud
    * \param offset the translation component of the rigid transformation
    * \param rotation the rotation component of the rigid transformation
    * \note density of the point cloud is lost, since density implies that the
    * origin is the point of view
    */
  template <typename PointT> inline void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Vector3f &offset, 
                                  const Eigen::Quaternionf &rotation);
}

#include "pcl/registration/transforms.hpp"

#endif // PCL_TRANSFORMS_H_
