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
 *
 */
#ifndef PCL_TRANSFORMS_H_
#define PCL_TRANSFORMS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/PointIndices.h>

namespace pcl
{
  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, cloud_out, transform));
  }

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
  {
    return (transformPointCloud<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
  {
    return (transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (transformPointCloud<PointT, Scalar> (cloud_in, cloud_out, t));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, cloud_out, transform));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (transformPointCloud<PointT, Scalar> (cloud_in, indices, cloud_out, t));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    return (transformPointCloud<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform)
  {
    return (transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (transformPointCloudWithNormals<PointT, Scalar> (cloud_in, cloud_out, t));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices, cloud_out, t));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices, cloud_out, t));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform));
  }

  /** \brief Apply a rigid transform defined by a 3D offset and a quaternion
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] offset the translation component of the rigid transformation
    * \param[in] rotation the rotation component of the rigid transformation
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 3, 1> &offset, 
                       const Eigen::Quaternion<Scalar> &rotation);

  template <typename PointT> inline void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Vector3f &offset, 
                       const Eigen::Quaternionf &rotation)
  {
    return (transformPointCloud<PointT, float> (cloud_in, cloud_out, offset, rotation));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] offset the translation component of the rigid transformation
    * \param[in] rotation the rotation component of the rigid transformation
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 3, 1> &offset, 
                                  const Eigen::Quaternion<Scalar> &rotation);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Vector3f &offset, 
                                  const Eigen::Quaternionf &rotation)
  {
    return (transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, offset, rotation));
  }

  /** \brief Transform a point with members x,y,z
    * \param[in] point the point to transform
    * \param[out] transform the transformation to apply
    * \return the transformed point
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline PointT
  transformPoint (const PointT &point, 
                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);
  
  template <typename PointT> inline PointT
  transformPoint (const PointT &point, 
                  const Eigen::Affine3f &transform)
  {
    return (transformPoint<PointT, float> (point, transform));
  }

  /** \brief Calculates the principal (PCA-based) alignment of the point cloud
    * \param[in] cloud the input point cloud
    * \param[out] transform the resultant transform
    * \return the ratio lambda1/lambda2 or lambda2/lambda3, whatever is closer to 1.
    * \note If the return value is close to one then the transformation might be not unique -> two principal directions have
    * almost same variance (extend)
    */
  template <typename PointT, typename Scalar> inline double
  getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                              Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> inline double
  getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                              Eigen::Affine3f &transform)
  {
    return (getPrincipalTransformation<PointT, float> (cloud, transform));
  }
}

#include <pcl/common/impl/transforms.hpp>

#endif // PCL_TRANSFORMS_H_
