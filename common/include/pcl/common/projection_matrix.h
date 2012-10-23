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
 */

#ifndef __PCL_ORGANIZED_PROJECTION_MATRIX_H__
#define __PCL_ORGANIZED_PROJECTION_MATRIX_H__

#include <pcl/common/eigen.h>
#include <pcl/console/print.h>

/**
  * \file common/geometry.h
  * Defines some geometrical functions and utility functions
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  template <typename T> class PointCloud;

  /** \brief Estimates the projection matrix P = K * (R|-R*t) from organized point clouds, with
    *        K = [[fx, s, cx], [0, fy, cy], [0, 0, 1]]
    *        R = rotation matrix and
    *        t = translation vector  
    * 
    * \param[in] cloud input cloud. Must be organized and from a projective device. e.g. stereo or kinect, ...
    * \param[out] projection_matrix output projection matrix
    * \param[in] indices The indices to be used to determine the projection matrix 
    * \return the resudial error. A high residual indicates, that the point cloud was not from a projective device.
    */
  template<typename PointT> double
  estimateProjectionMatrix (typename pcl::PointCloud<PointT>::ConstPtr cloud, Eigen::Matrix<float, 3, 4, Eigen::RowMajor>& projection_matrix, const std::vector<int>& indices = std::vector<int> ());
  
  /** \brief Determines the camera matrix from the given projection matrix.
    * \note This method does NOT use a RQ decomposition, but uses the fact that the left 3x3 matrix P' of P squared eliminates the rotational part.
    *       P' = K * R -> P' * P'^T = K * R * R^T * K = K * K^T
    * \param[in] projection_matrix
    * \param[out] camera_matrix
    */
  PCL_EXPORTS void
  getCameraMatrixFromProjectionMatrix (const Eigen::Matrix<float, 3, 4, Eigen::RowMajor>& projection_matrix, Eigen::Matrix3f& camera_matrix);  
}

#include <pcl/common/impl/projection_matrix.hpp>

#endif // __PCL_ORGANIZED_PROJECTION_MATRIX_H__
