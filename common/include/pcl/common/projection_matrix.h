#ifndef __PCL_ORGANIZED_PROJECTION_MATRIX_H__
#define __PCL_ORGANIZED_PROJECTION_MATRIX_H__
#include <pcl/common/eigen.h>

/**
  * \file geometry.h
  * Defines some geometrical functions and utility functions
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief estimates the projection matrix P = K * (R|-R*t) from organized point clouds, with
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
  
  /** \brief determines the camera matrix from the given projection matrix.
    * \note This method does NOT use a RQ decomposition, but uses the fact that the left 3x3 matrix P' of P squared eliminates the rotational part.
    *       P' = K * R -> P' * P'^T = K * R * R^T * K = K * K^T
    * \param[in] projection_matrix
    * \param[out] camera_matrix
    */
  PCL_EXPORTS inline void
  getCameraMatrixFromProjectionMatrix (const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> projection_matrix, Eigen::Matrix3f& camera_matrix);  
}

#include "impl/projection_matrix.hpp"
#endif // __PCL_ORGANIZED_PROJECTION_MATRIX_H__