/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Eigen::Matrix4f &transformation_matrix)
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationSVD::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  // Approximate as a linear least squares problem
  Eigen::MatrixXf A (nr_points, 6);
  Eigen::MatrixXf b (nr_points, 1);
  for (size_t i = 0; i < nr_points; ++i)
  {
    const float & sx = cloud_src.points[i].x;
    const float & sy = cloud_src.points[i].y;
    const float & sz = cloud_src.points[i].z;
    const float & dx = cloud_tgt.points[i].x;
    const float & dy = cloud_tgt.points[i].y;
    const float & dz = cloud_tgt.points[i].z;
    const float & nx = cloud_tgt.points[i].normal[0];
    const float & ny = cloud_tgt.points[i].normal[1];
    const float & nz = cloud_tgt.points[i].normal[2];
    A (i, 0) = nz*sy - ny*sz;
    A (i, 1) = nx*sz - nz*sx; 
    A (i, 2) = ny*sx - nx*sy;
    A (i, 3) = nx;
    A (i, 4) = ny;
    A (i, 5) = nz;
    b (i, 0) = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
  }

  // Solve A*x = b
  Eigen::VectorXf x = A.colPivHouseholderQr ().solve (b);
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
 
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Eigen::Matrix4f &transformation_matrix)
{
  size_t nr_points = indices_src.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationSVD::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  // Approximate as a linear least squares problem
  Eigen::MatrixXf A (nr_points, 6);
  Eigen::MatrixXf b (nr_points, 1);
  for (size_t i = 0; i < nr_points; ++i)
  {
    const float & sx = cloud_src.points[indices_src[i]].x;
    const float & sy = cloud_src.points[indices_src[i]].y;
    const float & sz = cloud_src.points[indices_src[i]].z;
    const float & dx = cloud_tgt.points[i].x;
    const float & dy = cloud_tgt.points[i].y;
    const float & dz = cloud_tgt.points[i].z;
    const float & nx = cloud_tgt.points[i].normal[0];
    const float & ny = cloud_tgt.points[i].normal[1];
    const float & nz = cloud_tgt.points[i].normal[2];
    A (i, 0) = nz*sy - ny*sz;
    A (i, 1) = nx*sz - nz*sx; 
    A (i, 2) = ny*sx - nx*sy;
    A (i, 3) = nx;
    A (i, 4) = ny;
    A (i, 5) = nz;
    b (i, 0) = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
  }

  // Solve A*x = b
  Eigen::VectorXf x = A.colPivHouseholderQr ().solve (b);
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const std::vector<int> &indices_tgt,
                             Eigen::Matrix4f &transformation_matrix)
{
  size_t nr_points = indices_src.size ();
  if (indices_tgt.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  // Approximate as a linear least squares problem
  Eigen::MatrixXf A (nr_points, 6);
  Eigen::MatrixXf b (nr_points, 1);
  for (size_t i = 0; i < nr_points; ++i)
  {
    const float & sx = cloud_src.points[indices_src[i]].x;
    const float & sy = cloud_src.points[indices_src[i]].y;
    const float & sz = cloud_src.points[indices_src[i]].z;
    const float & dx = cloud_tgt.points[indices_tgt[i]].x;
    const float & dy = cloud_tgt.points[indices_tgt[i]].y;
    const float & dz = cloud_tgt.points[indices_tgt[i]].z;
    const float & nx = cloud_tgt.points[indices_tgt[i]].normal[0];
    const float & ny = cloud_tgt.points[indices_tgt[i]].normal[1];
    const float & nz = cloud_tgt.points[indices_tgt[i]].normal[2];
    A (i, 0) = nz*sy - ny*sz;
    A (i, 1) = nx*sz - nz*sx; 
    A (i, 2) = ny*sx - nx*sy;
    A (i, 3) = nx;
    A (i, 4) = ny;
    A (i, 5) = nz;
    b (i, 0) = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
  }

  // Solve A*x = b
  Eigen::VectorXf x = A.colPivHouseholderQr ().solve (b);
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const pcl::Correspondences &correspondences,
                             Eigen::Matrix4f &transformation_matrix)
{
  size_t nr_points = correspondences.size ();

  // Approximate as a linear least squares problem
  Eigen::MatrixXf A (nr_points, 6);
  Eigen::MatrixXf b (nr_points, 1);
  for (size_t i = 0; i < nr_points; ++i)
  {
    const int & src_idx = correspondences[i].index_query;
    const int & tgt_idx = correspondences[i].index_match;
    const float & sx = cloud_src.points[src_idx].x;
    const float & sy = cloud_src.points[src_idx].y;
    const float & sz = cloud_src.points[src_idx].z;
    const float & dx = cloud_tgt.points[tgt_idx].x;
    const float & dy = cloud_tgt.points[tgt_idx].y;
    const float & dz = cloud_tgt.points[tgt_idx].z;
    const float & nx = cloud_tgt.points[tgt_idx].normal[0];
    const float & ny = cloud_tgt.points[tgt_idx].normal[1];
    const float & nz = cloud_tgt.points[tgt_idx].normal[2];
    A (i, 0) = nz*sy - ny*sz;
    A (i, 1) = nx*sz - nz*sx; 
    A (i, 2) = ny*sx - nx*sy;
    A (i, 3) = nx;
    A (i, 4) = ny;
    A (i, 5) = nz;
    b (i, 0) = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
  }

  // Solve A*x = b
  Eigen::VectorXf x = A.colPivHouseholderQr ().solve (b);
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
}

template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget>::
constructTransformationMatrix (const float & alpha, const float & beta, const float & gamma,
                               const float & tx, const float & ty, const float & tz,
                               Eigen::Matrix4f &transformation_matrix)
{

  // Construct the transformation matrix from rotation and translation 
  transformation_matrix = Eigen::Matrix4f::Zero ();
  transformation_matrix (0, 0) =  cosf (gamma) * cosf (beta);
  transformation_matrix (0, 1) = -sinf (gamma) * cosf (alpha) + cosf (gamma) * sinf (beta) * sinf (alpha);
  transformation_matrix (0, 2) =  sinf (gamma) * sinf (alpha) + cosf (gamma) * sinf (beta) * cosf (alpha);
  transformation_matrix (1, 0) =  sinf (gamma) * cosf (beta);
  transformation_matrix (1, 1) =  cosf (gamma) * cosf (alpha) + sinf (gamma) * sinf (beta) * sinf (alpha);
  transformation_matrix (1, 2) = -cosf (gamma) * sinf (alpha) + sinf (gamma) * sinf (beta) * cosf (alpha);
  transformation_matrix (2, 0) = -sinf (beta);
  transformation_matrix (2, 1) =  cosf (beta) * sinf (alpha);
  transformation_matrix (2, 2) =  cosf (beta) * cosf (alpha);

  transformation_matrix (0, 3) = tx;
  transformation_matrix (1, 3) = ty;
  transformation_matrix (2, 3) = tz;
  transformation_matrix (3, 3) = 1;
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_ */
