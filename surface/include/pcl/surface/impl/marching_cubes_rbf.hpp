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
 */

#ifndef PCL_SURFACE_IMPL_MARCHING_CUBES_RBF_H_
#define PCL_SURFACE_IMPL_MARCHING_CUBES_RBF_H_

#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>
#include <pcl/kdtree/kdtree_flann.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesRBF<PointNT>::MarchingCubesRBF ()
  : MarchingCubes<PointNT> (),
    off_surface_epsilon_ (0.1f)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesRBF<PointNT>::~MarchingCubesRBF ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubesRBF<PointNT>::voxelizeData ()
{
  // Initialize data structures
  unsigned int N = input_->size ();
  Eigen::MatrixXd M (2*N, 2*N),
                  d (2*N, 1);


  for (unsigned int row_i = 0; row_i < 2*N; ++row_i)
  {
    // boolean variable to determine whether we are in the off_surface domain for the rows
    bool row_off = (row_i >= N) ? 1 : 0;
    for (unsigned int col_i = 0; col_i < 2*N; ++col_i)
    {
      // boolean variable to determine whether we are in the off_surface domain for the columns
      bool col_off = (col_i >= N) ? 1 : 0;
      M (row_i, col_i) = kernel (input_->points[col_i%N].getVector3fMap () + input_->points[col_i%N].getNormalVector3fMap () * col_off * off_surface_epsilon_,
                                 input_->points[row_i%N].getVector3fMap () + input_->points[row_i%N].getNormalVector3fMap () * row_off * off_surface_epsilon_);
    }

    d (row_i, 0) = row_off * off_surface_epsilon_;
  }

  // Solve for the weights
  Eigen::MatrixXd w (2*N, 1);

  // Solve_linear_system (M, d, w);
  w = M.fullPivLu ().solve (d);

  std::vector<float> weights_;
  std::vector<Eigen::Vector3f> centers_;
  weights_.resize (2*N);
  centers_.resize (2*N);
  for (unsigned int i = 0; i < N; ++i)
  {
    centers_[i] = input_->points[i].getVector3fMap ();
    centers_[i + N] = input_->points[i].getVector3fMap () + input_->points[i].getNormalVector3fMap () * off_surface_epsilon_;
    weights_[i] = w (i, 0);
    weights_[i + N] = w (i + N, 0);
  }



  for (int x = 0; x < res_x_; ++x)
    for (int y = 0; y < res_y_; ++y)
      for (int z = 0; z < res_z_; ++z)
      {
        Eigen::Vector3f point;
        point[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * x / res_x_;
        point[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * y / res_y_;
        point[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * z / res_z_;

        float f = 0.0f;
        std::vector<float>::const_iterator w_it (weights_.begin());
        for (std::vector<Eigen::Vector3f>::const_iterator c_it = centers_.begin ();
             c_it != centers_.end (); ++c_it, ++w_it)
          f += *w_it * kernel (*c_it, point);

        grid_[x * res_y_*res_z_ + y * res_z_ + z] = f;
      }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> float
pcl::MarchingCubesRBF<PointNT>::kernel (Eigen::Vector3f c, Eigen::Vector3f x)
{
  float r = (x - c).norm();
  return r*r*r;
}



#define PCL_INSTANTIATE_MarchingCubesRBF(T) template class PCL_EXPORTS pcl::MarchingCubesRBF<T>;

#endif    // PCL_SURFACE_IMPL_MARCHING_CUBES_HOPPE_H_

