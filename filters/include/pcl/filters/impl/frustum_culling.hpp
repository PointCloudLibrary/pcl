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

#ifndef PCL_FILTERS_IMPL_FRUSTUM_CULLING_HPP_
#define PCL_FILTERS_IMPL_FRUSTUM_CULLING_HPP_

#include <pcl/filters/frustum_culling.h>
#include <vector>

///////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::FrustumCulling<PointT>::applyFilter (Indices &indices)
{
  Eigen::Vector4f pl_n; // near plane 
  Eigen::Vector4f pl_f; // far plane
  Eigen::Vector4f pl_t; // top plane
  Eigen::Vector4f pl_b; // bottom plane
  Eigen::Vector4f pl_r; // right plane
  Eigen::Vector4f pl_l; // left plane

  Eigen::Vector3f view = camera_pose_.block<3, 1> (0, 0);    // view vector for the camera  - first column of the rotation matrix
  Eigen::Vector3f up = camera_pose_.block<3, 1> (0, 1);      // up vector for the camera    - second column of the rotation matrix
  Eigen::Vector3f right = camera_pose_.block<3, 1> (0, 2);   // right vector for the camera - third column of the rotation matrix
  Eigen::Vector3f T = camera_pose_.block<3, 1> (0, 3);       // The (X, Y, Z) position of the camera w.r.t origin


  float vfov_rad = float (vfov_ * M_PI / 180);  // degrees to radians
  float hfov_rad = float (hfov_ * M_PI / 180);  // degrees to radians
  
  float np_h = float (2 * tan (vfov_rad / 2) * np_dist_);  // near plane height
  float np_w = float (2 * tan (hfov_rad / 2) * np_dist_);  // near plane width

  float fp_h = float (2 * tan (vfov_rad / 2) * fp_dist_);  // far plane height
  float fp_w = float (2 * tan (hfov_rad / 2) * fp_dist_);  // far plane width

  Eigen::Vector3f fp_c (T + view * fp_dist_);                           // far plane center
  Eigen::Vector3f fp_tl (fp_c + (up * fp_h / 2) - (right * fp_w / 2));  // Top left corner of the far plane
  Eigen::Vector3f fp_tr (fp_c + (up * fp_h / 2) + (right * fp_w / 2));  // Top right corner of the far plane
  Eigen::Vector3f fp_bl (fp_c - (up * fp_h / 2) - (right * fp_w / 2));  // Bottom left corner of the far plane
  Eigen::Vector3f fp_br (fp_c - (up * fp_h / 2) + (right * fp_w / 2));  // Bottom right corner of the far plane

  Eigen::Vector3f np_c (T + view * np_dist_);                           // near plane center
  //Eigen::Vector3f np_tl = np_c + (up * np_h/2) - (right * np_w/2);    // Top left corner of the near plane
  Eigen::Vector3f np_tr (np_c + (up * np_h / 2) + (right * np_w / 2));  // Top right corner of the near plane
  Eigen::Vector3f np_bl (np_c - (up * np_h / 2) - (right * np_w / 2));  // Bottom left corner of the near plane
  Eigen::Vector3f np_br (np_c - (up * np_h / 2) + (right * np_w / 2));  // Bottom right corner of the near plane

  pl_f.head<3> () = (fp_bl - fp_br).cross (fp_tr - fp_br);  // Far plane equation - cross product of the 
  pl_f (3) = -fp_c.dot (pl_f.head<3> ());                   // perpendicular edges of the far plane

  pl_n.head<3> () = (np_tr - np_br).cross (np_bl - np_br);  // Near plane equation - cross product of the 
  pl_n (3) = -np_c.dot (pl_n.head<3> ());                   // perpendicular edges of the far plane

  Eigen::Vector3f a (fp_bl - T);  // Vector connecting the camera and far plane bottom left
  Eigen::Vector3f b (fp_br - T);  // Vector connecting the camera and far plane bottom right
  Eigen::Vector3f c (fp_tr - T);  // Vector connecting the camera and far plane top right
  Eigen::Vector3f d (fp_tl - T);  // Vector connecting the camera and far plane top left

  //                   Frustum and the vectors a, b, c and d. T is the position of the camera
  //                             _________
  //                           /|       . |
  //                       d  / |   c .   |
  //                         /  | __._____| 
  //                        /  /  .      .
  //                 a <---/-/  .    .
  //                      / / .   .  b
  //                     /   .
  //                     . 
  //                   T
  //

  pl_r.head<3> () = b.cross (c);
  pl_l.head<3> () = d.cross (a);
  pl_t.head<3> () = c.cross (d);
  pl_b.head<3> () = a.cross (b);

  pl_r (3) = -T.dot (pl_r.head<3> ());
  pl_l (3) = -T.dot (pl_l.head<3> ());
  pl_t (3) = -T.dot (pl_t.head<3> ());
  pl_b (3) = -T.dot (pl_b.head<3> ());

  if (extract_removed_indices_)
  {
    removed_indices_->resize (indices_->size ());
  }
  indices.resize (indices_->size ());
  std::size_t indices_ctr = 0;
  std::size_t removed_ctr = 0;
  for (std::size_t i = 0; i < indices_->size (); i++) 
  {
    int idx = indices_->at (i);
    Eigen::Vector4f pt ((*input_)[idx].x,
                        (*input_)[idx].y,
                        (*input_)[idx].z,
                        1.0f);
    bool is_in_fov = (pt.dot (pl_l) <= 0) && 
                     (pt.dot (pl_r) <= 0) &&
                     (pt.dot (pl_t) <= 0) && 
                     (pt.dot (pl_b) <= 0) && 
                     (pt.dot (pl_f) <= 0) &&
                     (pt.dot (pl_n) <= 0);
    if (is_in_fov ^ negative_)
    {
      indices[indices_ctr++] = idx;
    }
    else if (extract_removed_indices_)
    {
      (*removed_indices_)[removed_ctr++] = idx;
    }
  }
  indices.resize (indices_ctr);
  removed_indices_->resize (removed_ctr);
}

#define PCL_INSTANTIATE_FrustumCulling(T) template class PCL_EXPORTS pcl::FrustumCulling<T>;

#endif
