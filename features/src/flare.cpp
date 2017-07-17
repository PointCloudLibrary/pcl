/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2016-, Open Perception, Inc.
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
#include <pcl/features/impl/flare.hpp>


void
pcl::projectPointOnPlane ( Eigen::Vector3f const &point,
                           Eigen::Vector3f const &plane_point,
                           Eigen::Vector3f const &plane_normal,
                           Eigen::Vector3f &projected_point)
{
  const Eigen::Vector3f xo = point - plane_point;
  const float t = plane_normal.dot (xo);

  projected_point = point - (t * plane_normal);
}


void
pcl::directedOrthogonalAxis ( Eigen::Vector3f const &axis,
                              Eigen::Vector3f const &axis_origin,
                              Eigen::Vector3f const &point,
                              Eigen::Vector3f &directed_ortho_axis)
{
  Eigen::Vector3f projection;
  projectPointOnPlane (point, axis_origin, axis, projection);
  directed_ortho_axis = projection - axis_origin;

  directed_ortho_axis.normalize ();
}


void
pcl::randomOrthogonalAxis ( Eigen::Vector3f const &axis,
                            Eigen::Vector3f &rand_ortho_axis)
{
  if (std::abs (axis.z ()) > 1E-8f)
  {
    rand_ortho_axis.x () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.y () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = -(axis.x () * rand_ortho_axis.x () + axis.y () * rand_ortho_axis.y ()) / axis.z ();
  }
  else if (std::abs (axis.y ()) > 1E-8f)
  {
    rand_ortho_axis.x () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.y () = -(axis.x () * rand_ortho_axis.x () + axis.z () * rand_ortho_axis.z ()) / axis.y ();
  }
  else if (std::abs (axis.x ()) > 1E-8f)
  {
    rand_ortho_axis.y () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.x () = -(axis.y () * rand_ortho_axis.y () + axis.z () * rand_ortho_axis.z ()) / axis.x ();
  }
  else
  {
    PCL_WARN ("[pcl::randomOrthogonalAxis] provided axis has norm < 1E-8f");
  }

  rand_ortho_axis.normalize ();
}


#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE_PRODUCT(FLARELocalReferenceFrameEstimation, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA))((pcl::Normal))((pcl::ReferenceFrame))((float)(int)(short)))
#else
PCL_INSTANTIATE_PRODUCT(FLARELocalReferenceFrameEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES)((pcl::ReferenceFrame))((float)(int)(short)))
#endif
#endif    // PCL_NO_PRECOMPILE
