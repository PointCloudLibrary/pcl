/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#ifndef _NURBS_DATA_H_
#define _NURBS_DATA_H_

#include <vector>
#include "eigen_defs.h"

namespace pcl
{
  namespace nurbs
  {

    struct NurbsData
    {
      Eigen::Matrix3d eigenvectors;
      Eigen::Vector3d mean;

      vector_vec3 interior; ///<<input
      std::vector<double> interior_error;
      vector_vec2 interior_param; ///>> output
      vector_vec3 interior_line_start; ///>> output
      vector_vec3 interior_line_end; ///>> output
      vector_vec3 interior_normals; ///>> output
      // http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html

      vector_vec3 boundary; ///<<input
      std::vector<double> boundary_error;
      vector_vec2 boundary_param; ///>> output
      vector_vec3 boundary_line_start; ///>> output
      vector_vec3 boundary_line_end; ///>> output
      vector_vec3 boundary_normals; ///>> output
      // http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html

      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      inline void
      clear_boundary ()
      {
        boundary.clear ();
        boundary_error.clear ();
        boundary_param.clear ();
        boundary_line_start.clear ();
        boundary_line_end.clear ();
        boundary_normals.clear ();
      }

    };
  } // namespace nurbs
} // namespace pcl

#endif

