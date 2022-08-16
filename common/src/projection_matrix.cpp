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

#include <pcl/common/projection_matrix.h>

//////////////////////////////////////////////////////////////////////////////
void 
pcl::getCameraMatrixFromProjectionMatrix (
    const Eigen::Matrix<float, 3, 4, Eigen::RowMajor>& projection_matrix, 
    Eigen::Matrix3f& camera_matrix)
{
  Eigen::Matrix3f KR = projection_matrix.topLeftCorner <3, 3> ();

  Eigen::Matrix3f KR_KRT = KR * KR.transpose ();
  
  Eigen::Matrix3f cam = KR_KRT / KR_KRT.coeff (8);

  camera_matrix.setZero();
  camera_matrix.coeffRef (8) = 1.0;
  
  if (camera_matrix.Flags & Eigen::RowMajorBit)
  {
    camera_matrix.coeffRef (2) = cam.coeff (2);
    camera_matrix.coeffRef (5) = cam.coeff (5);
    camera_matrix.coeffRef (4) = static_cast<float> (std::sqrt (cam.coeff (4) - cam.coeff (5) * cam.coeff (5)));
    camera_matrix.coeffRef (1) = (cam.coeff (1) - cam.coeff (2) * cam.coeff (5)) / camera_matrix.coeff (4);
    camera_matrix.coeffRef (0) = static_cast<float> (std::sqrt (cam.coeff (0) - camera_matrix.coeff (1) * camera_matrix.coeff (1) - cam.coeff (2) * cam.coeff (2)));
  }
  else
  {
    camera_matrix.coeffRef (6) = cam.coeff (2);
    camera_matrix.coeffRef (7) = cam.coeff (5);
    camera_matrix.coeffRef (4) = static_cast<float> (std::sqrt (cam.coeff (4) - cam.coeff (5) * cam.coeff (5)));
    camera_matrix.coeffRef (3) = (cam.coeff (1) - cam.coeff (2) * cam.coeff (5)) / camera_matrix.coeff (4);
    camera_matrix.coeffRef (0) = static_cast<float> (std::sqrt (cam.coeff (0) - camera_matrix.coeff (3) * camera_matrix.coeff (3) - cam.coeff (2) * cam.coeff (2)));
  }
}

