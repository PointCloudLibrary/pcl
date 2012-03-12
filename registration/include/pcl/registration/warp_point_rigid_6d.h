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
 * $Id$
 *
 */


#ifndef PCL_WARP_POINT_RIGID_6D_H_
#define PCL_WARP_POINT_RIGID_6D_H_

#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{

  template <class PointSourceT, class PointTargetT>
  class WarpPointRigid6D : public WarpPointRigid<PointSourceT, PointTargetT>
  {
  public:
    WarpPointRigid6D ()
      : WarpPointRigid<PointSourceT, PointTargetT> (6) {}

    virtual void setParam (const Eigen::VectorXf& p)
    {
      assert(p.rows () == this->getDimension ());
      Eigen::Matrix4f& trans = this->transform_matrix_;      

      trans = Eigen::Matrix4f::Zero ();
      trans (3,3) = 1;

      // Copy the rotation and translation components
      trans.block <4, 1> (0, 3) = Eigen::Vector4f(p[0], p[1], p[2], 1.0);

      // Compute w from the unit quaternion
      Eigen::Quaternionf q (0, p[3], p[4], p[5]);
      q.w () = sqrt (1 - q.dot (q));
      trans.topLeftCorner<3, 3> () = q.toRotationMatrix();
    }
  };
}

#endif

