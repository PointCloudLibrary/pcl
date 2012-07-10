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

#ifndef PCL_WARP_POINT_RIGID_H_
#define PCL_WARP_POINT_RIGID_H_

#include <pcl/registration/eigen.h>

namespace pcl
{
  template <class PointSourceT, class PointTargetT>
  class WarpPointRigid
  {
  public:

    WarpPointRigid (int nr_dim): nr_dim_ (nr_dim), transform_matrix_ (Eigen::Matrix4f::Zero ())
    {
      transform_matrix_ (3,3) = 1.0;
    };

    virtual ~WarpPointRigid () {};

    virtual void 
    setParam (const Eigen::VectorXf& p) = 0;

    void 
    warpPoint (const PointSourceT& pnt_in, PointSourceT& pnt_out) const
    {
      pnt_out.getVector3fMap () = transform_matrix_.topLeftCorner<3, 3> () * pnt_in.getVector3fMap() + 
        transform_matrix_.block<3,1> (0, 3);
      pnt_out.data [3] = pnt_in.data [3];
    }

    int 
    getDimension () const {return nr_dim_;}

    const Eigen::Matrix4f& 
    getTransform () const { return transform_matrix_; }
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    int nr_dim_;
    Eigen::Matrix4f transform_matrix_;
  };

}

#endif
