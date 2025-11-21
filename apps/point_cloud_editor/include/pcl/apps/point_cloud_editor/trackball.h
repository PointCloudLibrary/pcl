///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file trackball.h
/// @details Generic object for generating rotations given mouse input.  This
/// class has been based on 
/// @author Matthew Hielsberg

#pragma once

#include <boost/math/quaternion.hpp>
#include <pcl/apps/point_cloud_editor/localTypes.h>

class TrackBall
{
  public:
    TrackBall();
    TrackBall(const TrackBall &copy);
    ~TrackBall();
    
    TrackBall& operator=(const TrackBall &rhs);
    
    void start(int s_x, int s_y);
    
    void update(int s_x, int s_y);
    
    void getRotationMatrix(float (&rot)[MATRIX_SIZE]);
    
    void reset();
    
  private:
    
    void getPointFromScreenPoint(int s_x, int s_y, float &x, float &y, float &z) const;

    /// the quaternion representing the current orientation of the trackball
    boost::math::quaternion<float> quat_;
    
    /// the original mouse screen coordinates converted to a 3d point
    float origin_x_, origin_y_, origin_z_;
    
    /// the radius of the trackball squared
    float radius_sqr_;
        
}; // class TrackBall
