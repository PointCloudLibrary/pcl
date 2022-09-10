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

/// @file trackball.cpp
/// @details Generic object for generating rotations given mouse input.  This
/// class has been based on
/// @author Matthew Hielsberg

#include <limits>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/trackball.h>

TrackBall::TrackBall() : quat_(1.0f), origin_x_(0), origin_y_(0), origin_z_(0)
{
  radius_sqr_ = (TRACKBALL_RADIUS_SCALE * static_cast<float>(WINDOW_WIDTH)) *
                (TRACKBALL_RADIUS_SCALE * static_cast<float>(WINDOW_WIDTH));
}

TrackBall::TrackBall(const TrackBall &copy)  
= default;

TrackBall::~TrackBall()
= default;

TrackBall&
TrackBall::operator=(const TrackBall &rhs)
= default;

void
TrackBall::start(int s_x, int s_y)
{
  getPointFromScreenPoint(s_x, s_y, origin_x_, origin_y_, origin_z_);
}

void
normalize(float x, float y, float z, float &nx, float &ny, float &nz)
{
    float inv_len = 1.0f / std::sqrt(x * x + y * y + z * z);
    nx = x * inv_len;
    ny = y * inv_len;
    nz = z * inv_len;
}

boost::math::quaternion<float>
normalizeQuaternion(const boost::math::quaternion<float> &q)
{
    float w = q.R_component_1();
    float x = q.R_component_2();
    float y = q.R_component_3();
    float z = q.R_component_4();
    float inv_len = 1.0f / std::sqrt(w * w + x * x + y * y + z * z);
    return boost::math::quaternion<float>(w * inv_len, x * inv_len, y * inv_len,
                                          z * inv_len);
}

boost::math::quaternion<float>
quaternionFromAngleAxis(float angle, float x, float y, float z)
{
  float s = std::sin(0.5f * angle);
  float qw = std::cos(0.5f * angle);
  float qx = x * s;
  float qy = y * s;
  float qz = z * s;
  return normalizeQuaternion(boost::math::quaternion<float>(qw, qx, qy, qz));
}

boost::math::quaternion<float>
multiplyQuaternion(const boost::math::quaternion<float> &lhs,
                   const boost::math::quaternion<float> &rhs)
{
    float lw = lhs.R_component_1();
    float lx = lhs.R_component_2();
    float ly = lhs.R_component_3();
    float lz = lhs.R_component_4();
    
    float rw = rhs.R_component_1();
    float rx = rhs.R_component_2();
    float ry = rhs.R_component_3();
    float rz = rhs.R_component_4();
    
    float tw = lw * rw - lx * rx - ly * ry - lz * rz;
    float tx = lw * rx + lx * rw - ly * rz + lz * ry;
    float ty = lw * ry + lx * rz + ly * rw - lz * rx;
    float tz = lw * rz - lx * ry + ly * rx + lz * rw;
    
    return boost::math::quaternion<float>(tw, tx, ty, tz);
}

void
TrackBall::update(int s_x, int s_y)
{
  float cur_x, cur_y, cur_z;
  getPointFromScreenPoint(s_x, s_y, cur_x, cur_y, cur_z);
    
  float d_x = cur_x - origin_x_;
  float d_y = cur_y - origin_y_;
  float d_z = cur_z - origin_z_;

  float dot = d_x * d_x + d_y * d_y + d_z * d_z;
  if (dot < std::numeric_limits<float>::epsilon())
  {
    quat_ = boost::math::quaternion<float>(1.0f);
    return;
  }
  float nc_x, nc_y, nc_z;
  float no_x, no_y, no_z;
  normalize(cur_x, cur_y, cur_z, nc_x, nc_y, nc_z);
  normalize(origin_x_, origin_y_, origin_z_, no_x, no_y, no_z);
    
  // compute the angle of rotation
  float angle = std::acos(nc_x * no_x + nc_y * no_y + nc_z * no_z);

  // compute the axis of rotation
  float cross_x = nc_y * no_z - nc_z * no_y;
  float cross_y = nc_z * no_x - nc_x * no_z;
  float cross_z = nc_x * no_y - nc_y * no_x;
    
  // reuse of nc_*
  normalize(cross_x, cross_y, cross_z, nc_x, nc_y, nc_z);

  quat_ = quaternionFromAngleAxis(angle, nc_x, nc_y, nc_z);
  if (std::isnan(quat_.R_component_1()))
    quat_ = boost::math::quaternion<float>(1.0f);

  origin_x_ = cur_x;
  origin_y_ = cur_y;
  origin_z_ = cur_z;
}

void
TrackBall::getRotationMatrix(float (&rot)[MATRIX_SIZE])
{
  // This function is based on quaternion_to_R3_rotation from
  // http://www.boost.org/doc/libs/1_41_0/libs/math/quaternion/HSO3.hpp
    
  float a = quat_.R_component_1();
  float b = quat_.R_component_2();
  float c = quat_.R_component_3();
  float d = quat_.R_component_4();
    
  float aa = a*a;
  float ab = a*b;
  float ac = a*c;
  float ad = a*d;
  float bb = b*b;
  float bc = b*c;
  float bd = b*d;
  float cc = c*c;
  float cd = c*d;
  float dd = d*d;
    
  setIdentity(rot);
  float n = aa + bb + cc + dd;
    
  if (n <= std::numeric_limits<float>::epsilon())
    return;

  // fill the upper 3x3
  rot[0] = (aa + bb - cc - dd);
  rot[1] = 2 * (-ad + bc);
  rot[2] = 2 * (ac + bd);
  rot[MATRIX_SIZE_DIM+0] = 2 * (ad + bc);
  rot[MATRIX_SIZE_DIM+1] = (aa - bb + cc - dd);
  rot[MATRIX_SIZE_DIM+2] = 2 * (-ab + cd);
  rot[2*MATRIX_SIZE_DIM+0] = 2 * (-ac + bd);
  rot[2*MATRIX_SIZE_DIM+1] = 2 * (ab + cd);
  rot[2*MATRIX_SIZE_DIM+2] = (aa - bb - cc + dd);
}

void
TrackBall::reset()
{
  quat_ = boost::math::quaternion<float>(1.0f);
}

void
TrackBall::getPointFromScreenPoint(int s_x, int s_y,
                                   float &x, float &y, float &z) const
{
  // See http://www.opengl.org/wiki/Trackball for more info
    
  x = static_cast<float>(s_x) - (static_cast<float>(WINDOW_WIDTH) * 0.5f);
  y = (static_cast<float>(WINDOW_HEIGHT) * 0.5f) - static_cast<float>(s_y);
  float d = x * x + y * y;
  if (d > 0.5f * radius_sqr_)
  {
    // use hyperbolic sheet
    z = (0.5f * radius_sqr_) / std::sqrt(d);
    return;
  }
  // use sphere
  z = std::sqrt(radius_sqr_ - d);
}

