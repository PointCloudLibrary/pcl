/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/visibility_confidence.h>

pcl::ihs::Dome::Dome ()
{
  vertices_.col ( 0) = Eigen::Vector4f (-0.000000119f,  0.000000000f, 1.000000000f, 0.f);
  vertices_.col ( 1) = Eigen::Vector4f ( 0.894427180f,  0.000000000f, 0.447213739f, 0.f);
  vertices_.col ( 2) = Eigen::Vector4f ( 0.276393145f,  0.850650907f, 0.447213650f, 0.f);
  vertices_.col ( 3) = Eigen::Vector4f (-0.723606884f,  0.525731146f, 0.447213531f, 0.f);
  vertices_.col ( 4) = Eigen::Vector4f (-0.723606884f, -0.525731146f, 0.447213531f, 0.f);
  vertices_.col ( 5) = Eigen::Vector4f ( 0.276393145f, -0.850650907f, 0.447213650f, 0.f);
  vertices_.col ( 6) = Eigen::Vector4f ( 0.343278527f,  0.000000000f, 0.939233720f, 0.f);
  vertices_.col ( 7) = Eigen::Vector4f ( 0.686557174f,  0.000000000f, 0.727075875f, 0.f);
  vertices_.col ( 8) = Eigen::Vector4f ( 0.792636156f,  0.326477438f, 0.514918089f, 0.f);
  vertices_.col ( 9) = Eigen::Vector4f ( 0.555436373f,  0.652954817f, 0.514918029f, 0.f);
  vertices_.col (10) = Eigen::Vector4f ( 0.106078848f,  0.326477438f, 0.939233720f, 0.f);
  vertices_.col (11) = Eigen::Vector4f ( 0.212157741f,  0.652954817f, 0.727075756f, 0.f);
  vertices_.col (12) = Eigen::Vector4f (-0.065560505f,  0.854728878f, 0.514917910f, 0.f);
  vertices_.col (13) = Eigen::Vector4f (-0.449357629f,  0.730025530f, 0.514917850f, 0.f);
  vertices_.col (14) = Eigen::Vector4f (-0.277718395f,  0.201774135f, 0.939233661f, 0.f);
  vertices_.col (15) = Eigen::Vector4f (-0.555436671f,  0.403548241f, 0.727075696f, 0.f);
  vertices_.col (16) = Eigen::Vector4f (-0.833154857f,  0.201774105f, 0.514917850f, 0.f);
  vertices_.col (17) = Eigen::Vector4f (-0.833154857f, -0.201774150f, 0.514917850f, 0.f);
  vertices_.col (18) = Eigen::Vector4f (-0.277718395f, -0.201774135f, 0.939233661f, 0.f);
  vertices_.col (19) = Eigen::Vector4f (-0.555436671f, -0.403548241f, 0.727075696f, 0.f);
  vertices_.col (20) = Eigen::Vector4f (-0.449357659f, -0.730025649f, 0.514917910f, 0.f);
  vertices_.col (21) = Eigen::Vector4f (-0.065560460f, -0.854728937f, 0.514917850f, 0.f);
  vertices_.col (22) = Eigen::Vector4f ( 0.106078848f, -0.326477438f, 0.939233720f, 0.f);
  vertices_.col (23) = Eigen::Vector4f ( 0.212157741f, -0.652954817f, 0.727075756f, 0.f);
  vertices_.col (24) = Eigen::Vector4f ( 0.555436373f, -0.652954757f, 0.514917970f, 0.f);
  vertices_.col (25) = Eigen::Vector4f ( 0.792636156f, -0.326477349f, 0.514918089f, 0.f);
  vertices_.col (26) = Eigen::Vector4f ( 0.491123378f,  0.356822133f, 0.794654608f, 0.f);
  vertices_.col (27) = Eigen::Vector4f (-0.187592626f,  0.577350259f, 0.794654429f, 0.f);
  vertices_.col (28) = Eigen::Vector4f (-0.607062101f, -0.000000016f, 0.794654369f, 0.f);
  vertices_.col (29) = Eigen::Vector4f (-0.187592626f, -0.577350378f, 0.794654489f, 0.f);
  vertices_.col (30) = Eigen::Vector4f ( 0.491123348f, -0.356822133f, 0.794654548f, 0.f);

  for (unsigned int i=0; i<vertices_.cols (); ++i)
  {
    vertices_.col (i).head <3> ().normalize ();
  }
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::Dome::Vertices
pcl::ihs::Dome::getVertices () const
{
  return (vertices_);
}

////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    static const pcl::ihs::Dome dome;
  } // End namespace ihs
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::addDirection (const Eigen::Vector4f& normal,
                        const Eigen::Vector4f& direction,
                        uint32_t&              directions)
{
  // Find the rotation that aligns the normal with [0; 0; 1]
  const float dot = normal.z ();

  Eigen::Isometry3f R = Eigen::Isometry3f::Identity ();

  // No need to transform if the normal is already very close to [0; 0; 1] (also avoids numerical issues)
  // TODO: The threshold is hard coded for a frequency=3.
  //       It can be calculated with
  //       - max_z = maximum z value of the dome vertices (excluding [0; 0; 1])
  //       - thresh = cos (acos (max_z) / 2)
  //       - always round up!
  //       - with max_z = 0.939 -> thresh = 0.9847 ~ 0.985
  if (dot <= .985f)
  {
    const Eigen::Vector3f axis = Eigen::Vector3f (normal.y (), -normal.x (), 0.f).normalized ();
    R = Eigen::Isometry3f (Eigen::AngleAxisf (std::acos (dot), axis));
  }

  // Transform the direction into the dome coordinate system (which is aligned with the normal)
  Eigen::Vector4f aligned_direction = (R * direction);
  aligned_direction.head <3> ().normalize ();

  if (aligned_direction.z () < 0)
  {
    return;
  }

  // Find the closest viewing direction
  // NOTE: cos (0deg) = 1 = max
  //       acos (angle) = dot (a, b) / (norm (a) * norm (b)
  //       m_sphere_vertices are already normalized
  unsigned int index = 0;
  (aligned_direction.transpose () * pcl::ihs::dome.getVertices ()).maxCoeff (&index);

  // Set the observed direction bit at 'index'
  // http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c/47990#47990
  directions |= (1 << index);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int
pcl::ihs::countDirections (const uint32_t directions)
{
  // http://stackoverflow.com/questions/109023/best-algorithm-to-count-the-number-of-set-bits-in-a-32-bit-integer/109025#109025
  unsigned int i = directions - ((directions >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);

  return ((((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24);
}

////////////////////////////////////////////////////////////////////////////////
