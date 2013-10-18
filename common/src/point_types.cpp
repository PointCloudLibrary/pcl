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
#include <pcl/point_types.h>

namespace pcl
{
  std::ostream& 
  operator << (std::ostream& os, const PointXYZ& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const RGB& p)
  {
    os << "("
      << static_cast<int>(p.r) << ","
      << static_cast<int>(p.g) << ","
      << static_cast<int>(p.b) << ","
      << static_cast<int>(p.a) << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const Intensity& p)
  {
    os << "( " << static_cast<int>(p.intensity) << " )";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const Intensity8u& p)
  {
    os << "( " << static_cast<int>(p.intensity) << " )";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PointXYZI& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZL& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.label << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const Label& p)
  {
    os << "(" << p.label << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PointXYZRGBA& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - "
      << static_cast<int>(p.r) << ","
      << static_cast<int>(p.g) << ","
      << static_cast<int>(p.b) << ","
      << static_cast<int>(p.a) << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZRGB& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - "
      << static_cast<int>(p.r) << ","
      << static_cast<int>(p.g) << ","
      << static_cast<int>(p.b) << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZRGBL& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.r << "," << p.g << "," << p.b << " - " << p.label << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZHSV& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.h << " , " <<  p.s << " , " << p.v << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXY& p)
  {
    os << "(" << p.x << "," << p.y << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointUV& p)
  {
    os << "(" << p.u << "," << p.v << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const InterestPoint& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.strength << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const Normal& p)
  {
    os << "(" << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const Axis& p)
  {
    os << "(" << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << ")";
    return os;
  }

  std::ostream& 
  operator << (std::ostream& os, const _Axis& p)
  {
    os << "(" << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << ")";
    return os;
  }

  std::ostream& 
  operator << (std::ostream& os, const PointNormal& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZRGBNormal& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.r << ", " << p.g << ", " << p.b << " - " << p.curvature << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointXYZINormal& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointWithRange& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.range << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointWithViewpoint& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.vp_x << "," << p.vp_y << "," << p.vp_z << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const MomentInvariants& p)
  {
    os << "(" << p.j1 << "," << p.j2 << "," << p.j3 << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PrincipalRadiiRSD& p)
  {
    os << "(" << p.r_min << "," << p.r_max << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const Boundary& p)
  {
    os << p.boundary_point;
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PrincipalCurvatures& p)
  {
    os << "(" << p.principal_curvature[0] << "," << p.principal_curvature[1] << "," << p.principal_curvature[2] << " - " << p.pc1 << "," << p.pc2 << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PFHSignature125& p)
  {
    for (int i = 0; i < 125; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 124 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PFHRGBSignature250& p)
  {
    for (int i = 0; i < 250; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 249 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PPFSignature& p)
  {
    os << "(" << p.f1 << ", " << p.f2 << ", " << p.f3 << ", " << p.f4 << ", " << p.alpha_m << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const CPPFSignature& p)
  {
    os << "(" << p.f1 << ", " << p.f2 << ", " << p.f3 << ", " << p.f4 << ", " << p.f5 << ", " << p.f6 << ", " << p.f7 << ", " << p.f8 << ", " << p.f9 << ", " << p.f10 << ", " << p.alpha_m << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PPFRGBSignature& p)
   {
     os << "(" << p.f1 << ", " << p.f2 << ", " << p.f3 << ", " << p.f4 << ", " <<
         p.r_ratio << ", " << p.g_ratio << ", " << p.b_ratio << ", " << p.alpha_m << ")";
     return (os);
   }

  std::ostream& 
  operator << (std::ostream& os, const NormalBasedSignature12& p)
  {
    for (int i = 0; i < 12; ++i)
    os << (i == 0 ? "(" : "") << p.values[i] << (i < 11 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const ShapeContext1980& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (size_t i = 0; i < 1980; ++i)
      os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 1979 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const SHOT352& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (size_t i = 0; i < 352; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 351 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const SHOT1344& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (size_t i = 0; i < 1344; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 1343 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const ReferenceFrame& p)
  {
    os << "("
       << p.x_axis[0] << " " << p.x_axis[1] << " " << p.x_axis[2] << ","
       << p.y_axis[0] << " " << p.y_axis[1] << " " << p.y_axis[2] << ","
       << p.z_axis[0] << " " << p.z_axis[1] << " " << p.z_axis[2] << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const FPFHSignature33& p)
  {
    for (int i = 0; i < 33; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 32 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const VFHSignature308& p)
  {
    for (int i = 0; i < 308; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 307 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const BRISKSignature512& p)
  {
    os << p.scale << " " << p.orientation << " ";
    for (int i = 0; i < 64; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 63 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const ESFSignature640& p)
  {
    for (int i = 0; i < 640; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 639 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const GFPFHSignature16& p)
  {
    for (int i = 0; i < p.descriptorSize (); ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < (p.descriptorSize () - 1) ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const Narf36& p)
  {
    os << p.x<<","<<p.y<<","<<p.z<<" - "<<p.roll*360.0/M_PI<<"deg,"<<p.pitch*360.0/M_PI<<"deg,"<<p.yaw*360.0/M_PI<<"deg - ";
    for (int i = 0; i < 36; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 35 ? ", " : ")");
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const BorderDescription& p)
  {
    os << "(" << p.x << "," << p.y << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const IntensityGradient& p)
  {
    os << "(" << p.gradient[0] << "," << p.gradient[1] << "," << p.gradient[2] << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointWithScale& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.scale << "," << p.angle << "," << p.response << "," << p.octave << ")";
    return (os);
  }

  std::ostream& 
  operator << (std::ostream& os, const PointSurfel& p)
  {
    const unsigned char* rgba_ptr = reinterpret_cast<const unsigned char*>(&p.rgba);
    os <<
    "(" << p.x << "," << p.y << "," << p.z << " - " <<
    p.normal_x << "," << p.normal_y << "," << p.normal_z << " - "
    << static_cast<int>(*rgba_ptr) << ","
    << static_cast<int>(*(rgba_ptr+1)) << ","
    << static_cast<int>(*(rgba_ptr+2)) << ","
    << static_cast<int>(*(rgba_ptr+3)) << " - " <<
    p.radius << " - " << p.confidence << " - " << p.curvature << ")";
    return (os);
  }
}
