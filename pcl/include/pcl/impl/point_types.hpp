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
 * $Id: point_types.hpp 35899 2011-02-09 09:17:49Z magnenat $
 *
 */

#ifndef PCL_IMPL_POINT_TYPES_HPP_
#define PCL_IMPL_POINT_TYPES_HPP_

// Define all PCL point types
#define PCL_POINT_TYPES       \
  (pcl::PointXYZ)             \
  (pcl::PointXYZI)            \
  (pcl::PointXYZRGBA)         \
  (pcl::PointXYZRGB)          \
  (pcl::PointXY)              \
  (pcl::InterestPoint)        \
  (pcl::Normal)               \
  (pcl::PointNormal)          \
  (pcl::PointXYZRGBNormal)    \
  (pcl::PointXYZINormal)      \
  (pcl::PointWithRange)       \
  (pcl::PointWithViewpoint)   \
  (pcl::MomentInvariants)     \
  (pcl::PrincipalRadiiRSD)    \
  (pcl::Boundary)             \
  (pcl::PrincipalCurvatures)  \
  (pcl::PFHSignature125)      \
  (pcl::FPFHSignature33)      \
  (pcl::VFHSignature308)      \
  (pcl::Narf36)               \
  (pcl::BorderDescription)    \
  (pcl::IntensityGradient)    \
  (pcl::Histogram<2>)         \
  (pcl::PointWithScale)

// Define all point types that include XYZ data
#define PCL_XYZ_POINT_TYPES   \
  (pcl::PointXYZ)             \
  (pcl::PointXYZI)            \
  (pcl::PointXYZRGBA)         \
  (pcl::PointXYZRGB)          \
  (pcl::InterestPoint)        \
  (pcl::PointNormal)          \
  (pcl::PointXYZRGBNormal)    \
  (pcl::PointXYZINormal)      \
  (pcl::PointWithRange)       \
  (pcl::PointWithViewpoint)   \
  (pcl::PointWithScale)

// Define all point types that include normal[3] data
#define PCL_NORMAL_POINT_TYPES  \
  (pcl::Normal)                 \
  (pcl::PointNormal)            \
  (pcl::PointXYZRGBNormal)      \
  (pcl::PointXYZINormal)

namespace pcl
{

#define PCL_ADD_POINT4D \
  union { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  } EIGEN_ALIGN16; \
  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

#define PCL_ADD_NORMAL4D \
  union { \
    float data_n[4]; \
    float normal[3]; \
    struct { \
      float normal_x; \
      float normal_y; \
      float normal_z; \
    }; \
  } EIGEN_ALIGN16; \
  inline Eigen::Map<Eigen::Vector3f> getNormalVector3fMap () { return (Eigen::Vector3f::Map (data_n)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getNormalVector3fMap () const { return (Eigen::Vector3f::Map (data_n)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getNormalVector4fMap () { return (Eigen::Vector4f::MapAligned (data_n)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getNormalVector4fMap () const { return (Eigen::Vector4f::MapAligned (data_n)); }

typedef Eigen::Map<Eigen::Array3f> Array3fMap;
typedef const Eigen::Map<const Eigen::Array3f> Array3fMapConst;
typedef Eigen::Map<Eigen::Array4f, Eigen::Aligned> Array4fMap;
typedef const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> Array4fMapConst;
typedef Eigen::Map<Eigen::Vector3f> Vector3fMap;
typedef const Eigen::Map<const Eigen::Vector3f> Vector3fMapConst;
typedef Eigen::Map<Eigen::Vector4f, Eigen::Aligned> Vector4fMap;
typedef const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> Vector4fMapConst;


struct _PointXYZ
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

/*struct PointXYZ
{
  ADD_4D_POINT_WITH_XYZ;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  //inline PointXYZ() {}
  //inline PointXYZ(float x, float y, float z) : x(x), y(y), z(z) {}
};*/
struct PointXYZ : public _PointXYZ
{
  inline PointXYZ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
  }
  inline PointXYZ (float _x, float _y, float _z) { x = _x; y = _y; z = _z; data[3] = 1.0f; }
};

inline std::ostream& operator << (std::ostream& os, const PointXYZ& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << ")";
  return (os);
}

struct PointXYZI 
{ 
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float intensity; 
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16; 
inline std::ostream& operator << (std::ostream& os, const PointXYZI& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << ")";
  return (os);
}


struct PointXYZRGBA
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      uint32_t rgba;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointXYZRGBA& p)
{
  unsigned char* rgba_ptr = (unsigned char*)&p.rgba;
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << (int)(*rgba_ptr) << "," << (int)(*(rgba_ptr+1)) << "," << (int)(*(rgba_ptr+2)) << "," <<(int)(*(rgba_ptr+3)) << ")";
  return (os);
}


struct PointXYZRGB
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float rgb;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointXYZRGB& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << ")";
  return (os);
}


struct PointXY
{
  float x;
  float y;
};
inline std::ostream& operator << (std::ostream& os, const PointXY& p)
{
  os << "(" << p.x << "," << p.y << ")";
  return (os);
}


struct InterestPoint
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float strength;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const InterestPoint& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.strength << ")";
  return (os);
}


struct Normal
{
  PCL_ADD_NORMAL4D;  // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float curvature;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const Normal& p)
{
  os << "(" << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
  return (os);
}


struct PointNormal
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float curvature;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointNormal& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
  return (os);
}


struct PointXYZRGBNormal
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float rgb;
      float curvature;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointXYZRGBNormal& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
  return (os);
}

struct PointXYZINormal
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float intensity;
      float curvature;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointXYZINormal& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
  return (os);
}

struct PointWithRange 
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float range;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointWithRange& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.range << ")";
  return (os);
}

struct _PointWithViewpoint 
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      float vp_x;
      float vp_y;
      float vp_z;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

struct PointWithViewpoint : public _PointWithViewpoint
{
  PointWithViewpoint(float _x=0.0f, float _y=0.0f, float _z=0.0f, float _vp_x=0.0f, float _vp_y=0.0f, float _vp_z=0.0f)
  {
    x=_x; y=_y; z=_z; data[3] = 1.0f;
    vp_x=_vp_x; vp_y=_vp_y; vp_z=_vp_z;
  }
};
inline std::ostream& operator << (std::ostream& os, const PointWithViewpoint& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.vp_x << "," << p.vp_y << "," << p.vp_z << ")";
  return (os);
}

struct MomentInvariants
{
  float j1, j2, j3;
};
inline std::ostream& operator << (std::ostream& os, const MomentInvariants& p)
{
  os << "(" << p.j1 << "," << p.j2 << "," << p.j3 << ")";
  return (os);
}

struct PrincipalRadiiRSD
{
  float r_min, r_max;
};
inline std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p)
{
  os << "(" << p.r_min << "," << p.r_max << ")";
  return (os);
}

struct Boundary
{
  uint8_t boundary_point;
};
inline std::ostream& operator << (std::ostream& os, const Boundary& p)
{
  os << p.boundary_point;
  return (os);
}

struct PrincipalCurvatures
{
  union
  {
    float principal_curvature[3];
    struct
    {
      float principal_curvature_x;
      float principal_curvature_y;
      float principal_curvature_z;
    };
  };
  float pc1;
  float pc2;
};
inline std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p)
{
  os << "(" << p.principal_curvature[0] << "," << p.principal_curvature[1] << "," << p.principal_curvature[2] << " - " << p.pc1 << "," << p.pc2 << ")";
  return (os);
}

struct PFHSignature125
{
  float histogram[125];
};
inline std::ostream& operator << (std::ostream& os, const PFHSignature125& p)
{
  for (int i = 0; i < 125; ++i) 
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 124 ? ", " : ")");
  return (os);
}

struct FPFHSignature33
{
  float histogram[33];
};
inline std::ostream& operator << (std::ostream& os, const FPFHSignature33& p)
{
  for (int i = 0; i < 33; ++i) 
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 32 ? ", " : ")");
  return (os);
}

struct VFHSignature308
{
  float histogram[308];
};
inline std::ostream& operator << (std::ostream& os, const VFHSignature308& p)
{
  for (int i = 0; i < 308; ++i) 
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 307 ? ", " : ")");
  return (os);
}

struct Narf36
{
  float x, y, z, roll, pitch, yaw;
  float descriptor[36];
};
inline std::ostream& operator << (std::ostream& os, const Narf36& p)
{
  os << p.x<<","<<p.y<<","<<p.z<<" - "<<p.roll*360.0/M_PI<<"deg,"<<p.pitch*360.0/M_PI<<"deg,"<<p.yaw*360.0/M_PI<<"deg - ";
  for (int i = 0; i < 36; ++i) 
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 35 ? ", " : ")");
  return (os);
}

struct BorderDescription 
{
  int x, y;
  BorderTraits traits;
  //std::vector<const BorderDescription*> neighbors;
};

inline std::ostream& operator << (std::ostream& os, const BorderDescription& p)
{
  os << "(" << p.x << "," << p.y << ")";
  return (os);
}

struct IntensityGradient
{
  union
  {
    float gradient[3];
    struct
    {
      float gradient_x;
      float gradient_y;
      float gradient_z;
    };
  };
};
inline std::ostream& operator << (std::ostream& os, const IntensityGradient& p)
{
  os << "(" << p.gradient[0] << "," << p.gradient[1] << "," << p.gradient[2] << ")";
  return (os);
}

template <int N>
struct Histogram
{
  float histogram[N];
};
template <int N>
inline std::ostream& operator << (std::ostream& os, const Histogram<N>& p)
{
  for (int i = 0; i < N; ++i) 
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < N-1 ? ", " : ")");
  return (os);
}

struct PointWithScale
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  float scale;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointWithScale& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.scale << ")";
  return (os);
}

struct PointSurfel
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      uint32_t rgba;
      float radius;
      float confidence;
      float curvature;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointSurfel& p)
{
  unsigned char* rgba_ptr = (unsigned char*)&p.rgba;
  os << 
    "(" << p.x << "," << p.y << "," << p.z << " - " <<
    p.normal_x << "," << p.normal_y << "," << p.normal_z << " - " <<
    (int)(*rgba_ptr) << "," << (int)(*(rgba_ptr+1)) << "," << (int)(*(rgba_ptr+2)) << "," <<(int)(*(rgba_ptr+3)) << " - " <<
    p.radius << " - " << p.confidence << " - " << p.curvature << ")";
  return (os);
}


template <typename PointType1, typename PointType2>
inline float squaredEuclideanDistance (const PointType1& p1, const PointType2& p2)
{
  float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
  return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

template <typename PointType1, typename PointType2>
inline float euclideanDistance (const PointType1& p1, const PointType2& p2)
{
  return (sqrtf (squaredEuclideanDistance (p1, p2)));
}

template <typename PointType>
inline bool hasValidXYZ (const PointType& p)
{
  return (pcl_isfinite (p.x) && pcl_isfinite (p.y) && pcl_isfinite (p.z));
}

}  // End namespace

#endif
