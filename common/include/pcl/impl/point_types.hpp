/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_IMPL_POINT_TYPES_HPP_
#define PCL_IMPL_POINT_TYPES_HPP_

#include <cstdint>
#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <algorithm>
#include <ostream>

#include <Eigen/Core>

#include <pcl/pcl_macros.h>

// Define all PCL point types
#define PCL_POINT_TYPES         \
  (pcl::PointXYZ)               \
  (pcl::PointXYZI)              \
  (pcl::PointXYZL)              \
  (pcl::Label)                  \
  (pcl::PointXYZRGBA)           \
  (pcl::PointXYZRGB)            \
  (pcl::PointXYZRGBL)           \
  (pcl::PointXYZHSV)            \
  (pcl::PointXY)                \
  (pcl::InterestPoint)          \
  (pcl::Axis)                   \
  (pcl::Normal)                 \
  (pcl::PointNormal)            \
  (pcl::PointXYZRGBNormal)      \
  (pcl::PointXYZINormal)        \
  (pcl::PointXYZLNormal)        \
  (pcl::PointWithRange)         \
  (pcl::PointWithViewpoint)     \
  (pcl::MomentInvariants)       \
  (pcl::PrincipalRadiiRSD)      \
  (pcl::Boundary)               \
  (pcl::PrincipalCurvatures)    \
  (pcl::PFHSignature125)        \
  (pcl::PFHRGBSignature250)     \
  (pcl::PPFSignature)           \
  (pcl::CPPFSignature)          \
  (pcl::PPFRGBSignature)        \
  (pcl::NormalBasedSignature12) \
  (pcl::FPFHSignature33)        \
  (pcl::VFHSignature308)        \
  (pcl::GASDSignature512)       \
  (pcl::GASDSignature984)       \
  (pcl::GASDSignature7992)      \
  (pcl::GRSDSignature21)        \
  (pcl::ESFSignature640)        \
  (pcl::BRISKSignature512)      \
  (pcl::Narf36)                 \
  (pcl::IntensityGradient)      \
  (pcl::PointWithScale)         \
  (pcl::PointSurfel)            \
  (pcl::ShapeContext1980)       \
  (pcl::UniqueShapeContext1960) \
  (pcl::SHOT352)                \
  (pcl::SHOT1344)               \
  (pcl::PointUV)                \
  (pcl::ReferenceFrame)         \
  (pcl::PointDEM)

// Define all point types that include RGB data
#define PCL_RGB_POINT_TYPES     \
  (pcl::PointXYZRGBA)           \
  (pcl::PointXYZRGB)            \
  (pcl::PointXYZRGBL)           \
  (pcl::PointXYZRGBNormal)      \
  (pcl::PointSurfel)            \

// Define all point types that include XYZ data
#define PCL_XYZ_POINT_TYPES   \
  (pcl::PointXYZ)             \
  (pcl::PointXYZI)            \
  (pcl::PointXYZL)            \
  (pcl::PointXYZRGBA)         \
  (pcl::PointXYZRGB)          \
  (pcl::PointXYZRGBL)         \
  (pcl::PointXYZHSV)          \
  (pcl::InterestPoint)        \
  (pcl::PointNormal)          \
  (pcl::PointXYZRGBNormal)    \
  (pcl::PointXYZINormal)      \
  (pcl::PointXYZLNormal)      \
  (pcl::PointWithRange)       \
  (pcl::PointWithViewpoint)   \
  (pcl::PointWithScale)       \
  (pcl::PointSurfel)          \
  (pcl::PointDEM)

// Define all point types with XYZ and label
#define PCL_XYZL_POINT_TYPES  \
  (pcl::PointXYZL)            \
  (pcl::PointXYZRGBL)         \
  (pcl::PointXYZLNormal)

// Define all point types that include normal[3] data
#define PCL_NORMAL_POINT_TYPES  \
  (pcl::Normal)                 \
  (pcl::PointNormal)            \
  (pcl::PointXYZRGBNormal)      \
  (pcl::PointXYZINormal)        \
  (pcl::PointXYZLNormal)        \
  (pcl::PointSurfel)

// Define all point types that represent features
#define PCL_FEATURE_POINT_TYPES \
  (pcl::PFHSignature125)        \
  (pcl::PFHRGBSignature250)     \
  (pcl::PPFSignature)           \
  (pcl::CPPFSignature)          \
  (pcl::PPFRGBSignature)        \
  (pcl::NormalBasedSignature12) \
  (pcl::FPFHSignature33)        \
  (pcl::VFHSignature308)        \
  (pcl::GASDSignature512)       \
  (pcl::GASDSignature984)       \
  (pcl::GASDSignature7992)      \
  (pcl::GRSDSignature21)        \
  (pcl::ESFSignature640)        \
  (pcl::BRISKSignature512)      \
  (pcl::Narf36)

namespace pcl
{

  using Array3fMap = Eigen::Map<Eigen::Array3f>;
  using Array3fMapConst = const Eigen::Map<const Eigen::Array3f>;
  using Array4fMap = Eigen::Map<Eigen::Array4f, Eigen::Aligned>;
  using Array4fMapConst = const Eigen::Map<const Eigen::Array4f, Eigen::Aligned>;
  using Vector3fMap = Eigen::Map<Eigen::Vector3f>;
  using Vector3fMapConst = const Eigen::Map<const Eigen::Vector3f>;
  using Vector4fMap = Eigen::Map<Eigen::Vector4f, Eigen::Aligned>;
  using Vector4fMapConst = const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned>;

  using Vector3c = Eigen::Matrix<std::uint8_t, 3, 1>;
  using Vector3cMap = Eigen::Map<Vector3c>;
  using Vector3cMapConst = const Eigen::Map<const Vector3c>;
  using Vector4c = Eigen::Matrix<std::uint8_t, 4, 1>;
  using Vector4cMap = Eigen::Map<Vector4c, Eigen::Aligned>;
  using Vector4cMapConst = const Eigen::Map<const Vector4c, Eigen::Aligned>;

#define PCL_ADD_UNION_POINT4D \
  union EIGEN_ALIGN16 { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  };

#define PCL_ADD_EIGEN_MAPS_POINT4D \
  inline pcl::Vector3fMap getVector3fMap () { return (pcl::Vector3fMap (data)); } \
  inline pcl::Vector3fMapConst getVector3fMap () const { return (pcl::Vector3fMapConst (data)); } \
  inline pcl::Vector4fMap getVector4fMap () { return (pcl::Vector4fMap (data)); } \
  inline pcl::Vector4fMapConst getVector4fMap () const { return (pcl::Vector4fMapConst (data)); } \
  inline pcl::Array3fMap getArray3fMap () { return (pcl::Array3fMap (data)); } \
  inline pcl::Array3fMapConst getArray3fMap () const { return (pcl::Array3fMapConst (data)); } \
  inline pcl::Array4fMap getArray4fMap () { return (pcl::Array4fMap (data)); } \
  inline pcl::Array4fMapConst getArray4fMap () const { return (pcl::Array4fMapConst (data)); }

#define PCL_ADD_POINT4D \
  PCL_ADD_UNION_POINT4D \
  PCL_ADD_EIGEN_MAPS_POINT4D

#define PCL_ADD_UNION_NORMAL4D \
  union EIGEN_ALIGN16 { \
    float data_n[4]; \
    float normal[3]; \
    struct { \
      float normal_x; \
      float normal_y; \
      float normal_z; \
    }; \
  };

#define PCL_ADD_EIGEN_MAPS_NORMAL4D \
  inline pcl::Vector3fMap getNormalVector3fMap () { return (pcl::Vector3fMap (data_n)); } \
  inline pcl::Vector3fMapConst getNormalVector3fMap () const { return (pcl::Vector3fMapConst (data_n)); } \
  inline pcl::Vector4fMap getNormalVector4fMap () { return (pcl::Vector4fMap (data_n)); } \
  inline pcl::Vector4fMapConst getNormalVector4fMap () const { return (pcl::Vector4fMapConst (data_n)); }

#define PCL_ADD_NORMAL4D \
  PCL_ADD_UNION_NORMAL4D \
  PCL_ADD_EIGEN_MAPS_NORMAL4D

#define PCL_ADD_UNION_RGB \
  union \
  { \
    union \
    { \
      struct \
      { \
        std::uint8_t b; \
        std::uint8_t g; \
        std::uint8_t r; \
        std::uint8_t a; \
      }; \
      float rgb; \
    }; \
    std::uint32_t rgba; \
  };

#define PCL_ADD_EIGEN_MAPS_RGB \
  inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); } \
  inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); } \
  inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, a)); } \
  inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, a)); } \
  inline Eigen::Vector4i getRGBAVector4i () { return (Eigen::Vector4i (r, g, b, a)); } \
  inline const Eigen::Vector4i getRGBAVector4i () const { return (Eigen::Vector4i (r, g, b, a)); } \
  inline pcl::Vector3cMap getBGRVector3cMap () { return (pcl::Vector3cMap (reinterpret_cast<std::uint8_t*> (&rgba))); } \
  inline pcl::Vector3cMapConst getBGRVector3cMap () const { return (pcl::Vector3cMapConst (reinterpret_cast<const std::uint8_t*> (&rgba))); } \
  inline pcl::Vector4cMap getBGRAVector4cMap () { return (pcl::Vector4cMap (reinterpret_cast<std::uint8_t*> (&rgba))); } \
  inline pcl::Vector4cMapConst getBGRAVector4cMap () const { return (pcl::Vector4cMapConst (reinterpret_cast<const std::uint8_t*> (&rgba))); }

#define PCL_ADD_RGB \
  PCL_ADD_UNION_RGB \
  PCL_ADD_EIGEN_MAPS_RGB

#define PCL_ADD_INTENSITY \
    struct \
    { \
      float intensity; \
    }; \

#define PCL_ADD_INTENSITY_8U \
    struct \
    { \
      std::uint8_t intensity; \
    }; \

#define PCL_ADD_INTENSITY_32U \
    struct \
    { \
        std::uint32_t intensity; \
    }; \


  struct _PointXYZ
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZ& p);
  /** \brief A point structure representing Euclidean xyz coordinates. (SSE friendly)
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointXYZ : public _PointXYZ
  {
    inline PointXYZ (const _PointXYZ &p): PointXYZ(p.x, p.y, p.z) {}

    inline PointXYZ (): PointXYZ(0.f, 0.f, 0.f) {}

    inline PointXYZ (float _x, float _y, float _z)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZ& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


#ifdef RGB
#undef RGB
#endif
  struct _RGB
  {
    PCL_ADD_RGB;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const RGB& p);
  /** \brief A structure representing RGB color information.
    *
    * The RGBA information is available either as separate r, g, b, or as a
    * packed std::uint32_t rgba value. To pack it, use:
    *
    * \code
    * int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    * \endcode
    *
    * To unpack it use:
    *
    * \code
    * int rgb = ...;
    * std::uint8_t r = (rgb >> 16) & 0x0000ff;
    * std::uint8_t g = (rgb >> 8)  & 0x0000ff;
    * std::uint8_t b = (rgb)     & 0x0000ff;
    * \endcode
    *
    */
  struct RGB: public _RGB
  {
    inline RGB (const _RGB &p)
    {
        rgba = p.rgba;
    }

    inline RGB (): RGB(0, 0, 0) {}

    inline RGB (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b)
    {
      r = _r; g = _g; b = _b;
      a = 255;
    }

    friend std::ostream& operator << (std::ostream& os, const RGB& p);
  };

  struct _Intensity
  {
    PCL_ADD_INTENSITY;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Intensity& p);
  /** \brief A point structure representing the grayscale intensity in single-channel images.
    * Intensity is represented as a float value.
    * \ingroup common
    */
  struct Intensity: public _Intensity
  {
    inline Intensity (const _Intensity &p)
    {
      intensity = p.intensity;
    }

    inline Intensity (float _intensity = 0.f)
    {
        intensity = _intensity;
    }

    friend std::ostream& operator << (std::ostream& os, const Intensity& p);
  };


  struct _Intensity8u
  {
    PCL_ADD_INTENSITY_8U;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Intensity8u& p);
  /** \brief A point structure representing the grayscale intensity in single-channel images.
    * Intensity is represented as a std::uint8_t value.
    * \ingroup common
    */
  struct Intensity8u: public _Intensity8u
  {
    inline Intensity8u (const _Intensity8u &p)
    {
      intensity = p.intensity;
    }

    inline Intensity8u (std::uint8_t _intensity = 0)
    {
      intensity = _intensity;
    }

#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION <= 1101
    operator unsigned char() const
    {
      return intensity;
    }
#endif

    friend std::ostream& operator << (std::ostream& os, const Intensity8u& p);
  };

  struct _Intensity32u
  {
    PCL_ADD_INTENSITY_32U;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Intensity32u& p);
  /** \brief A point structure representing the grayscale intensity in single-channel images.
    * Intensity is represented as a std::uint32_t value.
    * \ingroup common
    */
  struct Intensity32u: public _Intensity32u
  {
    inline Intensity32u (const _Intensity32u &p)
    {
      intensity = p.intensity;
    }

    inline Intensity32u (std::uint32_t _intensity = 0)
    {
      intensity = _intensity;
    }

    friend std::ostream& operator << (std::ostream& os, const Intensity32u& p);
  };

  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 _PointXYZI
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float intensity;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  struct PointXYZI : public _PointXYZI
  {
    inline PointXYZI (const _PointXYZI &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      intensity = p.intensity;
    }

    inline PointXYZI (float _intensity = 0.f): PointXYZI(0.f, 0.f, 0.f, _intensity) {}

    inline PointXYZI (float _x, float _y, float _z, float _intensity = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      intensity = _intensity;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  };


  struct EIGEN_ALIGN16 _PointXYZL
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    std::uint32_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZL& p);
  struct PointXYZL : public _PointXYZL
  {
    inline PointXYZL (const _PointXYZL &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      label = p.label;
    }

    inline PointXYZL (std::uint32_t _label = 0): PointXYZL(0.f, 0.f, 0.f, _label) {}

    inline PointXYZL (float _x, float _y, float _z, std::uint32_t _label = 0)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      label = _label;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZL& p);
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Label& p);
  struct Label
  {
    std::uint32_t label = 0;

    Label (std::uint32_t _label = 0): label(_label) {}

    friend std::ostream& operator << (std::ostream& os, const Label& p);
  };


  struct EIGEN_ALIGN16 _PointXYZRGBA
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBA& p);
  /** \brief A point structure representing Euclidean xyz coordinates, and the RGBA color.
    *
    * The RGBA information is available either as separate r, g, b, or as a
    * packed std::uint32_t rgba value. To pack it, use:
    *
    * \code
    * int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    * \endcode
    *
    * To unpack it use:
    *
    * \code
    * int rgb = ...;
    * std::uint8_t r = (rgb >> 16) & 0x0000ff;
    * std::uint8_t g = (rgb >> 8)  & 0x0000ff;
    * std::uint8_t b = (rgb)     & 0x0000ff;
    * \endcode
    *
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointXYZRGBA : public _PointXYZRGBA
  {
    inline PointXYZRGBA (const _PointXYZRGBA &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgba = p.rgba;
    }

    inline PointXYZRGBA (): PointXYZRGBA (0, 0, 0, 0) {}

    inline PointXYZRGBA (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b, std::uint8_t _a):
      PointXYZRGBA (0.f, 0.f, 0.f, _r, _g, _b, _a) {}

    inline PointXYZRGBA (float _x, float _y, float _z):
      PointXYZRGBA (_x, _y, _z, 0, 0, 0, 0) {}

    inline PointXYZRGBA (float _x, float _y, float _z, std::uint8_t _r,
                         std::uint8_t _g, std::uint8_t _b, std::uint8_t _a)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      r = _r; g = _g; b = _b; a = _a;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBA& p);
  };


  struct EIGEN_ALIGN16 _PointXYZRGB
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct EIGEN_ALIGN16 _PointXYZRGBL
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    std::uint32_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGB& p);
  /** \brief A point structure representing Euclidean xyz coordinates, and the RGB color.
    *
    * Due to historical reasons (PCL was first developed as a ROS package), the
    * RGB information is packed into an integer and casted to a float. This is
    * something we wish to remove in the near future, but in the meantime, the
    * following code snippet should help you pack and unpack RGB colors in your
    * PointXYZRGB structure:
    *
    * \code
    * // pack r/g/b into rgb
    * std::uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    * std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    * p.rgb = *reinterpret_cast<float*>(&rgb);
    * \endcode
    *
    * To unpack the data into separate values, use:
    *
    * \code
    * PointXYZRGB p;
    * // unpack rgb into r/g/b
    * std::uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
    * std::uint8_t r = (rgb >> 16) & 0x0000ff;
    * std::uint8_t g = (rgb >> 8)  & 0x0000ff;
    * std::uint8_t b = (rgb)       & 0x0000ff;
    * \endcode
    *
    *
    * Alternatively, from 1.1.0 onwards, you can use p.r, p.g, and p.b directly.
    *
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointXYZRGB : public _PointXYZRGB
  {
    inline PointXYZRGB (const _PointXYZRGB &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgb = p.rgb;
    }

    inline PointXYZRGB (): PointXYZRGB (0.f, 0.f, 0.f) {}

    inline PointXYZRGB (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointXYZRGB (0.f, 0.f, 0.f, _r, _g, _b) {}

    inline PointXYZRGB (float _x, float _y, float _z):
      PointXYZRGB (_x, _y, _z, 0, 0, 0) {}

    inline PointXYZRGB (float _x, float _y, float _z,
                         std::uint8_t _r, std::uint8_t _g, std::uint8_t _b)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      r = _r; g = _g; b = _b;
      a = 255;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGB& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBL& p);
  struct EIGEN_ALIGN16 PointXYZRGBL : public _PointXYZRGBL
  {
    inline PointXYZRGBL (const _PointXYZRGBL &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgba = p.rgba;
      label = p.label;
    }

    inline PointXYZRGBL (std::uint32_t _label = 0):
      PointXYZRGBL (0.f, 0.f, 0.f, 0, 0, 0, _label) {}

    inline PointXYZRGBL (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointXYZRGBL (0.f, 0.f, 0.f, _r, _g, _b) {}

    inline PointXYZRGBL (float _x, float _y, float _z):
      PointXYZRGBL (_x, _y, _z, 0, 0, 0) {}

    inline PointXYZRGBL (float _x, float _y, float _z,
                         std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                         std::uint32_t _label = 0)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      r = _r; g = _g; b = _b;
      a = 255;
      label = _label;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBL& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


  struct EIGEN_ALIGN16 _PointXYZHSV
  {
    PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float h;
        float s;
        float v;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZHSV& p);
  struct EIGEN_ALIGN16 PointXYZHSV : public _PointXYZHSV
  {
    inline PointXYZHSV (const _PointXYZHSV &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      h = p.h; s = p.s; v = p.v;
    }

    inline PointXYZHSV (): PointXYZHSV (0.f, 0.f, 0.f) {}

    // @TODO: Use strong types??
    // This is a dangerous type, doesn't behave like others
    inline PointXYZHSV (float _h, float _s, float _v):
      PointXYZHSV (0.f, 0.f, 0.f, _h, _s, _v) {}

    inline PointXYZHSV (float _x, float _y, float _z,
                        float _h, float _s, float _v)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      h = _h; s = _s; v = _v;
      data_c[3] = 0;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZHSV& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };



  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXY& p);
  /** \brief A 2D point structure representing Euclidean xy coordinates.
    * \ingroup common
    */
  struct PointXY
  {
    float x = 0.f;
    float y = 0.f;

    inline PointXY() = default;

    inline PointXY(float _x, float _y): x(_x), y(_y) {}

    friend std::ostream& operator << (std::ostream& os, const PointXY& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointUV& p);
  /** \brief A 2D point structure representing pixel image coordinates.
    * \note We use float to be able to represent subpixels.
    * \ingroup common
    */
  struct PointUV
  {
    float u = 0.f;
    float v = 0.f;

    inline PointUV() = default;

    inline PointUV(float _u, float _v): u(_u), v(_v) {}

    friend std::ostream& operator << (std::ostream& os, const PointUV& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const InterestPoint& p);
  /** \brief A point structure representing an interest point with Euclidean xyz coordinates, and an interest value.
    * \ingroup common
    */
  // @TODO: inheritance trick like on other PointTypes
  struct EIGEN_ALIGN16 InterestPoint
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float strength;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW

    friend std::ostream& operator << (std::ostream& os, const InterestPoint& p);
  };

  struct EIGEN_ALIGN16 _Normal
  {
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float curvature;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Normal& p);
  /** \brief A point structure representing normal coordinates and the surface curvature estimate. (SSE friendly)
    * \ingroup common
    */
  struct Normal : public _Normal
  {
    inline Normal (const _Normal &p)
    {
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z;
      data_n[3] = 0.0f;
      curvature = p.curvature;
    }

    inline Normal (float _curvature = 0.f): Normal (0.f, 0.f, 0.f, _curvature) {}

    inline Normal (float n_x, float n_y, float n_z, float _curvature = 0.f)
    {
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.0f;
      curvature = _curvature;
    }

    friend std::ostream& operator << (std::ostream& os, const Normal& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


  struct EIGEN_ALIGN16 _Axis
  {
    PCL_ADD_NORMAL4D;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Axis& p);
  /** \brief A point structure representing an Axis using its normal coordinates. (SSE friendly)
    *  \ingroup common
    */
  struct EIGEN_ALIGN16 Axis : public _Axis
  {
    inline Axis (const _Axis &p)
    {
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z;
      data_n[3] = 0.0f;
    }

    inline Axis (): Axis (0.f, 0.f, 0.f) {}

    inline Axis (float n_x, float n_y, float n_z)
    {
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const Axis& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


  struct EIGEN_ALIGN16 _PointNormal
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float curvature;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointNormal& p);
  /** \brief A point structure representing Euclidean xyz coordinates, together with normal coordinates and the surface curvature estimate. (SSE friendly)
    * \ingroup common
    */
  struct PointNormal : public _PointNormal
  {
    inline PointNormal (const _PointNormal &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
      curvature = p.curvature;
    }

    inline PointNormal (float _curvature = 0.f): PointNormal (0.f, 0.f, 0.f, 0.f, 0.f, 0.f, _curvature) {}

    inline PointNormal (float _x, float _y, float _z):
      PointNormal (_x, _y, _z, 0.f, 0.f, 0.f, 0.f) {}

    inline PointNormal (float _x, float _y, float _z, float n_x, float n_y, float n_z, float _curvature = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.0f;
      curvature = _curvature;
    }

    friend std::ostream& operator << (std::ostream& os, const PointNormal& p);
  };


  struct EIGEN_ALIGN16 _PointXYZRGBNormal
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        PCL_ADD_UNION_RGB;
        float curvature;
      };
      float data_c[4];
    };
    PCL_ADD_EIGEN_MAPS_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBNormal& p);
  /** \brief A point structure representing Euclidean xyz coordinates, and the RGB color, together with normal coordinates and the surface curvature estimate.
    * Due to historical reasons (PCL was first developed as a ROS package), the
    * RGB information is packed into an integer and casted to a float. This is
    * something we wish to remove in the near future, but in the meantime, the
    * following code snippet should help you pack and unpack RGB colors in your
    * PointXYZRGB structure:
    *
    * \code
    * // pack r/g/b into rgb
    * std::uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    * std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    * p.rgb = *reinterpret_cast<float*>(&rgb);
    * \endcode
    *
    * To unpack the data into separate values, use:
    *
    * \code
    * PointXYZRGB p;
    * // unpack rgb into r/g/b
    * std::uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
    * std::uint8_t r = (rgb >> 16) & 0x0000ff;
    * std::uint8_t g = (rgb >> 8)  & 0x0000ff;
    * std::uint8_t b = (rgb)       & 0x0000ff;
    * \endcode
    *
    *
    * Alternatively, from 1.1.0 onwards, you can use p.r, p.g, and p.b directly.
    * \ingroup common
    */
  struct PointXYZRGBNormal : public _PointXYZRGBNormal
  {
    inline PointXYZRGBNormal (const _PointXYZRGBNormal &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
      curvature = p.curvature;
      rgba = p.rgba;
    }

    inline PointXYZRGBNormal (float _curvature = 0.f): PointXYZRGBNormal (0.f, 0.f, 0.f) {}

    inline PointXYZRGBNormal (float _x, float _y, float _z):
      PointXYZRGBNormal (_x, _y, _z, 0, 0, 0) {}

    inline PointXYZRGBNormal (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointXYZRGBNormal (0.f, 0.f, 0.f, _r, _g, _b) {}

    inline PointXYZRGBNormal (float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointXYZRGBNormal (_x, _y, _z, _r, _g, _b, 0.f, 0.f, 0.f) {}

    inline PointXYZRGBNormal (float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                              float n_x, float n_y, float n_z, float _curvature = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      r = _r; g = _g; b = _b;
      a = 255;
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.f;
      curvature = _curvature;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBNormal& p);
  };

  struct EIGEN_ALIGN16 _PointXYZINormal
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float intensity;
        float curvature;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZINormal& p);
  /** \brief A point structure representing Euclidean xyz coordinates, intensity, together with normal coordinates and the surface curvature estimate.
    * \ingroup common
    */
  struct PointXYZINormal : public _PointXYZINormal
  {
    inline PointXYZINormal (const _PointXYZINormal &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
      curvature = p.curvature;
      intensity = p.intensity;
    }
    
    inline PointXYZINormal (float _intensity = 0.f): PointXYZINormal (0.f, 0.f, 0.f, 0.f) {}
    
    inline PointXYZINormal (float _x, float _y, float _z, float _intensity = 0.f):
      PointXYZINormal (_x, _y, _z, 0.f, 0.f, 0.f, 0.f) {}

    inline PointXYZINormal (float _x, float _y, float _z, float _intensity,
                            float n_x, float n_y, float n_z, float _curvature = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      intensity = _intensity;
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.f;
      curvature = _curvature;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZINormal& p);
  };

//----
  struct EIGEN_ALIGN16 _PointXYZLNormal
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        std::uint32_t label;
        float curvature;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZLNormal& p);
  /** \brief A point structure representing Euclidean xyz coordinates, a label, together with normal coordinates and the surface curvature estimate.
    * \ingroup common
    */
  struct PointXYZLNormal : public _PointXYZLNormal
  {
    inline PointXYZLNormal (const _PointXYZLNormal &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
      curvature = p.curvature;
      label = p.label;
    }

    inline PointXYZLNormal (std::uint32_t _label = 0): PointXYZLNormal (0.f, 0.f, 0.f, 0) {}
    
    inline PointXYZLNormal (float _x, float _y, float _z, std::uint32_t _label = 0.f):
      PointXYZLNormal (_x, _y, _z, 0, 0.f, 0.f, 0.f) {}

    inline PointXYZLNormal (float _x, float _y, float _z, std::uint32_t _label,
                            float n_x, float n_y, float n_z, float _curvature = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      label = _label;
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.f;
      curvature = _curvature;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZLNormal& p);
  };

//  ---


  struct EIGEN_ALIGN16 _PointWithRange
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        float range;
      };
      float data_c[4];
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointWithRange& p);
  /** \brief A point structure representing Euclidean xyz coordinates, padded with an extra range float.
    * \ingroup common
    */
  struct PointWithRange : public _PointWithRange
  {
    inline PointWithRange (const _PointWithRange &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      range = p.range;
    }

    inline PointWithRange (float _range = 0.f): PointWithRange (0.f, 0.f, 0.f, _range) {}

    inline PointWithRange (float _x, float _y, float _z, float _range = 0.f)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      range = _range;
    }

    friend std::ostream& operator << (std::ostream& os, const PointWithRange& p);
  };


  struct EIGEN_ALIGN16 _PointWithViewpoint
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
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
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointWithViewpoint& p);
  /** \brief A point structure representing Euclidean xyz coordinates together with the viewpoint from which it was seen.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointWithViewpoint : public _PointWithViewpoint
  {
    inline PointWithViewpoint (const _PointWithViewpoint &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      vp_x = p.vp_x; vp_y = p.vp_y; vp_z = p.vp_z;
    }

    inline PointWithViewpoint (): PointWithViewpoint (0.f, 0.f, 0.f) {}

    PCL_DEPRECATED("Use ctor accepting all position (x, y, z) data")
    inline PointWithViewpoint (float _x, float _y = 0.f):
      PointWithViewpoint (_x, _y, 0.f) {}

    PCL_DEPRECATED("Use ctor accepting all viewpoint (vp_x, vp_y, vp_z) data")
    inline PointWithViewpoint (float _x, float _y, float _z, float _vp_x, float _vp_y = 0.f):
      PointWithViewpoint (_x, _y, _z, _vp_x, _vp_y, 0.f) {}
    
    inline PointWithViewpoint (float _x, float _y, float _z): PointWithViewpoint (_x, _y, _z, 0.f, 0.f, 0.f) {}

    inline PointWithViewpoint (float _x, float _y, float _z, float _vp_x, float _vp_y, float _vp_z)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      vp_x = _vp_x; vp_y = _vp_y; vp_z = _vp_z;
    }

    friend std::ostream& operator << (std::ostream& os, const PointWithViewpoint& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const MomentInvariants& p);
  /** \brief A point structure representing the three moment invariants.
    * \ingroup common
    */
  struct MomentInvariants
  {
    float j1 = 0.f, j2 = 0.f, j3 = 0.f;

    inline MomentInvariants () = default;

    inline MomentInvariants (float _j1, float _j2, float _j3): j1 (_j1), j2 (_j2), j3 (_j3) {}

    friend std::ostream& operator << (std::ostream& os, const MomentInvariants& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  /** \brief A point structure representing the minimum and maximum surface radii (in meters) computed using RSD.
    * \ingroup common
    */
  struct PrincipalRadiiRSD
  {
    float r_min = 0.f, r_max = 0.f;

    inline PrincipalRadiiRSD () = default;

    inline PrincipalRadiiRSD (float _r_min, float _r_max): r_min (_r_min), r_max (_r_max) {}
    
    friend std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Boundary& p);
  /** \brief A point structure representing a description of whether a point is lying on a surface boundary or not.
    * \ingroup common
    */
  struct Boundary
  {
    std::uint8_t boundary_point = 0;

#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION <= 1101
    operator unsigned char() const
    {
      return boundary_point;
    }
#endif

    inline Boundary (std::uint8_t _boundary = 0): boundary_point (_boundary) {}

    friend std::ostream& operator << (std::ostream& os, const Boundary& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p);
  /** \brief A point structure representing the principal curvatures and their magnitudes.
    * \ingroup common
    */
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
    float pc1 = 0.f;
    float pc2 = 0.f;

    inline PrincipalCurvatures (): PrincipalCurvatures (0.f, 0.f) {}

    inline PrincipalCurvatures (float _pc1, float _pc2): PrincipalCurvatures (0.f, 0.f, 0.f, _pc1, _pc2) {}

    inline PrincipalCurvatures (float _x, float _y, float _z): PrincipalCurvatures (_x, _y, _z, 0.f, 0.f) {}

    inline PrincipalCurvatures (float _x, float _y, float _z, float _pc1, float _pc2):
      principal_curvature_x (_x), principal_curvature_y (_y), principal_curvature_z (_z), pc1 (_pc1), pc2 (_pc2) {}

    friend std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  /** \brief A point structure representing the Point Feature Histogram (PFH).
    * \ingroup common
    */
  struct PFHSignature125
  {
    float histogram[125] = {0.f};
    static int descriptorSize () { return 125; }

    inline PFHSignature125 () = default;

    friend std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
  /** \brief A point structure representing the Point Feature Histogram with colors (PFHRGB).
    * \ingroup common
    */
  struct PFHRGBSignature250
  {
    float histogram[250] = {0.f};
    static int descriptorSize () { return 250; }

    inline PFHRGBSignature250 () = default;

    friend std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PPFSignature& p);
  /** \brief A point structure for storing the Point Pair Feature (PPF) values
    * \ingroup common
    */
  struct PPFSignature
  {
    float f1 = 0.f, f2 = 0.f, f3 = 0.f, f4 = 0.f;
    float alpha_m = 0.f;

    inline PPFSignature (float _alpha = 0.f): PPFSignature (0.f, 0.f, 0.f, 0.f, _alpha) {}

    inline PPFSignature (float _f1, float _f2, float _f3, float _f4, float _alpha = 0.f):
      f1 (_f1), f2 (_f2), f3 (_f3), f4 (_f4), alpha_m (_alpha) {}

    friend std::ostream& operator << (std::ostream& os, const PPFSignature& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const CPPFSignature& p);
  /** \brief A point structure for storing the Point Pair Feature (CPPF) values
    * \ingroup common
    */
  struct CPPFSignature
  {
    float f1, f2, f3, f4, f5, f6, f7, f8, f9, f10;
    float alpha_m;

    inline CPPFSignature (float _alpha = 0.f):
      CPPFSignature (0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, _alpha) {}

    inline CPPFSignature (float _f1, float _f2, float _f3, float _f4, float _f5, float _f6,
                          float _f7, float _f8, float _f9, float _f10, float _alpha = 0.f):
      f1 (_f1), f2 (_f2), f3 (_f3), f4 (_f4), f5 (_f5), f6 (_f6),
      f7 (_f7), f8 (_f8), f9 (_f9), f10 (_f10), alpha_m (_alpha) {}

    friend std::ostream& operator << (std::ostream& os, const CPPFSignature& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PPFRGBSignature& p);
  /** \brief A point structure for storing the Point Pair Color Feature (PPFRGB) values
    * \ingroup common
    */
  struct PPFRGBSignature
  {
    float f1 = 0.f, f2 = 0.f, f3 = 0.f, f4 = 0.f;
    float r_ratio = 0.f, g_ratio = 0.f, b_ratio = 0.f;
    float alpha_m = 0.f;

    inline PPFRGBSignature (float _alpha = 0.f): PPFRGBSignature (0.f, 0.f, 0.f, 0.f, _alpha) {}

    inline PPFRGBSignature (float _f1, float _f2, float _f3, float _f4, float _alpha = 0.f):
      PPFRGBSignature (0.f, 0.f, 0.f, 0.f, _alpha, 0.f, 0.f, 0.f) {}

    inline PPFRGBSignature (float _f1, float _f2, float _f3, float _f4, float _alpha, float _r, float _g, float _b):
      f1 (_f1), f2 (_f2), f3 (_f3), f4 (_f4), r_ratio (_r), g_ratio (_g), b_ratio (_b), alpha_m (_alpha) {}

    friend std::ostream& operator << (std::ostream& os, const PPFRGBSignature& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const NormalBasedSignature12& p);
  /** \brief A point structure representing the Normal Based Signature for
    * a feature matrix of 4-by-3
    * \ingroup common
    */
  struct NormalBasedSignature12
  {
    float values[12] = {0.f};

    inline NormalBasedSignature12 () = default;

    friend std::ostream& operator << (std::ostream& os, const NormalBasedSignature12& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  /** \brief A point structure representing a Shape Context.
    * \ingroup common
    */
  struct ShapeContext1980
  {
    float descriptor[1980] = {0.f};
    float rf[9] = {0.f};
    static int descriptorSize () { return 1980; }

    inline ShapeContext1980 () = default;

    friend std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  /** \brief A point structure representing a Unique Shape Context.
    * \ingroup common
    */
  struct UniqueShapeContext1960
  {
    float descriptor[1960] = {0.f};
    float rf[9] = {0.f};
    static int descriptorSize () { return 1960; }

    inline UniqueShapeContext1960 () = default;

    friend std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT352& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape only.
    * \ingroup common
    */
  struct SHOT352
  {
    float descriptor[352] = {0.f};
    float rf[9] = {0.f};
    static int descriptorSize () { return 352; }

    inline SHOT352 () = default;

    friend std::ostream& operator << (std::ostream& os, const SHOT352& p);
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT1344& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape+color.
    * \ingroup common
    */
  struct SHOT1344
  {
    float descriptor[1344] = {0.f};
    float rf[9] = {0.f};
    static int descriptorSize () { return 1344; }

    inline SHOT1344 () = default;

    friend std::ostream& operator << (std::ostream& os, const SHOT1344& p);
  };


  /** \brief A structure representing the Local Reference Frame of a point.
    *  \ingroup common
    */
  struct EIGEN_ALIGN16 _ReferenceFrame
  {
    union
    {
      float rf[9];
      struct
      {
        float x_axis[3];
        float y_axis[3];
        float z_axis[3];
      };
    };

    inline Eigen::Map<Eigen::Vector3f> getXAxisVector3fMap () { return (Eigen::Vector3f::Map (x_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getXAxisVector3fMap () const { return (Eigen::Vector3f::Map (x_axis)); }
    inline Eigen::Map<Eigen::Vector3f> getYAxisVector3fMap () { return (Eigen::Vector3f::Map (y_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getYAxisVector3fMap () const { return (Eigen::Vector3f::Map (y_axis)); }
    inline Eigen::Map<Eigen::Vector3f> getZAxisVector3fMap () { return (Eigen::Vector3f::Map (z_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getZAxisVector3fMap () const { return (Eigen::Vector3f::Map (z_axis)); }
    inline Eigen::Map<Eigen::Matrix3f> getMatrix3fMap () { return (Eigen::Matrix3f::Map (rf)); }
    inline const Eigen::Map<const Eigen::Matrix3f> getMatrix3fMap () const { return (Eigen::Matrix3f::Map (rf)); }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
  struct EIGEN_ALIGN16 ReferenceFrame : public _ReferenceFrame
  {
    inline ReferenceFrame (const _ReferenceFrame &p)
    {
      std::copy_n(p.rf, 9, rf);
    }

    inline ReferenceFrame ()
    {
      std::fill_n(x_axis, 3, 0);
      std::fill_n(y_axis, 3, 0);
      std::fill_n(z_axis, 3, 0);
    }

    // @TODO: add other ctors

    friend std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const FPFHSignature33& p);
  /** \brief A point structure representing the Fast Point Feature Histogram (FPFH).
    * \ingroup common
    */
  struct FPFHSignature33
  {
    float histogram[33] = {0.f};
    static int descriptorSize () { return 33; }

    inline FPFHSignature33 () = default;

    friend std::ostream& operator << (std::ostream& os, const FPFHSignature33& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const VFHSignature308& p);
  /** \brief A point structure representing the Viewpoint Feature Histogram (VFH).
    * \ingroup common
    */
  struct VFHSignature308
  {
    float histogram[308] = {0.f};
    static int descriptorSize () { return 308; }

    inline VFHSignature308 () = default;

    friend std::ostream& operator << (std::ostream& os, const VFHSignature308& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GRSDSignature21& p);
  /** \brief A point structure representing the Global Radius-based Surface Descriptor (GRSD).
    * \ingroup common
    */
  struct GRSDSignature21
  {
    float histogram[21] = {0.f};
    static int descriptorSize () { return 21; }

    inline GRSDSignature21 () = default;

    friend std::ostream& operator << (std::ostream& os, const GRSDSignature21& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const BRISKSignature512& p);
  /** \brief A point structure representing the Binary Robust Invariant Scalable Keypoints (BRISK).
    * \ingroup common
    */
  struct BRISKSignature512
  {
    float scale = 0.f;
    float orientation = 0.f;
    unsigned char descriptor[64] = {0};
    static int descriptorSize () { return 64; }

    inline BRISKSignature512 () = default;

    inline BRISKSignature512 (float _scale, float _orientation): scale (_scale), orientation (_orientation) {}

    friend std::ostream& operator << (std::ostream& os, const BRISKSignature512& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ESFSignature640& p);
  /** \brief A point structure representing the Ensemble of Shape Functions (ESF).
    * \ingroup common
    */
  struct ESFSignature640
  {
    float histogram[640] = {0.f};
    static int descriptorSize () { return 640; }

    inline ESFSignature640 () = default;

    friend std::ostream& operator << (std::ostream& os, const ESFSignature640& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GASDSignature512& p);
  /** \brief A point structure representing the Globally Aligned Spatial Distribution (GASD) shape descriptor.
  * \ingroup common
  */
  struct GASDSignature512
  {
    float histogram[512] = {0.f};
    static int descriptorSize() { return 512; }

    inline GASDSignature512 () = default;

    friend std::ostream& operator << (std::ostream& os, const GASDSignature512& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GASDSignature984& p);
  /** \brief A point structure representing the Globally Aligned Spatial Distribution (GASD) shape and color descriptor.
  * \ingroup common
  */
  struct GASDSignature984
  {
    float histogram[984] = {0.f};
    static int descriptorSize() { return 984; }

    inline GASDSignature984 () = default;

    friend std::ostream& operator << (std::ostream& os, const GASDSignature984& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GASDSignature7992& p);
  /** \brief A point structure representing the Globally Aligned Spatial Distribution (GASD) shape and color descriptor.
  * \ingroup common
  */
  struct GASDSignature7992
  {
    float histogram[7992] = {0.f};
    static int descriptorSize() { return 7992; }

    inline GASDSignature7992 () = default;

    friend std::ostream& operator << (std::ostream& os, const GASDSignature7992& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GFPFHSignature16& p);
  /** \brief A point structure representing the GFPFH descriptor with 16 bins.
    * \ingroup common
    */
  struct GFPFHSignature16
  {
    float histogram[16] = {0.f};
    static int descriptorSize () { return 16; }

    inline GFPFHSignature16 () = default;

    friend std::ostream& operator << (std::ostream& os, const GFPFHSignature16& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Narf36& p);
  /** \brief A point structure representing the Narf descriptor.
    * \ingroup common
    */
  struct Narf36
  {
    float x = 0.f, y = 0.f, z = 0.f, roll = 0.f, pitch = 0.f, yaw = 0.f;
    float descriptor[36] = {0.f};
    static int descriptorSize () { return 36; }

    inline Narf36 () = default;

    inline Narf36 (float _x, float _y, float _z): Narf36 (_x, _y, _z, 0.f, 0.f, 0.f) {}

    inline Narf36 (float _x, float _y, float _z, float _roll, float _pitch, float _yaw):
      x (_x), y (_y), z (_z), roll (_roll), pitch (_pitch), yaw (_yaw) {}

    friend std::ostream& operator << (std::ostream& os, const Narf36& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const BorderDescription& p);
  /** \brief A structure to store if a point in a range image lies on a border between an obstacle and the background.
    * \ingroup common
    */
  struct BorderDescription
  {
    int x = 0.f, y = 0.f;
    BorderTraits traits;
    //std::vector<const BorderDescription*> neighbors;

    inline BorderDescription () = default;

    // TODO: provide other ctors

    friend std::ostream& operator << (std::ostream& os, const BorderDescription& p);
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const IntensityGradient& p);
  /** \brief A point structure representing the intensity gradient of an XYZI point cloud.
    * \ingroup common
    */
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

    inline IntensityGradient (): IntensityGradient (0.f, 0.f, 0.f) {}

    inline IntensityGradient (float _x, float _y, float _z): gradient_x (_x), gradient_y (_y), gradient_z (_z) {}

    friend std::ostream& operator << (std::ostream& os, const IntensityGradient& p);
  };

  // TODO: Maybe make other histogram based structs an alias for this
  /** \brief A point structure representing an N-D histogram.
    * \ingroup common
    */
  template <int N>
  struct Histogram
  {
    float histogram[N];
    static int descriptorSize () { return N; }
  };

  struct EIGEN_ALIGN16 _PointWithScale
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

    union
    {
      /** \brief Diameter of the meaningful keypoint neighborhood. */
      float scale;
      float size;
    };

    /** \brief Computed orientation of the keypoint (-1 if not applicable). */
    float angle;
    /** \brief The response by which the most strong keypoints have been selected. */
    float response;
    /** \brief octave (pyramid layer) from which the keypoint has been extracted. */
    int   octave;

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointWithScale& p);
  /** \brief A point structure representing a 3-D position and scale.
    * \ingroup common
    */
  struct PointWithScale : public _PointWithScale
  {
    inline PointWithScale (const _PointWithScale &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      scale = p.scale;
      angle = p.angle;
      response = p.response;
      octave = p.octave;
    }

    inline PointWithScale (): PointWithScale (0.f, 0.f, 0.f) {}

    inline PointWithScale (float _x, float _y, float _z, float _scale = 1.f, 
                           float _angle = -1.f, float _response = 0.f, int _octave = 0)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      scale = _scale;
      angle = _angle;
      response = _response;
      octave = _octave;
    }

    friend std::ostream& operator << (std::ostream& os, const PointWithScale& p);
  };


  struct EIGEN_ALIGN16 _PointSurfel
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        PCL_ADD_UNION_RGB;
        float radius;
        float confidence;
        float curvature;
      };
      float data_c[4];
    };
    PCL_ADD_EIGEN_MAPS_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointSurfel& p);
  /** \brief A surfel, that is, a point structure representing Euclidean xyz coordinates, together with normal coordinates, a RGBA color, a radius, a confidence value and the surface curvature estimate.
    * \ingroup common
    */
  struct PointSurfel : public _PointSurfel
  {
    inline PointSurfel (const _PointSurfel &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      rgba = p.rgba;
      radius = p.radius;
      confidence = p.confidence;
      curvature = p.curvature;
    }

    inline PointSurfel ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      r = g = b = 0;
      a = 255;
      radius = confidence = curvature = 0.0f;
    }

    // TODO: add other ctor to PointSurfel

    friend std::ostream& operator << (std::ostream& os, const PointSurfel& p);
  };

  struct EIGEN_ALIGN16 _PointDEM
  {
    PCL_ADD_POINT4D;
    float intensity;
    float intensity_variance;
    float height_variance;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointDEM& p);
  /** \brief A point structure representing Digital Elevation Map.
    * \ingroup common
    */
  struct PointDEM : public _PointDEM
  {
    inline PointDEM (const _PointDEM &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      intensity = p.intensity;
      intensity_variance = p.intensity_variance;
      height_variance = p.height_variance;
    }

    inline PointDEM (): PointDEM (0.f, 0.f, 0.f) {}

    inline PointDEM (float _x, float _y, float _z): PointDEM (_x, _y, _z, 0.f, 0.f, 0.f) {}

    inline PointDEM (float _x, float _y, float _z, float _intensity,
                     float _intensity_variance, float _height_variance)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      intensity = _intensity;
      intensity_variance = _intensity_variance;
      height_variance = _height_variance;
    }

    friend std::ostream& operator << (std::ostream& os, const PointDEM& p);
  };

  template <int N> std::ostream& 
  operator << (std::ostream& os, const Histogram<N>& p)
  {
    // make constexpr
    if (N > 0)
    {
        os << "(" << p.histogram[0];
        std::for_each(p.histogram + 1, std::end(p.histogram),
            [&os](const auto& hist) { os << ", " << hist; });
        os << ")";
    }
    return (os);
  }
} // End namespace

#endif
