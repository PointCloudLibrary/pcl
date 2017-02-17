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

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <Eigen/Core>
#include <ostream>

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
  (pcl::GRSDSignature21)        \
  (pcl::ESFSignature640)        \
  (pcl::BRISKSignature512)      \
  (pcl::Narf36)

namespace pcl
{

  typedef Eigen::Map<Eigen::Array3f> Array3fMap;
  typedef const Eigen::Map<const Eigen::Array3f> Array3fMapConst;
  typedef Eigen::Map<Eigen::Array4f, Eigen::Aligned> Array4fMap;
  typedef const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> Array4fMapConst;
  typedef Eigen::Map<Eigen::Vector3f> Vector3fMap;
  typedef const Eigen::Map<const Eigen::Vector3f> Vector3fMapConst;
  typedef Eigen::Map<Eigen::Vector4f, Eigen::Aligned> Vector4fMap;
  typedef const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> Vector4fMapConst;

  typedef Eigen::Matrix<uint8_t, 3, 1> Vector3c;
  typedef Eigen::Map<Vector3c> Vector3cMap;
  typedef const Eigen::Map<const Vector3c> Vector3cMapConst;
  typedef Eigen::Matrix<uint8_t, 4, 1> Vector4c;
  typedef Eigen::Map<Vector4c, Eigen::Aligned> Vector4cMap;
  typedef const Eigen::Map<const Vector4c, Eigen::Aligned> Vector4cMapConst;

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
        uint8_t b; \
        uint8_t g; \
        uint8_t r; \
        uint8_t a; \
      }; \
      float rgb; \
    }; \
    uint32_t rgba; \
  };

#define PCL_ADD_EIGEN_MAPS_RGB \
  inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); } \
  inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); } \
  inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, a)); } \
  inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, a)); } \
  inline Eigen::Vector4i getRGBAVector4i () { return (Eigen::Vector4i (r, g, b, a)); } \
  inline const Eigen::Vector4i getRGBAVector4i () const { return (Eigen::Vector4i (r, g, b, a)); } \
  inline pcl::Vector3cMap getBGRVector3cMap () { return (pcl::Vector3cMap (reinterpret_cast<uint8_t*> (&rgba))); } \
  inline pcl::Vector3cMapConst getBGRVector3cMap () const { return (pcl::Vector3cMapConst (reinterpret_cast<const uint8_t*> (&rgba))); } \
  inline pcl::Vector4cMap getBGRAVector4cMap () { return (pcl::Vector4cMap (reinterpret_cast<uint8_t*> (&rgba))); } \
  inline pcl::Vector4cMapConst getBGRAVector4cMap () const { return (pcl::Vector4cMapConst (reinterpret_cast<const uint8_t*> (&rgba))); }

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
      uint8_t intensity; \
    }; \

#define PCL_ADD_INTENSITY_32U \
    struct \
    { \
        uint32_t intensity; \
    }; \


  struct _PointXYZ
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZ& p);
  /** \brief A point structure representing Euclidean xyz coordinates. (SSE friendly)
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointXYZ : public _PointXYZ
  {
    inline PointXYZ (const _PointXYZ &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    }

    inline PointXYZ ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
    }

    inline PointXYZ (float _x, float _y, float _z)
    {
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZ& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    * packed uint32_t rgba value. To pack it, use:
    *
    * \code
    * int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    * \endcode
    *
    * To unpack it use:
    *
    * \code
    * int rgb = ...;
    * uint8_t r = (rgb >> 16) & 0x0000ff;
    * uint8_t g = (rgb >> 8)  & 0x0000ff;
    * uint8_t b = (rgb)     & 0x0000ff;
    * \endcode
    *
    */
  struct RGB: public _RGB
  {
    inline RGB (const _RGB &p)
    {
      rgba = p.rgba;
    }

    inline RGB ()
    {
      r = g = b = 0;
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

    inline Intensity ()
    {
      intensity = 0.0f;
    }
  
    friend std::ostream& operator << (std::ostream& os, const Intensity& p);
  };
  

  struct _Intensity8u
  {
    PCL_ADD_INTENSITY_8U;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Intensity8u& p);
  /** \brief A point structure representing the grayscale intensity in single-channel images.
    * Intensity is represented as a uint8_t value.
    * \ingroup common
    */
  struct Intensity8u: public _Intensity8u
  {
    inline Intensity8u (const _Intensity8u &p)
    {
      intensity = p.intensity;
    }

    inline Intensity8u ()
    {
      intensity = 0;
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
    * Intensity is represented as a uint8_t value.
    * \ingroup common
    */
  struct Intensity32u: public _Intensity32u
  {
    inline Intensity32u (const _Intensity32u &p)
    {
      intensity = p.intensity;
    }

    inline Intensity32u ()
    {
      intensity = 0;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  struct PointXYZI : public _PointXYZI
  {
    inline PointXYZI (const _PointXYZI &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      intensity = p.intensity;
    }

    inline PointXYZI ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      intensity = 0.0f;
    }
    inline PointXYZI (float _intensity)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      intensity = _intensity;
    }
    friend std::ostream& operator << (std::ostream& os, const PointXYZI& p);
  };
  

  struct EIGEN_ALIGN16 _PointXYZL
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZL& p);
  struct PointXYZL : public _PointXYZL
  {
    inline PointXYZL (const _PointXYZL &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      label = p.label;
    }

    inline PointXYZL ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      label = 0;
    }
  
    friend std::ostream& operator << (std::ostream& os, const PointXYZL& p);
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Label& p);
  struct Label
  {
    uint32_t label;
  
    friend std::ostream& operator << (std::ostream& os, const Label& p);
  };


  struct EIGEN_ALIGN16 _PointXYZRGBA
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBA& p);
  /** \brief A point structure representing Euclidean xyz coordinates, and the RGBA color.
    *
    * The RGBA information is available either as separate r, g, b, or as a
    * packed uint32_t rgba value. To pack it, use:
    *
    * \code
    * int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    * \endcode
    *
    * To unpack it use:
    *
    * \code
    * int rgb = ...;
    * uint8_t r = (rgb >> 16) & 0x0000ff;
    * uint8_t g = (rgb >> 8)  & 0x0000ff;
    * uint8_t b = (rgb)     & 0x0000ff;
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

    inline PointXYZRGBA ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBA& p);
  };


  struct EIGEN_ALIGN16 _PointXYZRGB
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct EIGEN_ALIGN16 _PointXYZRGBL
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    * uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    * uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    * p.rgb = *reinterpret_cast<float*>(&rgb);
    * \endcode
    *
    * To unpack the data into separate values, use:
    *
    * \code
    * PointXYZRGB p;
    * // unpack rgb into r/g/b
    * uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
    * uint8_t r = (rgb >> 16) & 0x0000ff;
    * uint8_t g = (rgb >> 8)  & 0x0000ff;
    * uint8_t b = (rgb)       & 0x0000ff;
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

    inline PointXYZRGB ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
    }
    inline PointXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = _r;
      g = _g;
      b = _b;
      a = 255;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZRGB& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointXYZRGBL ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
      label = 255;
    }
    inline PointXYZRGBL (uint8_t _r, uint8_t _g, uint8_t _b, uint32_t _label)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = _r;
      g = _g;
      b = _b;
      a = 255;
      label = _label;
    }
  
    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBL& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  struct _PointXYZHSV
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZHSV& p);
  struct EIGEN_ALIGN16 PointXYZHSV : public _PointXYZHSV
  {
    inline PointXYZHSV (const _PointXYZHSV &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      h = p.h; s = p.s; v = p.v;
    }

    inline PointXYZHSV ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      h = s = v = data_c[3] = 0;
    }
    inline PointXYZHSV (float _h, float _v, float _s)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      h = _h; v = _v; s = _s;
      data_c[3] = 0;
    }
  
    friend std::ostream& operator << (std::ostream& os, const PointXYZHSV& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };



  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXY& p);
  /** \brief A 2D point structure representing Euclidean xy coordinates.
    * \ingroup common
    */
  struct PointXY
  {
    float x;
    float y;
  
    friend std::ostream& operator << (std::ostream& os, const PointXY& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointUV& p);
  /** \brief A 2D point structure representing pixel image coordinates.
    * \note We use float to be able to represent subpixels.
    * \ingroup common
    */
  struct PointUV
  {
    float u;
    float v;
  
    friend std::ostream& operator << (std::ostream& os, const PointUV& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const InterestPoint& p);
  /** \brief A point structure representing an interest point with Euclidean xyz coordinates, and an interest value.
    * \ingroup common
    */
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline Normal ()
    {
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      curvature = 0;
    }

    inline Normal (float n_x, float n_y, float n_z)
    {
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      curvature = 0;
      data_n[3] = 0.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const Normal& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  struct EIGEN_ALIGN16 _Axis
  {
    PCL_ADD_NORMAL4D;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
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

    inline Axis ()
    {
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    }

    inline Axis (float n_x, float n_y, float n_z)
    {
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const Axis& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointNormal ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    * uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    * uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    * p.rgb = *reinterpret_cast<float*>(&rgb);
    * \endcode
    *
    * To unpack the data into separate values, use:
    *
    * \code
    * PointXYZRGB p;
    * // unpack rgb into r/g/b
    * uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
    * uint8_t r = (rgb >> 16) & 0x0000ff;
    * uint8_t g = (rgb >> 8)  & 0x0000ff;
    * uint8_t b = (rgb)       & 0x0000ff;
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

    inline PointXYZRGBNormal ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      curvature = 0;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointXYZINormal ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      intensity = 0.0f;
      curvature = 0;
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
        uint32_t label;
        float curvature;
      };
      float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointXYZLNormal ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      label = 0;
      curvature = 0;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointWithRange ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      range = 0.0f;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointWithViewpoint (float _x = 0.0f, float _y = 0.0f, float _z = 0.0f,
                               float _vp_x = 0.0f, float _vp_y = 0.0f, float _vp_z = 0.0f)
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
    float j1, j2, j3;
  
    friend std::ostream& operator << (std::ostream& os, const MomentInvariants& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  /** \brief A point structure representing the minimum and maximum surface radii (in meters) computed using RSD.
    * \ingroup common
    */
  struct PrincipalRadiiRSD
  {
    float r_min, r_max;
  
    friend std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Boundary& p);
  /** \brief A point structure representing a description of whether a point is lying on a surface boundary or not.
    * \ingroup common
    */
  struct Boundary
  {
    uint8_t boundary_point;

#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION <= 1101
    operator unsigned char() const
    {
      return boundary_point;
    }
#endif
  
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
    float pc1;
    float pc2;
  
    friend std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  /** \brief A point structure representing the Point Feature Histogram (PFH).
    * \ingroup common
    */
  struct PFHSignature125
  {
    float histogram[125];
    static int descriptorSize () { return 125; }
  
    friend std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
  /** \brief A point structure representing the Point Feature Histogram with colors (PFHRGB).
    * \ingroup common
    */
  struct PFHRGBSignature250
  {
    float histogram[250];
    static int descriptorSize () { return 250; }

    friend std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PPFSignature& p);
  /** \brief A point structure for storing the Point Pair Feature (PPF) values
    * \ingroup common
    */
  struct PPFSignature
  {
    float f1, f2, f3, f4;
    float alpha_m;
  
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
  
    friend std::ostream& operator << (std::ostream& os, const CPPFSignature& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PPFRGBSignature& p);
  /** \brief A point structure for storing the Point Pair Color Feature (PPFRGB) values
    * \ingroup common
    */
  struct PPFRGBSignature
  {
    float f1, f2, f3, f4;
    float r_ratio, g_ratio, b_ratio;
    float alpha_m;
  
    friend std::ostream& operator << (std::ostream& os, const PPFRGBSignature& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const NormalBasedSignature12& p);
  /** \brief A point structure representing the Normal Based Signature for
    * a feature matrix of 4-by-3
    * \ingroup common
    */
  struct NormalBasedSignature12
  {
    float values[12];
  
    friend std::ostream& operator << (std::ostream& os, const NormalBasedSignature12& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  /** \brief A point structure representing a Shape Context.
    * \ingroup common
    */
  struct ShapeContext1980
  {
    float descriptor[1980];
    float rf[9];
    static int descriptorSize () { return 1980; }

    friend std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  /** \brief A point structure representing a Unique Shape Context.
    * \ingroup common
    */
  struct UniqueShapeContext1960
  {
    float descriptor[1960];
    float rf[9];
    static int descriptorSize () { return 1960; }

    friend std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT352& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape only.
    * \ingroup common
    */
  struct SHOT352
  {
    float descriptor[352];
    float rf[9];
    static int descriptorSize () { return 352; }

    friend std::ostream& operator << (std::ostream& os, const SHOT352& p);
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT1344& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape+color.
    * \ingroup common
    */
  struct SHOT1344
  {
    float descriptor[1344];
    float rf[9];
    static int descriptorSize () { return 1344; }

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
  struct EIGEN_ALIGN16 ReferenceFrame : public _ReferenceFrame
  {
    inline ReferenceFrame (const _ReferenceFrame &p)
    {
      for (int d = 0; d < 9; ++d)
        rf[d] = p.rf[d];
    }

    inline ReferenceFrame ()
    {
      for (int d = 0; d < 3; ++d)
        x_axis[d] = y_axis[d] = z_axis[d] = 0;
    }

    friend std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const FPFHSignature33& p);
  /** \brief A point structure representing the Fast Point Feature Histogram (FPFH).
    * \ingroup common
    */
  struct FPFHSignature33
  {
    float histogram[33];
    static int descriptorSize () { return 33; }

    friend std::ostream& operator << (std::ostream& os, const FPFHSignature33& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const VFHSignature308& p);
  /** \brief A point structure representing the Viewpoint Feature Histogram (VFH).
    * \ingroup common
    */
  struct VFHSignature308
  {
    float histogram[308];
    static int descriptorSize () { return 308; }

    friend std::ostream& operator << (std::ostream& os, const VFHSignature308& p);
  };
  
  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GRSDSignature21& p);
  /** \brief A point structure representing the Global Radius-based Surface Descriptor (GRSD).
    * \ingroup common
    */
  struct GRSDSignature21
  {
    float histogram[21];
    static int descriptorSize () { return 21; }

    friend std::ostream& operator << (std::ostream& os, const GRSDSignature21& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const BRISKSignature512& p);
  /** \brief A point structure representing the Binary Robust Invariant Scalable Keypoints (BRISK).
    * \ingroup common
    */
  struct BRISKSignature512
  {
    float scale;
    float orientation;
    unsigned char descriptor[64];
    static int descriptorSize () { return 64; }

    friend std::ostream& operator << (std::ostream& os, const BRISKSignature512& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ESFSignature640& p);
  /** \brief A point structure representing the Ensemble of Shape Functions (ESF).
    * \ingroup common
    */
  struct ESFSignature640
  {
    float histogram[640];
    static int descriptorSize () { return 640; }

    friend std::ostream& operator << (std::ostream& os, const ESFSignature640& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const GFPFHSignature16& p);
  /** \brief A point structure representing the GFPFH descriptor with 16 bins.
    * \ingroup common
    */
  struct GFPFHSignature16
  {
    float histogram[16];
    static int descriptorSize () { return 16; }
    
    friend std::ostream& operator << (std::ostream& os, const GFPFHSignature16& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Narf36& p);
  /** \brief A point structure representing the Narf descriptor.
    * \ingroup common
    */
  struct Narf36
  {
    float x, y, z, roll, pitch, yaw;
    float descriptor[36];
    static int descriptorSize () { return 36; }

    friend std::ostream& operator << (std::ostream& os, const Narf36& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const BorderDescription& p);
  /** \brief A structure to store if a point in a range image lies on a border between an obstacle and the background.
    * \ingroup common
    */
  struct BorderDescription
  {
    int x, y;
    BorderTraits traits;
    //std::vector<const BorderDescription*> neighbors;
  
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
  
    friend std::ostream& operator << (std::ostream& os, const IntensityGradient& p);
  };

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
      /** \brief Diameter of the meaningfull keypoint neighborhood. */
      float scale;
      float size;
    };

    /** \brief Computed orientation of the keypoint (-1 if not applicable). */
    float angle;
    /** \brief The response by which the most strong keypoints have been selected. */
    float response;
    /** \brief octave (pyramid layer) from which the keypoint has been extracted. */
    int   octave;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    inline PointWithScale ()
    {
      x = y = z = 0.0f;
      scale = 1.0f;
      angle = -1.0f;
      response = 0.0f;
      octave = 0;
      data[3] = 1.0f;
    }

    inline PointWithScale (float _x, float _y, float _z, float _scale)
    {
      x = _x;
      y = _y;
      z = _z;
      scale = _scale;
      angle = -1.0f;
      response = 0.0f;
      octave = 0;
      data[3] = 1.0f;
    }

    inline PointWithScale (float _x, float _y, float _z, float _scale, float _angle, float _response, int _octave)
    {
      x = _x;
      y = _y;
      z = _z;
      scale = _scale;
      angle = _angle;
      response = _response;
      octave = _octave;
      data[3] = 1.0f;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  
    friend std::ostream& operator << (std::ostream& os, const PointSurfel& p);
  };

  struct EIGEN_ALIGN16 _PointDEM
  {
    PCL_ADD_POINT4D;
    float intensity;
    float intensity_variance;
    float height_variance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointDEM& p);
  /** \brief A point structure representing Digital Elevation Map.
    * \ingroup common
    */
  struct PointDEM : public _PointDEM
  {
    inline PointDEM (const _PointDEM &p)
    {
      x = p.x; y = p.y; x = p.z; data[3] = 1.0f;
      intensity = p.intensity;
      intensity_variance = p.intensity_variance;
      height_variance = p.height_variance;
    }

    inline PointDEM ()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      intensity = 0.0f;
      intensity_variance = height_variance = 0.0f;
    }

    friend std::ostream& operator << (std::ostream& os, const PointDEM& p);
  };

  template <int N> std::ostream& 
  operator << (std::ostream& os, const Histogram<N>& p)
  {
    for (int i = 0; i < N; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < N-1 ? ", " : ")");
    return (os);
  }
} // End namespace

// Preserve API for PCL users < 1.4
#include <pcl/common/point_tests.h>

#endif
