/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#pragma once

/**
 * \file pcl/pcl_macros.h
 *
 * \brief Defines all the PCL and non-PCL macros used
 * \ingroup common
 */

#if defined __INTEL_COMPILER
  #pragma warning disable 2196 2536 279
#endif

#if defined _MSC_VER
  // 4244 : conversion from 'type1' to 'type2', possible loss of data
  // 4661 : no suitable definition provided for explicit template instantiation reques
  // 4503 : decorated name length exceeded, name was truncated
  // 4146 : unary minus operator applied to unsigned type, result still unsigned
  #pragma warning (disable: 4018 4244 4267 4521 4251 4661 4305 4503 4146)
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>

#include <boost/cstdint.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

//Eigen has an enum that clashes with X11 Success define, which is ultimately included by pcl
#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>

#include <pcl/pcl_config.h>

namespace pcl
{
  /**
   * \brief Alias for boost::shared_ptr
   *
   * For ease of switching from boost::shared_ptr to std::shared_ptr
   *
   * \see pcl::make_shared
   * \tparam T Type of the object stored inside the shared_ptr
   */
  template <typename T>
  using shared_ptr = boost::shared_ptr<T>;

  using uint8_t [[deprecated("use std::uint8_t instead of pcl::uint8_t")]] = std::uint8_t;
  using int8_t [[deprecated("use std::int8_t instead of pcl::int8_t")]] = std::int8_t;
  using uint16_t [[deprecated("use std::uint16_t instead of pcl::uint16_t")]] = std::uint16_t;
  using int16_t [[deprecated("use std::uint16_t instead of pcl::int16_t")]] = std::int16_t;
  using uint32_t [[deprecated("use std::uint32_t instead of pcl::uint32_t")]] = std::uint32_t;
  using int32_t [[deprecated("use std::int32_t instead of pcl::int32_t")]] = std::int32_t;
  using uint64_t [[deprecated("use std::uint64_t instead of pcl::uint64_t")]] = std::uint64_t;
  using int64_t [[deprecated("use std::int64_t instead of pcl::int64_t")]] = std::int64_t;
  using int_fast16_t [[deprecated("use std::int_fast16_t instead of pcl::int_fast16_t")]] = std::int_fast16_t;
}

#if defined _WIN32 && defined _MSC_VER

// Define math constants, without including math.h, to prevent polluting global namespace with old math methods
// Copied from math.h
#ifndef _MATH_DEFINES_DEFINED
  #define _MATH_DEFINES_DEFINED

  #define M_E        2.71828182845904523536   // e
  #define M_LOG2E    1.44269504088896340736   // log2(e)
  #define M_LOG10E   0.434294481903251827651  // log10(e)
  #define M_LN2      0.693147180559945309417  // ln(2)
  #define M_LN10     2.30258509299404568402   // ln(10)
  #define M_PI       3.14159265358979323846   // pi
  #define M_PI_2     1.57079632679489661923   // pi/2
  #define M_PI_4     0.785398163397448309616  // pi/4
  #define M_1_PI     0.318309886183790671538  // 1/pi
  #define M_2_PI     0.636619772367581343076  // 2/pi
  #define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
  #define M_SQRT2    1.41421356237309504880   // sqrt(2)
  #define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)
#endif

// Stupid. This should be removed when all the PCL dependencies have min/max fixed.
#ifndef NOMINMAX
# define NOMINMAX
#endif

# define __PRETTY_FUNCTION__ __FUNCTION__
# define __func__ __FUNCTION__

#endif //defined _WIN32 && defined _MSC_VER


template<typename T>
[[deprecated("use std::isnan instead of pcl_isnan")]]
bool pcl_isnan (T&& x) { return std::isnan (std::forward<T> (x)); }

template<typename T>
[[deprecated("use std::isfinite instead of pcl_isfinite")]]
bool pcl_isfinite (T&& x) { return std::isfinite (std::forward<T> (x)); }

template<typename T>
[[deprecated("use std::isinf instead of pcl_isinf")]]
bool pcl_isinf (T&& x) { return std::isinf (std::forward<T> (x)); }


#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

/** \brief Macro that maps version information given by major.minor.patch to a linear integer value to enable easy comparison
 */
#define PCL_LINEAR_VERSION(major,minor,patch) ((major)<<16|(minor)<<8|(patch))

/** Win32 doesn't seem to have rounding functions.
  * Therefore implement our own versions of these functions here.
  */

__inline double
pcl_round (double number)
{
  return (number < 0.0 ? std::ceil (number - 0.5) : std::floor (number + 0.5));
}
__inline float
pcl_round (float number)
{
  return (number < 0.0f ? std::ceil (number - 0.5f) : std::floor (number + 0.5f));
}

#ifdef __GNUC__
#define pcl_lrint(x) (lrint(static_cast<double> (x)))
#define pcl_lrintf(x) (lrintf(static_cast<float> (x)))
#else
#define pcl_lrint(x) (static_cast<long int>(pcl_round(x)))
#define pcl_lrintf(x) (static_cast<long int>(pcl_round(x)))
#endif

#ifdef WIN32
#define pcl_sleep(x) Sleep(1000*(x))
#else
#define pcl_sleep(x) sleep(x)
#endif

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif
#ifndef PVARN
#define PVARN(s) \
  #s << " = " << (s) << "\n"
#endif
#ifndef PVARC
#define PVARC(s) \
  #s << " = " << (s) << ", " << std::flush
#endif
#ifndef PVARS
#define PVARS(s) \
  #s << " = " << (s) << " " << std::flush
#endif
#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif
#ifndef PVARAN
#define PVARAN(s) \
  #s << " = " << RAD2DEG(s) << "deg\n"
#endif
#ifndef PVARAC
#define PVARAC(s) \
  #s << " = " << RAD2DEG(s) << "deg, " << std::flush
#endif
#ifndef PVARAS
#define PVARAS(s) \
  #s << " = " << RAD2DEG(s) << "deg " << std::flush
#endif

#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)

#ifndef ERASE_STRUCT
#define ERASE_STRUCT(var) memset(&var, 0, sizeof(var))
#endif

#ifndef ERASE_ARRAY
#define ERASE_ARRAY(var, size) memset(var, 0, size*sizeof(*var))
#endif

#ifndef SET_ARRAY
#define SET_ARRAY(var, value, size) { for (decltype(size) i = 0; i < size; ++i) var[i]=value; }
#endif

#ifndef PCL_EXTERN_C
    #ifdef __cplusplus
        #define PCL_EXTERN_C extern "C"
    #else
        #define PCL_EXTERN_C
    #endif
#endif

#if defined WIN32 || defined _WIN32 || defined WINCE || defined __MINGW32__
    #ifdef PCLAPI_EXPORTS
        #define PCL_EXPORTS __declspec(dllexport)
    #else
        #define PCL_EXPORTS
    #endif
#else
    #define PCL_EXPORTS
#endif

#if defined WIN32 || defined _WIN32
    #define PCL_CDECL __cdecl
    #define PCL_STDCALL __stdcall
#else
    #define PCL_CDECL
    #define PCL_STDCALL
#endif

#ifndef PCLAPI
    #define PCLAPI(rettype) PCL_EXTERN_C PCL_EXPORTS rettype PCL_CDECL
#endif

// Macro for pragma operator
#if (defined (__GNUC__) || defined(__clang__))
  #define PCL_PRAGMA(x) _Pragma (#x)
#elif _MSC_VER
  #define PCL_PRAGMA(x) __pragma (#x)
#else
  #define PCL_PRAGMA
#endif

// Macro for emitting pragma warning
#if (defined (__GNUC__) || defined(__clang__))
  #define PCL_PRAGMA_WARNING(x) PCL_PRAGMA (GCC warning x)
#elif _MSC_VER
  #define PCL_PRAGMA_WARNING(x) PCL_PRAGMA (warning (x))
#else
  #define PCL_PRAGMA_WARNING
#endif

//for clang cf. http://clang.llvm.org/docs/LanguageExtensions.html
#ifndef __has_extension
  #define __has_extension(x) 0 // Compatibility with pre-3.0 compilers.
#endif

#if defined (__GNUC__) || defined (__PGI) || defined (__IBMCPP__) || defined (__SUNPRO_CC)
  #define PCL_ALIGN(alignment) __attribute__((aligned(alignment)))
#elif defined (_MSC_VER)
  #define PCL_ALIGN(alignment) __declspec(align(alignment))
#else
  #error Alignment not supported on your platform
#endif

#if defined(__GLIBC__) && PCL_LINEAR_VERSION(__GLIBC__,__GLIBC_MINOR__,0)>PCL_LINEAR_VERSION(2,8,0)
  #define GLIBC_MALLOC_ALIGNED 1
#else
  #define GLIBC_MALLOC_ALIGNED 0
#endif

#if defined(__FreeBSD__) && !defined(__arm__) && !defined(__mips__)
  #define FREEBSD_MALLOC_ALIGNED 1
#else
  #define FREEBSD_MALLOC_ALIGNED 0
#endif

#if defined(__APPLE__) || defined(_WIN64) || GLIBC_MALLOC_ALIGNED || FREEBSD_MALLOC_ALIGNED
  #define MALLOC_ALIGNED 1
#endif

#if defined (HAVE_MM_MALLOC)
  // Intel compiler defines an incompatible _mm_malloc signature
  #if defined(__INTEL_COMPILER)
    #include <malloc.h>
  #else
    #include <mm_malloc.h>
  #endif
#endif

inline void*
aligned_malloc (std::size_t size)
{
  void *ptr;
#if   defined (MALLOC_ALIGNED)
  ptr = std::malloc (size);
#elif defined (HAVE_POSIX_MEMALIGN)
  if (posix_memalign (&ptr, 16, size))
    ptr = 0;
#elif defined (HAVE_MM_MALLOC)
  ptr = _mm_malloc (size, 16);
#elif defined (_MSC_VER)
  ptr = _aligned_malloc (size, 16);
#elif defined (ANDROID)
  ptr = memalign (16, size);
#else
  #error aligned_malloc not supported on your platform
  ptr = 0;
#endif
  return (ptr);
}

inline void
aligned_free (void* ptr)
{
#if   defined (MALLOC_ALIGNED) || defined (HAVE_POSIX_MEMALIGN)
  std::free (ptr);
#elif defined (HAVE_MM_MALLOC)
  _mm_free (ptr);
#elif defined (_MSC_VER)
  _aligned_free (ptr);
#elif defined (ANDROID)
  free (ptr);
#else
  #error aligned_free not supported on your platform
#endif
}

/**
 * \brief Macro to signal a class requires a custom allocator
 *
 *  It's an implementation detail to have pcl::has_custom_allocator work, a
 *  thin wrapper over Eigen's own macro
 *
 * \see pcl::has_custom_allocator, pcl::make_shared
 * \ingroup common
 */
#define PCL_MAKE_ALIGNED_OPERATOR_NEW \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  using _custom_allocator_type_trait = void;

/**
 * \brief Macro to add a no-op or a fallthrough attribute based on compiler feature
 *
 * \ingroup common
 */
#if (__cplusplus >= 201703L) || (defined(_MSC_VER) && (_MSC_VER >= 1910) && (_MSVC_LANG >= 201703L))
  #define PCL_FALLTHROUGH [[fallthrough]];
#elif defined(__clang__)
  #define PCL_FALLTHROUGH [[clang::fallthrough]];
#elif defined(__GNUC__) && (__GNUC__ >= 7)
  #define PCL_FALLTHROUGH [[gnu::fallthrough]];
#else
  #define PCL_FALLTHROUGH
#endif

#if (__cplusplus >= 201703L) || (defined(_MSC_VER) && (_MSC_VER >= 1911) && (_MSVC_LANG >= 201703L))
  #define PCL_NODISCARD [[nodiscard]]
#elif defined(__clang__) && (PCL_LINEAR_VERSION(__clang_major__, __clang_minor__, 0) >= PCL_LINEAR_VERSION(3, 9, 0))
  #define PCL_NODISCARD [[clang::warn_unused_result]]
#elif defined(__GNUC__)
  #define PCL_NODISCARD [[gnu::warn_unused_result]]
#else
  #define PCL_NODISCARD
#endif
