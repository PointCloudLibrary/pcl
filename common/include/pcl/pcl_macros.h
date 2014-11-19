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

#ifndef PCL_MACROS_H_
#define PCL_MACROS_H_

#include <pcl/pcl_config.h>
#include <boost/cstdint.hpp>
#include <cstdlib>

namespace pcl
{
  using boost::uint8_t;
  using boost::int8_t;
  using boost::int16_t;
  using boost::uint16_t;
  using boost::int32_t;
  using boost::uint32_t;
  using boost::int64_t;
  using boost::uint64_t;
  using boost::int_fast16_t;
}

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

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

// MSCV doesn't have std::{isnan,isfinite}
#if defined _WIN32 && defined _MSC_VER

// If M_PI is not defined, then probably all of them are undefined
#ifndef M_PI
// Copied from math.h
# define M_PI   3.14159265358979323846     // pi
# define M_PI_2    1.57079632679489661923  // pi/2
# define M_PI_4    0.78539816339744830962  // pi/4
# define M_PIl   3.1415926535897932384626433832795029L  // pi
# define M_PI_2l 1.5707963267948966192313216916397514L  // pi/2
# define M_PI_4l 0.7853981633974483096156608458198757L  // pi/4
#endif

// Stupid. This should be removed when all the PCL dependencies have min/max fixed.
#ifndef NOMINMAX
# define NOMINMAX
#endif

# define pcl_isnan(x)    _isnan(x)
# define pcl_isfinite(x) (_finite(x) != 0)
# define pcl_isinf(x)    (_finite(x) == 0)

# define __PRETTY_FUNCTION__ __FUNCTION__
# define __func__ __FUNCTION__

#elif ANDROID
// Use the math.h macros
# include <math.h>
# define pcl_isnan(x)    std::isnan(x)
# define pcl_isfinite(x) std::isfinite(x)
# define pcl_isinf(x)    std::isinf(x)

#elif _GLIBCXX_USE_C99_MATH
// Are the C++ cmath functions enabled?
# include <cmath>
# define pcl_isnan(x)    std::isnan(x)
# define pcl_isfinite(x) std::isfinite(x)
# define pcl_isinf(x)    std::isinf(x)

#elif __PATHCC__
# include <cmath>
# include <stdio.h>
template <typename T> int
pcl_isnan (T &val)
{
  return (val != val);
}
//# define pcl_isnan(x)    std::isnan(x)
# define pcl_isfinite(x) std::isfinite(x)
# define pcl_isinf(x)    std::isinf(x)

#else
// Use the math.h macros
# include <math.h>
# define pcl_isnan(x)    isnan(x)
# define pcl_isfinite(x) isfinite(x)
# define pcl_isinf(x)    isinf(x)

#endif

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
  return (number < 0.0 ? ceil (number - 0.5) : floor (number + 0.5));
}
__inline float
pcl_round (float number)
{
  return (number < 0.0f ? ceilf (number - 0.5f) : floorf (number + 0.5f));
}

#ifdef __GNUC__
#define pcl_lrint(x) (lrint(static_cast<double> (x)))
#define pcl_lrintf(x) (lrintf(static_cast<float> (x)))
#else
#define pcl_lrint(x) (static_cast<long int>(pcl_round(x)))
#define pcl_lrintf(x) (static_cast<long int>(pcl_round(x)))
#endif


#ifdef _WIN32
__inline float
log2f (float x)
{
  return (static_cast<float> (logf (x) * M_LOG2E));
}
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
#define SET_ARRAY(var, value, size) { for (int i = 0; i < static_cast<int> (size); ++i) var[i]=value; }
#endif

/* //This is copy/paste from http://gcc.gnu.org/wiki/Visibility */
/* #if defined _WIN32 || defined __CYGWIN__ */
/*   #ifdef BUILDING_DLL */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllexport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #else */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllimport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #endif */
/*   #define DLL_LOCAL */
/* #else */
/*   #if __GNUC__ >= 4 */
/* #define DLL_PUBLIC __attribute__ ((visibility("default"))) */
/* #define DLL_LOCAL  __attribute__ ((visibility("hidden"))) */
/*   #else */
/*     #define DLL_PUBLIC */
/*     #define DLL_LOCAL */
/*   #endif */
/* #endif */

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

// Macro to deprecate old functions
//
// Usage:
// don't use me any more
// PCL_DEPRECATED(void OldFunc(int a, float b), "Use newFunc instead, this functions will be gone in the next major release");
// use me instead
// void NewFunc(int a, double b);

//for clang cf. http://clang.llvm.org/docs/LanguageExtensions.html
#ifndef __has_extension
  #define __has_extension(x) 0 // Compatibility with pre-3.0 compilers.
#endif

#if (defined(__GNUC__) && PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) < PCL_LINEAR_VERSION(4,5,0) && ! defined(__clang__)) || defined(__INTEL_COMPILER)
#define PCL_DEPRECATED(message) __attribute__ ((deprecated))
#endif

// gcc supports this starting from 4.5 : http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43666
#if (defined(__GNUC__) && PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) >= PCL_LINEAR_VERSION(4,5,0)) || (defined(__clang__) && __has_extension(attribute_deprecated_with_message))
#define PCL_DEPRECATED(message) __attribute__ ((deprecated(message)))
#endif

#ifdef _MSC_VER
#define PCL_DEPRECATED(message) __declspec(deprecated(message))
#endif

#ifndef PCL_DEPRECATED
#pragma message("WARNING: You need to implement PCL_DEPRECATED for this compiler")
#define PCL_DEPRECATED(message)
#endif


// Macro to deprecate old classes/structs
//
// Usage:
// don't use me any more
// class PCL_DEPRECATED_CLASS(OldClass, "Use newClass instead, this class will be gone in the next major release")
// {
//   public:
//     OldClass() {}
// };
// use me instead
// class NewFunc
// {
//   public:
//     NewClass() {}
// };

#if (defined(__GNUC__) && PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) < PCL_LINEAR_VERSION(4,5,0) && ! defined(__clang__)) || defined(__INTEL_COMPILER)
#define PCL_DEPRECATED_CLASS(func, message) __attribute__ ((deprecated)) func
#endif

// gcc supports this starting from 4.5 : http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43666
#if (defined(__GNUC__) && PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) >= PCL_LINEAR_VERSION(4,5,0)) || (defined(__clang__) && __has_extension(attribute_deprecated_with_message))
#define PCL_DEPRECATED_CLASS(func, message) __attribute__ ((deprecated(message))) func
#endif

#ifdef _MSC_VER
#define PCL_DEPRECATED_CLASS(func, message) __declspec(deprecated(message)) func
#endif

#ifndef PCL_DEPRECATED_CLASS
#pragma message("WARNING: You need to implement PCL_DEPRECATED_CLASS for this compiler")
#define PCL_DEPRECATED_CLASS(func) func
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
#else
  #define MALLOC_ALIGNED 0
#endif

inline void*
aligned_malloc (size_t size)
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
  ptr = _mm_free (ptr);
#elif defined (_MSC_VER)
  _aligned_free (ptr);
#else
  #error aligned_free not supported on your platform
#endif
}

#endif  //#ifndef PCL_MACROS_H_
