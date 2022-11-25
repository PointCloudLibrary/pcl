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
#include <cstdlib>
#include <iostream>

// We need to check for GCC version, because GCC releases before 9 were implementing an
// OpenMP 3.1 data sharing rule, even OpenMP 4 is supported, so a plain OpenMP version 4 check
// isn't enough (see https://www.gnu.org/software/gcc/gcc-9/porting_to.html#ompdatasharing)
#if (defined _OPENMP && (_OPENMP <= 201307)) || (defined __GNUC__ && (__GNUC__ >= 6 && __GNUC__ < 9))
  #define OPENMP_LEGACY_CONST_DATA_SHARING_RULE 1
#else
  #define OPENMP_LEGACY_CONST_DATA_SHARING_RULE 0
#endif

#include <pcl/pcl_config.h>

#include <boost/preprocessor/arithmetic/add.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/comparison/less.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/stringize.hpp>

// MSVC < 2019 have issues:
// * can't short-circuiting logic in macros
// * don't define standard macros
// => this leads to annyoing C4067 warnings (see https://developercommunity.visualstudio.com/content/problem/327796/-has-cpp-attribute-emits-warning-is-wrong-highligh.html)
#if defined(_MSC_VER)
  // nvcc on msvc can't work with [[deprecated]]
  #if !defined(__CUDACC__)
    #define _PCL_DEPRECATED_IMPL(Message) [[deprecated(Message)]]
  #else
    #define _PCL_DEPRECATED_IMPL(Message)
  #endif
#elif __has_cpp_attribute(deprecated)
  #define _PCL_DEPRECATED_IMPL(Message) [[deprecated(Message)]]
#else
  #warning "You need to implement _PCL_DEPRECATED_IMPL for this compiler"
  #define _PCL_DEPRECATED_IMPL(Message)
#endif

// Macro for pragma operator
#if (defined (__GNUC__) || defined(__clang__))
  #define PCL_PRAGMA(x) _Pragma (#x)
#elif _MSC_VER
  #define PCL_PRAGMA(x) __pragma (#x)
#else
  #define PCL_PRAGMA
#endif

// Macro for emitting pragma warning for deprecated headers
#if (defined (__GNUC__) || defined(__clang__))
  #define _PCL_DEPRECATED_HEADER_IMPL(Message) PCL_PRAGMA (message Message)
#elif _MSC_VER
  #define _PCL_DEPRECATED_HEADER_IMPL(Message) PCL_PRAGMA (warning (Message))
#else
  #warning "You need to implement _PCL_DEPRECATED_HEADER_IMPL for this compiler"
  #define _PCL_DEPRECATED_HEADER_IMPL(Message)
#endif

/**
 * \brief A handy way to inform the user of the removal deadline
 */
#define _PCL_PREPARE_REMOVAL_MESSAGE(Major, Minor, Msg)                                 \
  Msg " (It will be removed in PCL " BOOST_PP_STRINGIZE(Major.Minor) ")"

/**
 * \brief Tests for Minor < PCL_MINOR_VERSION
 * \details When PCL VERSION is of format `34.12.99`, this macro behaves as if it is
 * already `34.13.0`, and allows for smoother transition for maintainers
 */
#define _PCL_COMPAT_MINOR_VERSION(Minor, IfPass, IfFail)                                \
  BOOST_PP_IF(BOOST_PP_EQUAL(PCL_REVISION_VERSION, 99),                                 \
    BOOST_PP_IF(BOOST_PP_LESS(BOOST_PP_ADD(PCL_MINOR_VERSION, 1), Minor), IfPass, IfFail),           \
    BOOST_PP_IF(BOOST_PP_LESS(PCL_MINOR_VERSION, Minor), IfPass, IfFail))

/**
 * \brief Tests for Major == PCL_MAJOR_VERSION
 * \details When PCL VERSION is of format `34.99.12`, this macro behaves as if it is
 * already `35.0.0`, and allows for smoother transition for maintainers
 */
#define _PCL_COMPAT_MAJOR_VERSION(Major, IfPass, IfFail)                                \
  BOOST_PP_IF(BOOST_PP_EQUAL(PCL_MINOR_VERSION, 99),                                    \
    BOOST_PP_IF(BOOST_PP_EQUAL(BOOST_PP_ADD(PCL_MAJOR_VERSION, 1), Major), IfPass, IfFail),          \
    BOOST_PP_IF(BOOST_PP_EQUAL(PCL_MAJOR_VERSION, Major), IfPass, IfFail))

/**
 * \brief macro for compatibility across compilers and help remove old deprecated
 *        items for the Major.Minor release
 *
 * \details compiler errors of `unneeded_deprecation` and `major_version_mismatch`
 * are hints to the developer that those items can be safely removed.
 * Behavior of PCL_DEPRECATED(1, 99, "Not needed anymore")
 *   * till PCL 1.98: "Not needed anymore (It will be removed in PCL 1.99)"
 *   * PCL 1.99 onwards: compiler error with "unneeded_deprecation"
 *   * PCL 2.0 onwards: compiler error with "major_version_mismatch"
 */
#define PCL_DEPRECATED(Major, Minor, Message)                                          \
  _PCL_COMPAT_MAJOR_VERSION(                                                           \
      Major,                                                                           \
      _PCL_COMPAT_MINOR_VERSION(                                                       \
          Minor,                                                                       \
          _PCL_DEPRECATED_IMPL(_PCL_PREPARE_REMOVAL_MESSAGE(Major, Minor, Message)),   \
          unneeded_deprecation),                                                       \
      major_version_mismatch)

/**
 * \brief macro for compatibility across compilers and help remove old deprecated
 *        headers for the Major.Minor release
 *
 * \details compiler errors of `unneeded_header` and `major_version_mismatch`
 * are hints to the developer that those items can be safely removed.
 * Behavior of PCL_DEPRECATED_HEADER(1, 99, "Use file <newfile.h> instead.")
 *   * till PCL 1.98: "This header is deprecated. Use file <newfile.h> instead. (It will be removed in PCL 1.99)"
 *   * PCL 1.99 onwards: compiler error with "unneeded_header"
 *   * PCL 2.0 onwards: compiler error with "major_version_mismatch"
 */
#define PCL_DEPRECATED_HEADER(Major, Minor, Message)                                   \
  _PCL_COMPAT_MAJOR_VERSION(                                                           \
      Major,                                                                           \
      _PCL_COMPAT_MINOR_VERSION(                                                       \
          Minor,                                                                       \
          _PCL_DEPRECATED_HEADER_IMPL(_PCL_PREPARE_REMOVAL_MESSAGE(                    \
              Major,                                                                   \
              Minor,                                                                   \
              "This header is deprecated. " Message)),                                 \
          unneeded_header),                                                            \
      major_version_mismatch)

#if defined _WIN32
// Define math constants, without including math.h, to prevent polluting global namespace with old math methods
// Copied from math.h
// Check for M_2_SQRTPI since the cmath header on mingw-w64 doesn't seem to define
// _MATH_DEFINES_DEFINED when included with _USE_MATH_DEFINES
#if !defined _MATH_DEFINES_DEFINED && !defined M_2_SQRTPI
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

#if defined _MSC_VER
  // Stupid. This should be removed when all the PCL dependencies have min/max fixed.
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif

  #define __PRETTY_FUNCTION__ __FUNCTION__
  #define __func__ __FUNCTION__
#endif
#endif // defined _WIN32

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

namespace pcl {

inline void*
aligned_malloc(std::size_t size)
{
  void* ptr;
#if defined(MALLOC_ALIGNED)
  ptr = std::malloc(size);
#elif defined(HAVE_POSIX_MEMALIGN)
  if (posix_memalign(&ptr, 16, size))
    ptr = 0;
#elif defined(HAVE_MM_MALLOC)
  ptr = _mm_malloc(size, 16);
#elif defined(_MSC_VER)
  ptr = _aligned_malloc(size, 16);
#elif defined(ANDROID)
  ptr = memalign(16, size);
#else
#error aligned_malloc not supported on your platform
  ptr = 0;
#endif
  return (ptr);
}

inline void
aligned_free(void* ptr)
{
#if defined(MALLOC_ALIGNED) || defined(HAVE_POSIX_MEMALIGN)
  std::free(ptr);
#elif defined(HAVE_MM_MALLOC)
  _mm_free(ptr);
#elif defined(_MSC_VER)
  _aligned_free(ptr);
#elif defined(ANDROID)
  free(ptr);
#else
#error aligned_free not supported on your platform
#endif
}

} // namespace pcl

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

#ifdef __cpp_if_constexpr
  #define PCL_IF_CONSTEXPR(x) if constexpr(x)
#else
  #define PCL_IF_CONSTEXPR(x) if (x)
#endif

// [[unlikely]] can be used on any conditional branch, but __builtin_expect is restricted to the evaluation point
// This makes it quite difficult to create a single macro for switch and while/if
/**
 * @def PCL_CONDITION_UNLIKELY
 * @brief Tries to inform the compiler to optimize codegen assuming that the condition will probably evaluate to false
 * @note Prefer using `PCL_{IF,WHILE}_UNLIKELY`
 * @warning This can't be used with switch statements
 * @details This tries to help the compiler optimize for the unlikely case.
 * Most compilers assume that the condition would evaluate to true in if and while loops (reference needed)
 * As such the opposite of this macro (PCL_CONDITION_LIKELY) will not result in significant performance improvement
 * 
 * Some sample usage:
 * @code{.cpp}
 * if PCL_CONDITION_UNLIKELY(x == 0) { return; } else { throw std::runtime_error("some error"); }
 * //
 * while PCL_CONDITION_UNLIKELY(wait_for_result) { sleep(1); }  // busy wait, with minimal chances of waiting
 * @endcode
 */
#if __has_cpp_attribute(unlikely)
  #define PCL_CONDITION_UNLIKELY(x) (static_cast<bool>(x)) [[unlikely]]
#elif defined(__GNUC__)
  #define PCL_CONDITION_UNLIKELY(x) (__builtin_expect(static_cast<bool>(x), 0))
#elif defined(__clang__) && (PCL_LINEAR_VERSION(__clang_major__, __clang_minor__, 0) >= PCL_LINEAR_VERSION(3, 9, 0))
  #define PCL_CONDITION_UNLIKELY(x) (__builtin_expect(static_cast<bool>(x), 0))
#else  // MSVC has no such alternative
  #define PCL_CONDITION_UNLIKELY(x) (x)
#endif

#define PCL_IF_UNLIKELY(x) if PCL_CONDITION_UNLIKELY(x)
#define PCL_WHILE_UNLIKELY(x) while PCL_CONDITION_UNLIKELY(x)
