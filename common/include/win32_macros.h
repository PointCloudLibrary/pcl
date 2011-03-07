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
 */

#ifndef PCL_WIN32_MACROS_H_
#define PCL_WIN32_MACROS_H_

// Windowz doesn't have std::{isnan,isfinite}
#ifdef _WIN32

// Stupid. This should be removed when all the PCL dependencies have min/max fixed.
# define NOMINMAX

# define pcl_isnan(x)    _isnan(x)
# define pcl_isfinite(x) _finite(x)
# define pcl_isinf(x)    (!_finite(x))

# define lrint(x) (floor(x+(x>0) ? 0.5 : -0.5))

# define __PRETTY_FUNCTION__ __FUNCTION__
# define __func__ __FUNCTION__

#elif ANDROID
# include <math.h>
# define pcl_isnan(x)    isnan(x)
# define pcl_isfinite(x) isfinite(x)
# define pcl_isinf(x)    isinf(x)

#else

# define pcl_isnan(x)    std::isnan(x)
# define pcl_isfinite(x) std::isfinite(x)
# define pcl_isinf(x)    std::isinf(x)

#endif

// Windows doesn't like M_PI.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_E
#define M_E 2.7182818284590452354
#endif

// Generic helper definitions for shared library support
// see http://gcc.gnu.org/wiki/Visibility for more information
#if defined _WIN32 || defined __CYGWIN__
  #define PCL_HELPER_DLL_IMPORT __declspec(dllimport)
  #define PCL_HELPER_DLL_EXPORT __declspec(dllexport)
  #define PCL_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define PCL_HELPER_DLL_IMPORT __attribute__ ((visibility("default")))
    #define PCL_HELPER_DLL_EXPORT __attribute__ ((visibility("default")))
    #define PCL_HELPER_DLL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_HELPER_DLL_IMPORT
    #define PCL_HELPER_DLL_EXPORT
    #define PCL_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define PCL_API and PCL_LOCAL.
// PCL_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// PCL_LOCAL is used for non-api symbols.

#ifdef PCL_DLL // defined if PCL is compiled as a DLL
  #ifdef PCL_DLL_EXPORTS // defined if we are building the PCL DLL (instead of using it)
    #define PCL_API PCL_HELPER_DLL_EXPORT
  #else
    #define PCL_API PCL_HELPER_DLL_IMPORT
  #endif // PCL_DLL_EXPORTS
  #define PCL_LOCAL PCL_HELPER_DLL_LOCAL
#else // PCL_DLL is not defined: this means PCL is a static lib.
  #define PCL_API
  #define PCL_LOCAL
#endif // PCL_DLL

#endif  //#ifndef PCL_WIN32_MACROS_H_
