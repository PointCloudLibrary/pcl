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

#include <boost/cstdint.hpp>

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
}

// MSCV doesn't have std::{isnan,isfinite}
#if defined _WIN32 && defined _MSC_VER

// Stupid. This should be removed when all the PCL dependencies have min/max fixed.
# define NOMINMAX

# define pcl_isnan(x)    _isnan(x)
# define pcl_isfinite(x) _finite(x)
# define pcl_isinf(x)    (!_finite(x))

# define __PRETTY_FUNCTION__ __FUNCTION__
# define __func__ __FUNCTION__

#elif ANDROID
// Use the math.h macros
# include <math.h>
# define pcl_isnan(x)    isnan(x)
# define pcl_isfinite(x) isfinite(x)
# define pcl_isinf(x)    isinf(x)

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

// Windows doesn't like M_PI.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_E
#define M_E 2.7182818284590452354
#endif

#ifndef M_LN2
#define M_LN2 0.693147180559945309417
#endif

/** Win32 doesn't seem to have rounding functions.
  * Therefore implement our own versions of these functions here.
  */
#include <math.h>
__inline double pcl_round(double number)
{
  return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}
__inline float pcl_round(float number)
{
  return number < 0.0f ? ceil(number - 0.5f) : floor(number + 0.5f);
}

#define pcl_lrint(x) ((long int) pcl_round(x))
#define pcl_lrintf(x) ((long int) pcl_round(x))

#ifdef WIN32
#define pcl_sleep(x) Sleep(1000*(x))
#else
#define pcl_sleep(x) sleep(x)
#endif

#endif  //#ifndef PCL_WIN32_MACROS_H_
