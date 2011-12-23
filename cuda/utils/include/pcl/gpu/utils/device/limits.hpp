/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
*/

#ifndef PCL_DEVICE_NUMERIC_LIMITS_HPP_
#define PCL_DEVICE_NUMERIC_LIMITS_HPP_

namespace pcl 
{ 
    namespace device
    {
        template<class T> struct numeric_limits
        {
            typedef T type;
            __device__ __forceinline__ static type min()  { return type(); };
            __device__ __forceinline__ static type max() { return type(); };
            __device__ __forceinline__ static type epsilon() { return type(); }
            __device__ __forceinline__ static type round_error() { return type(); }
            __device__ __forceinline__ static type denorm_min()  { return type(); }
            __device__ __forceinline__ static type infinity() { return type(); }
            __device__ __forceinline__ static type quiet_NaN() { return type(); }
            __device__ __forceinline__ static type signaling_NaN() { return T(); }
            static const bool is_signed;
        };

        template<> struct numeric_limits<bool>
        {
            typedef bool type;
            __device__ __forceinline__ static type min() { return false; };
            __device__ __forceinline__ static type max() { return true;  };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = false;
        };

        template<> struct numeric_limits<char>
        {
            typedef char type;
            __device__ __forceinline__ static type min() { return CHAR_MIN; };
            __device__ __forceinline__ static type max() { return CHAR_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = (char)-1 == -1;
        };

        template<> struct numeric_limits<signed char>
        {
            typedef char type;
            __device__ __forceinline__ static type min() { return CHAR_MIN; };
            __device__ __forceinline__ static type max() { return CHAR_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = (signed char)-1 == -1;
        };

        template<> struct numeric_limits<unsigned char>
        {
            typedef unsigned char type;
            __device__ __forceinline__ static type min() { return 0; };
            __device__ __forceinline__ static type max() { return UCHAR_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = false;
        };

        template<> struct numeric_limits<short>
        {
            typedef short type;
            __device__ __forceinline__ static type min() { return SHRT_MIN; };
            __device__ __forceinline__ static type max() { return SHRT_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = true;
        };

        template<> struct numeric_limits<unsigned short>
        {
            typedef unsigned short type;
            __device__ __forceinline__ static type min() { return 0; };
            __device__ __forceinline__ static type max() { return USHRT_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = false;
        };

        template<> struct numeric_limits<int>
        {
            typedef int type;
            __device__ __forceinline__ static type min() { return INT_MIN; };
            __device__ __forceinline__ static type max() { return INT_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = true;
        };


        template<> struct numeric_limits<unsigned int>
        {
            typedef unsigned int type;
            __device__ __forceinline__ static type min() { return 0; };
            __device__ __forceinline__ static type max() { return UINT_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = false;
        };

        template<> struct numeric_limits<long>
        {
            typedef long type;
            __device__ __forceinline__ static type min() { return LONG_MIN; };
            __device__ __forceinline__ static type max() { return LONG_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = true;
        };

        template<> struct numeric_limits<unsigned long>
        {
            typedef unsigned long type;
            __device__ __forceinline__ static type min() { return 0; };
            __device__ __forceinline__ static type max() { return ULONG_MAX; };
            __device__ __forceinline__ static type epsilon();
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = false;
        };

        template<> struct numeric_limits<float>
        {
            typedef float type;
            __device__ __forceinline__ static type min() { return 1.175494351e-38f/*FLT_MIN*/; };
            __device__ __forceinline__ static type max() { return 3.402823466e+38f/*FLT_MAX*/; };
            __device__ __forceinline__ static type epsilon() { return 1.192092896e-07f/*FLT_EPSILON*/; };
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity() { return __int_as_float(0x7f800000); /*CUDART_INF_F*/ };
            __device__ __forceinline__ static type quiet_NaN() { return __int_as_float(0x7fffffff); /*CUDART_NAN_F*/ };
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = true;
        };

        template<> struct numeric_limits<double>
        {
            typedef double type;
            __device__ __forceinline__ static type min() { return 2.2250738585072014e-308/*DBL_MIN*/; };
            __device__ __forceinline__ static type max() { return 1.7976931348623158e+308/*DBL_MAX*/; };
			__device__ __forceinline__ static type epsilon() { return 2.2204460492503131e-016 /*DBL_EPSILON*/; };
            __device__ __forceinline__ static type round_error();
            __device__ __forceinline__ static type denorm_min();
            __device__ __forceinline__ static type infinity();
            __device__ __forceinline__ static type quiet_NaN();
            __device__ __forceinline__ static type signaling_NaN();
            static const bool is_signed = true;
        };
    }
}
#endif /* PCL_DEVICE_NUMERIC_LIMITS_HPP_ */
