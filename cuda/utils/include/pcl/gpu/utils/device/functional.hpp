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


#ifndef PCL_DEVICE_FUNCTIONAL_HPP_
#define PCL_DEVICE_FUNCTIONAL_HPP_

#include <thrust/functional.h>

namespace pcl
{
    namespace device
    {
        // Function Objects

        using thrust::unary_function;
        using thrust::binary_function;

        // Arithmetic Operations

        using thrust::plus;
        using thrust::minus;
        using thrust::multiplies;
        using thrust::divides;
        using thrust::modulus;
        using thrust::negate;

        // Comparison Operations

        using thrust::equal_to;
        using thrust::not_equal_to;
        using thrust::greater;
        using thrust::less;
        using thrust::greater_equal;
        using thrust::less_equal;

        // Logical Operations

        using thrust::logical_and;
        using thrust::logical_or;
        using thrust::logical_not;

        // Bitwise Operations

        using thrust::bit_and;
        using thrust::bit_or;
        using thrust::bit_xor;

        template <typename T> struct bit_not : unary_function<T, T>
        {
            __forceinline__ __device__ T operator ()(const T& v) const {return ~v;}
        };

        // Generalized Identity Operations

        using thrust::identity;    
        using thrust::project1st;
        using thrust::project2nd;


        // Other functors

        template<typename T, typename W>
        struct plusWeighted : public plus<T>
        {
            W w;
            __device__ __host__ __forceinline__ plusWeighted(W weight) : w(weight) {}                
            __device__ __host__ __forceinline__ float operator()(const T& f1, const T& f2) const 
            {
                return f1 + f2 * w;
            }
        };
    }
};


#endif /* PCL_DEVICE_FUNCTIONAL_HPP_ */