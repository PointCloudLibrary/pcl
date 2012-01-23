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

#ifndef PCL_GPU_BITONIC_SORT_WARP_HPP
#define PCL_GPU_BITONIC_SORT_WARP_HPP

namespace pcl
{
    namespace device
    {
        template<typename T>    
        __device__ __forceinline__ void swap(T& a, T& b) { T t = a; a = b; b = t; }

        template<typename V, typename K>
        __device__ __forceinline__ void bitonicSortWarp(volatile K* keys, volatile V* vals, unsigned int dir = 1)
        {
            const unsigned int arrayLength = 64;   
            unsigned int lane = threadIdx.x & 31;

            for(unsigned int size = 2; size < arrayLength; size <<= 1)
            {
                //Bitonic merge
                unsigned int ddd = dir ^ ( (lane & (size / 2)) != 0 );

                for(unsigned int stride = size / 2; stride > 0; stride >>= 1)
                {            
                    unsigned int pos = 2 * lane - (lane & (stride - 1));

                    if ( (keys[pos] > keys[pos + stride]) == ddd )
                    {
                        swap(keys[pos], keys[pos + stride]);
                        swap(vals[pos], vals[pos + stride]);
                    }            
                }
            }

            //ddd == dir for the last bitonic merge step
            for(unsigned int stride = arrayLength / 2; stride > 0; stride >>= 1)
            {        
                unsigned int pos = 2 * lane - (lane & (stride - 1));

                if ( (keys[pos] > keys[pos + stride]) == dir )
                {
                    swap(keys[pos], keys[pos + stride]);
                    swap(vals[pos], vals[pos + stride]);
                }     
            }
        }

    }
}

#endif /* PCL_GPU_BITONIC_SORT_WARP_HPP */