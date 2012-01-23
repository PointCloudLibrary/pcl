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

#ifndef PCL_GPU_OCTREE_SCAN_BLOCK_HPP
#define PCL_GPU_OCTREE_SCAN_BLOCK_HPP


namespace pcl
{
    namespace device
    {
        enum ScanKind { exclusive,  inclusive } ;

        template <ScanKind Kind , class T> 
        __device__ __forceinline__ T scan_warp ( volatile T *ptr , const unsigned int idx = threadIdx.x )
        {
            const unsigned int lane = idx & 31; // index of thread in warp (0..31)

            if ( lane >=  1) ptr [idx ] = ptr [idx -  1] + ptr [idx];
            if ( lane >=  2) ptr [idx ] = ptr [idx -  2] + ptr [idx];
            if ( lane >=  4) ptr [idx ] = ptr [idx -  4] + ptr [idx];
            if ( lane >=  8) ptr [idx ] = ptr [idx -  8] + ptr [idx];
            if ( lane >= 16) ptr [idx ] = ptr [idx - 16] + ptr [idx];

            if( Kind == inclusive ) 
                return ptr [idx ];
            else 
                return (lane > 0) ? ptr [idx - 1] : 0;
        }

        template <ScanKind Kind , class T>
        __device__ __forceinline__ T scan_block( volatile T *ptr , const unsigned int idx = threadIdx.x )
        {        
            const unsigned int lane = idx & 31;
            const unsigned int warpid = idx >> 5;

            // Step 1: Intra - warp scan in each warp
            T val = scan_warp <Kind>( ptr , idx );

            __syncthreads ();    

            // Step 2: Collect per - warp partial results

            /*  if( warpid == 0 ) 
            if( lane == 31 ) 
            ptr [ warpid ] = ptr [idx ];    

            __syncthreads ();

            if( warpid > 0 ) */
            if( lane == 31 ) 
                ptr [ warpid ] = ptr [idx ];    

            __syncthreads ();

            // Step 3: Use 1st warp to scan per - warp results
            if( warpid == 0 ) 
                scan_warp<inclusive>( ptr , idx );

            __syncthreads ();

            // Step 4: Accumulate results from Steps 1 and 3
            if ( warpid > 0) 
                val = ptr [warpid -1] + val;

            __syncthreads ();

            // Step 5: Write and return the final result
            ptr[idx] = val;

            __syncthreads ();

            return val ;
        }
    }
}

#endif /* PCL_GPU_OCTREE_SCAN_BLOCK_HPP */