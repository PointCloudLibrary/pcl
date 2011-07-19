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

#ifndef PCL_GPU_OCTREE_GLOBAL_HPP
#define PCL_GPU_OCTREE_GLOBAL_HPP

#include "pcl/gpu/common/safe_call.hpp"

namespace pcl
{
    namespace device
    {
        struct OctreeGlobal
        {             
            int *nodes;
            int *codes;
            int *begs;
            int *ends;

            int *nodes_num;

            int *parent;

            OctreeGlobal() : nodes(0), codes(0), begs(0), ends(0), nodes_num(0), parent(0) {}
        };


      /*  template<bool WithParent = false>
        struct OctreeGlobalBase
        { 
            const static bool HasParent = false;

            int *nodes;
            int *codes;
            int *begs;
            int *ends;

            int *nodes_num;

            OctreeGlobalBase() : nodes(0), codes(0), begs(0), ends(0), nodes_num(0) {}
        };

        template<>
        struct OctreeGlobalBase<true> : public OctreeGlobalBase<false>
        {
            const static bool HasParent = true;

            int *parent;
            OctreeGlobalBase() : parent(0) {}
        };

        typedef OctreeGlobalBase<> OctreeGlobal;*/

        struct OctreeGlobalWithBox : public OctreeGlobal
        {    
            float3 minp, maxp;    
        };
        
        struct OctreeGlobalLifetime : public OctreeGlobalWithBox
        {
        public:
            size_t allocated_size;

            OctreeGlobalLifetime() : allocated_size(0) {}
            OctreeGlobalLifetime(size_t size) { create(size); }
            ~OctreeGlobalLifetime() { release(); }   

            void create(size_t size) 
            {   
                allocated_size = size;

                cudaSafeCall( cudaMalloc((void**)&nodes, allocated_size * sizeof(int)) );
                cudaSafeCall( cudaMalloc((void**)&codes, allocated_size * sizeof(int)) );
                cudaSafeCall( cudaMalloc((void**)&begs,  allocated_size * sizeof(int)) );
                cudaSafeCall( cudaMalloc((void**)&ends,  allocated_size * sizeof(int)) );

                cudaSafeCall( cudaMalloc((void**)&nodes_num, sizeof(int))  );  

                //alloc_parent(*this);
            }

            void release()
            {
                if (nodes) { cudaSafeCall( cudaFree(nodes) ); nodes = 0; }
                if (codes) { cudaSafeCall( cudaFree(codes) ); codes = 0; }
                if (begs)  { cudaSafeCall( cudaFree(begs) ); begs = 0; }
                if (ends)  { cudaSafeCall( cudaFree(ends) ); ends = 0; }

                if (nodes_num) { cudaSafeCall( cudaFree(nodes_num) ); nodes_num = 0; }    

                //free_parent(*this);
            }
        private:                                
            /*void alloc_parent(OctreeGlobalBase<false>& rthis) {}
            void  free_parent(OctreeGlobalBase<false>& rthis) {}

            void alloc_parent(OctreeGlobalBase<true>& rthis)
            {                
                cudaSafeCall( cudaMalloc((void**)&rthis.parent,  allocated_size * sizeof(int)) );
            }

            void  free_parent(OctreeGlobalBase<true>& rthis)
            {                
                if (rthis.parent) { cudaSafeCall( cudaFree(rthis.parent) ); rthis.parent = 0; }
            }*/
        };
    } // namespace device

} // namespace pcl

#endif /* PCL_GPU_OCTREE_GLOBAL_HPP */