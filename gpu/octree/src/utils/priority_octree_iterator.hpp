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

#ifndef PCL_GPU_OCTREE_PRIORITY_ITERATOR
#define PCL_GPU_OCTREE_PRIORITY_ITERATOR

namespace pcl
{
    namespace device
    {       
        struct OctreePriorityIteratorDevice
        {       
            int level;
            int node_idx;
            int length;
            const OctreeGlobalWithBox& octree;

            __device__ __forceinline__ OctreePriorityIteratorDevice(const OctreeGlobalWithBox& octree_arg) : octree(octree_arg)
            {
                node_idx = 0;
                level = 0;
                length = 1;
            }

            __device__ __forceinline__ void gotoNextLevel(int first, int len) 
            {  
                node_idx = first;
                length = len;
                ++level;
            }       

            __device__ __forceinline__ int operator*() const 
            { 
                return node_idx; 
            }        

            __device__ __forceinline__ void operator++()
            {
#if 1
                while(level >= 0)
                {                
                    if (length > 1)
                    {
                        length--;
                        node_idx++;                      
                        break;
                    }

                    if (node_idx == 0)
                    {
                        level = -1;
                        return;
                    }                

                    node_idx = octree.parent[node_idx];
                    --level;
                    if (node_idx == 0)
                    {
                        level = -1;
                        return;
                    }

                    int parent = octree.nodes[octree.parent[node_idx]];
                    int parent_first = parent >> 8;
                    int parent_len = __popc(parent & 0xFF);

                    int pos = node_idx - parent_first;

                    length = parent_len - pos;
                }
#endif
            }

        };
    }
}

#endif /* PCL_GPU_OCTREE_PRIORITY_ITERATOR */