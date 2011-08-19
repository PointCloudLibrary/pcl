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

#ifndef PCL_GPU_OCTREE_SYNCSTEP_HPP_
#define PCL_GPU_OCTREE_SYNCSTEP_HPP_


#include "cta_base.hpp"
#include "octree_global.hpp"
#include "tasks_global.hpp"

namespace pcl
{
    namespace device
    {
        namespace syncstep
        {
            struct KernelSyncPolicy
            {        
                enum 
                {
                    LEVEL_BITS_NUM = 3,
                    ARITY = 1 << LEVEL_BITS_NUM,

#if (__CUDA_ARCH__ >= 200)
                    CTA_SIZE = 1024,
#else
                    CTA_SIZE = 512,
#endif
                    
                    GRID_SIZE = 1,
                    PROCESS_STRIDE = CTA_SIZE * GRID_SIZE
                };

                struct SmemStorage
                {                   
                    int offsets[CTA_SIZE];            
                    int old_nodes_num;
                    int old_out_tasks_num;
                };
            };

            template<typename KernelPolicy>
            struct Cta : public pcl::device::base::Cta_base
            {
                typedef typename KernelPolicy::SmemStorage SmemStorage;

                SmemStorage& storage;
                TasksGlobal tasks_global;

                __device__ __forceinline__ Cta(OctreeGlobal& octree_global_arg, const int *codes_arg, TasksGlobal& tasks_global_arg, SmemStorage& storage_arg) 
                    : Cta_base(octree_global_arg, codes_arg), tasks_global(tasks_global_arg), storage(storage_arg) {}

                __device__ __forceinline__ void buildPiece(const int *tasksPiece, int countLeft)
                {
                    int tid = threadIdx.x;
                    int idx = threadIdx.x + blockIdx.x * blockDim.x;

                    int task;
                    int level;

                    int  cell_begs[KernelPolicy::ARITY + 1];
                    char cell_code[KernelPolicy::ARITY];

                    int cell_count = 0;

                    if (idx < countLeft)
                    {
                        int task_info = tasksPiece[idx];

                        task  = task_info >> 8;
                        level = task_info & 0xFF; 

                        cell_count = FindCells(task, level, cell_begs, cell_code);

                    }

                    storage.offsets[tid] = cell_count;

                    __syncthreads();

                    //TODO scans whole block, but can only if (tid < taskCount),  ( optional: + 1 to calcculate some thing total new)                
                    scan_block<pcl::device::exclusive>(storage.offsets);

                    //__syncthreads();  //because sync is inside the scan above

                    if (idx < countLeft)
                    {
                        if ((idx == countLeft - 1) || (tid == KernelPolicy::CTA_SIZE - 1))
                        {                    
                            int total_new = cell_count + storage.offsets[tid];

                            storage.old_nodes_num     = atomicAdd(octree_global.nodes_num,    total_new);                    
                            storage.old_out_tasks_num = atomicAdd(tasks_global.outCountPtr(), total_new);
                        }

                        __syncthreads();

                        if (cell_count > 0)
                        {
                            int parent_code_shifted = octree_global.codes[task] << KernelPolicy::LEVEL_BITS_NUM;

                            int offset = storage.offsets[tid];

                            int mask = 0;
                            for(int i = 0; i < cell_count; ++i)
                            {
                                octree_global.begs [storage.old_nodes_num + offset + i] = cell_begs[i];
                                octree_global.ends [storage.old_nodes_num + offset + i] = cell_begs[i + 1];
                                octree_global.codes[storage.old_nodes_num + offset + i] = parent_code_shifted + cell_code[i];
                                octree_global.parent[storage.old_nodes_num + offset + i] = task;
                                mask |= (1 << cell_code[i]);

                                tasks_global.outLine()[storage.old_out_tasks_num + offset + i] = ((storage.old_nodes_num + offset + i) << 8) + (level + 1);
                            }
                            octree_global.nodes[task] = ((storage.old_nodes_num + offset) << 8) + mask;
                        }            
                        __syncthreads();                
                    }
                }
            };

            template<typename KernelPolicy>
            __global__ void Kernel(TasksGlobal tasks_global, OctreeGlobal octree_global, const int *codes)
            {   
                typedef typename KernelPolicy::SmemStorage SmemStorage;

                __shared__ SmemStorage storage;                        

                Cta<KernelPolicy> cta(octree_global, codes, tasks_global, storage);

                for(;;)
                {            
                    int activeCount = *cta.tasks_global.activeCountPtr();

                    if (activeCount == 0)
                        break;

                    const int *tasksPiece = cta.tasks_global.activeLine();

                    while(activeCount > 0)
                    {                
                        cta.buildPiece(tasksPiece, activeCount);       
                        tasksPiece  += KernelPolicy::PROCESS_STRIDE;
                        activeCount -= KernelPolicy::PROCESS_STRIDE;
                    }

                    __syncthreads();
                    

                    cta.tasks_global.swap();

                    if (blockIdx.x == 0 && threadIdx.x == 0)
                        *cta.tasks_global.outCountPtr() = 0;

                    __syncthreads();                    
                }                
            }
        }
    }
}

#endif /* PCL_GPU_OCTREE_SYNCSTEP_HPP_ */
