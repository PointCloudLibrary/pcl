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

#ifndef PCL_GPU_OCTREE_TASKS_GLOBAL_HPP
#define PCL_GPU_OCTREE_TASKS_GLOBAL_HPP

namespace pcl
{
    namespace device
    {
        struct TasksGlobal
        {           
            int *line[2];
            int *count[2];
            int active_selector;  

            __device__ __host__ __forceinline__ const int* activeLine()     const { return  line[active_selector]; }
            __device__ __host__ __forceinline__ const int* activeCountPtr() const { return count[active_selector]; }


            __device__ __host__ __forceinline__ int* outLine()     { return  line[nextSelector()]; }
            __device__ __host__ __forceinline__ int* outCountPtr() { return count[nextSelector()]; }

            __device__ __host__ __forceinline__ void swap() { active_selector = nextSelector(); }
            __device__ __host__ __forceinline__ int nextSelector() const { return (active_selector + 1) & 1; }
        };

        struct TasksGlobalLifeTime : public TasksGlobal
        {    
            size_t allocated_size;

            TasksGlobalLifeTime() : allocated_size(0) 
            {
                active_selector = 0;
            }

            TasksGlobalLifeTime(size_t size)
            {
                create(size);
            }

            void create(size_t size)
            {
                active_selector = 0;
                allocated_size = size;

                cudaSafeCall( cudaMalloc((void**)&line[0], size * sizeof(int)) );
                cudaSafeCall( cudaMalloc((void**)&line[1], size * sizeof(int)) );
                cudaSafeCall( cudaMalloc((void**)&count[0], sizeof(int)) );        
                cudaSafeCall( cudaMalloc((void**)&count[1], sizeof(int)) );

                cudaSafeCall( cudaMemset(count[0], 0, sizeof(int)) );        
                cudaSafeCall( cudaMemset(count[1], 0, sizeof(int)) );        
            }

            ~TasksGlobalLifeTime()
            {
                if ( line[0]) cudaSafeCall( cudaFree( line[0]) );
                if ( line[1]) cudaSafeCall( cudaFree( line[1]) );
                if (count[0]) cudaSafeCall( cudaFree(count[0]) );
                if (count[1]) cudaSafeCall( cudaFree(count[1]) );
            }
        };

    } // namespace device

} // namespace pcl

#endif /* CL_GPU_OCTREE_TASKS_GLOBAL_HPP */