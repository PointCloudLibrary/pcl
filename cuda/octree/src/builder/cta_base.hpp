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

#ifndef PCL_GPU_OCTREE_STA_BASE_H_
#define PCL_GPU_OCTREE_STA_BASE_H_

#include "octree_global.hpp"
#include "pcl/gpu/utils/device/algorithm.hpp"
#include "pcl/gpu/utils/device/static_check.hpp"
#include "utils/morton.hpp"

namespace pcl
{
    namespace device
    {
        namespace base
        {
            struct Cta_base
            {        
                OctreeGlobal& octree_global;
                const int* codes;
                const static int max_points_per_leaf = 96;

                __device__ __forceinline__ Cta_base(OctreeGlobal& octree_global_arg, const int* codes_arg) : octree_global(octree_global_arg), codes(codes_arg) 
                {
                    //32 is a perfomance penalty step for search
                    Static<(max_points_per_leaf % 32) == 0>::check();                
                }

                __device__ __forceinline__ int FindCells(int task, int level, int cell_begs[], char cell_code[])
                {
                    int cell_count = 0;

                    int beg = octree_global.begs[task];
                    int end = octree_global.ends[task];

                    if (end - beg < max_points_per_leaf)
                    {                        
                        //cell_count == 0;
                    }
                    else
                    {
                        int cur_code = Morton::extractLevelCode(codes[beg], level);

                        cell_begs[cell_count] = beg;
                        cell_code[cell_count] = cur_code;     
                        ++cell_count;                        

                        int last_code = Morton::extractLevelCode(codes[end - 1], level);
                        if (last_code == cur_code)
                        {
                            cell_begs[cell_count] = end;                                         
                        }
                        else
                        {
                            for(;;)
                            {
                                int search_code = cur_code + 1;
                                if (search_code == 8)
                                {
                                    cell_begs[cell_count] = end;
                                    break;
                                }

                                int morton_code = Morton::shiftLevelCode(search_code, level);
                                int pos = lower_bound(codes + beg, codes + end, morton_code, CompareByLevelCode(level)) - codes; 

                                if (pos == end)
                                {
                                    cell_begs[cell_count] = end;
                                    break;
                                }
                                cur_code = Morton::extractLevelCode(codes[pos], level);

                                cell_begs[cell_count] = pos;
                                cell_code[cell_count] = cur_code;
                                ++cell_count;
                                beg = pos;
                            }        
                        }
                    }
                    return cell_count;
                }
            };
        }
    }
} 

#endif /* PCL_GPU_OCTREE_STA_BASE_H_ */