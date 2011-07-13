#pragma once

#include "octree_global.hpp"
#include "utils/lower_bound.hpp"
#include "utils/morton.hpp"

namespace base
{
    struct Cta_base
    {        
        OctreeGlobal& octree_global;
        const int* codes;
        const static int max_points_per_leaf = 10;

        __device__ __forceinline__ Cta_base(OctreeGlobal& octree_global_arg, const int* codes_arg) : octree_global(octree_global_arg), codes(codes_arg) {}


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