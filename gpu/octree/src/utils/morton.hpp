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

#ifndef PCL_GPU_OCTREE_MORTON_HPP
#define PCL_GPU_OCTREE_MORTON_HPP


namespace pcl
{
    namespace device
    {
        struct Morton
        {   
            const static int levels = 10;
            const static int bits_per_level = 3;
            const static int nbits = levels * bits_per_level;    

            using code_t = int;

            __device__ __host__ __forceinline__ 
                static int spreadBits(int x, int offset)
            {
                //......................9876543210
                x = (x | (x << 10)) & 0x000f801f; //............98765..........43210
                x = (x | (x <<  4)) & 0x00e181c3; //........987....56......432....10
                x = (x | (x <<  2)) & 0x03248649; //......98..7..5..6....43..2..1..0
                x = (x | (x <<  2)) & 0x09249249; //....9..8..7..5..6..4..3..2..1..0

                return x << offset;
            }

            __device__ __host__ __forceinline__ 
                static int compactBits(int x, int offset)
            {                                      
                x = ( x >> offset ) & 0x09249249;  //....9..8..7..5..6..4..3..2..1..0
                x = (x | (x >>  2)) & 0x03248649;  //......98..7..5..6....43..2..1..0                                          
                x = (x | (x >>  2)) & 0x00e181c3;  //........987....56......432....10                                       
                x = (x | (x >>  4)) & 0x000f801f;  //............98765..........43210                                          
                x = (x | (x >> 10)) & 0x000003FF;  //......................9876543210        

                return x;
            }

            __device__ __host__ __forceinline__
                static code_t createCode(int cell_x, int cell_y, int cell_z)
            { 
                return spreadBits(cell_x, 0) | spreadBits(cell_y, 1) | spreadBits(cell_z, 2); 
            }

            __device__ __host__ __forceinline__
                static void decomposeCode(code_t code, int& cell_x, int& cell_y, int& cell_z)
            { 
                cell_x = compactBits(code, 0);
                cell_y = compactBits(code, 1);
                cell_z = compactBits(code, 2);        
            }

            __device__ __host__ __forceinline__
                static uint3 decomposeCode(code_t code)
            {
                return make_uint3 (compactBits(code, 0), compactBits(code, 1), compactBits(code, 2));
            }

            __host__ __device__ __forceinline__ 
                static code_t extractLevelCode(code_t code, int level) 
            {
                return (code >> (nbits - 3 * (level + 1) )) & 7; 
            }

            __host__ __device__ __forceinline__
                static code_t shiftLevelCode(code_t level_code, int level)
            {
                return level_code << (nbits - 3 * (level + 1));
            }
        };

        struct CalcMorton
        {   
            const static int depth_mult = 1 << Morton::levels;

            float3 minp_;
            float3 dims_;    

            __device__ __host__ __forceinline__ CalcMorton(float3 minp, float3 maxp) : minp_(minp) 
            {        
                dims_.x = maxp.x - minp.x;
                dims_.y = maxp.y - minp.y;
                dims_.z = maxp.z - minp.z;        
            }			

            __device__ __host__ __forceinline__ Morton::code_t operator()(const float3& p) const
            {
                //Using floorf due to changes to MSVC 16.9. See details here: https://devtalk.blender.org/t/cuda-compile-error-windows-10/17886/4
                //floorf is without std:: see why here: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79700
                const int cellx = min((int)floorf(depth_mult * min(1.f, max(0.f, (p.x - minp_.x)/dims_.x))), depth_mult - 1);
                const int celly = min((int)floorf(depth_mult * min(1.f, max(0.f, (p.y - minp_.y)/dims_.y))), depth_mult - 1);
                const int cellz = min((int)floorf(depth_mult * min(1.f, max(0.f, (p.z - minp_.z)/dims_.z))), depth_mult - 1);

                return Morton::createCode(cellx, celly, cellz);
            }	
             __device__ __host__ __forceinline__ Morton::code_t operator()(const float4& p) const
            {			
                return (*this)(make_float3(p.x, p.y, p.z));                
            }	
        };

        struct CompareByLevelCode
        {
            int level;

            __device__ __host__ __forceinline__ 
                CompareByLevelCode(int level_arg) : level(level_arg) {}    

            __device__ __host__ __forceinline__
                bool operator()(Morton::code_t code1, Morton::code_t code2) const 
            {                  
                return Morton::extractLevelCode(code1, level) < Morton::extractLevelCode(code2, level);  
            }	
        };
    }
}

#endif /* PCL_GPU_OCTREE_MORTON_HPP */
