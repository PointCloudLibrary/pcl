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

#ifndef __PCL_GPU_UTILS_REPACKS_HPP__
#define __PCL_GPU_UTILS_REPACKS_HPP__

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
    namespace gpu
    {
        ///////////////////////////////////////
        ///  This is an experimental code   ///
        ///////////////////////////////////////

        const int NoCP = 0xFFFFFFFF;

        /** \brief Returns field copy operation code. */
        inline int cp(int from, int to) 
        { 
            return ((to & 0xF) << 4) + (from & 0xF); 
        }

        /* Combines several field copy operations to one int (called rule) */
        inline int rule(int cp1, int cp2 = NoCP, int cp3 = NoCP, int cp4 = NoCP)
        {
            return (cp1 & 0xFF) + ((cp2 & 0xFF) << 8) + ((cp3 & 0xFF) << 16) + ((cp4 & 0xFF) << 24);
        }

        /* Combines performs all field copy operations in given rule array (can be 0, 1, or 16 copies) */
        void copyFieldsImpl(int in_size, int out_size, int rules[4], int size, const void* input, void* output); 

        template<typename PointIn, typename PointOut>
        void copyFieldsEx(const DeviceArray<PointIn>& src, DeviceArray<PointOut>& dst, int rule1, int rule2 = NoCP, int rule3 = NoCP, int rule4 = NoCP)
        {
            int rules[4] = { rule1, rule2, rule3, rule4 };
            dst.create(src.size());
            copyFieldsImpl(sizeof(PointIn)/sizeof(int), sizeof(PointOut)/sizeof(int), rules, (int)src.size(), src.ptr(), dst.ptr());
        }

        void copyFields(const DeviceArray<PointXYZ>& src, DeviceArray<PointNormal>& dst)
        {
            //PointXYZ.x (0) -> PointNormal.x (0)
            //PointXYZ.y (1) -> PointNormal.y (1)
            //PointXYZ.z (2) -> PointNormal.z (2)
            copyFieldsEx(src, dst, rule(cp(0, 0), cp(1, 1), cp(2, 2)));
        };

        void copyFields(const DeviceArray<Normal>& src, DeviceArray<PointNormal>& dst)
        {
            //PointXYZ.normal_x (0)  -> PointNormal.normal_x (4)
            //PointXYZ.normal_y (1)  -> PointNormal.normal_y (5)
            //PointXYZ.normal_z (2)  -> PointNormal.normal_z (6)
            //PointXYZ.curvature (4) -> PointNormal.curvature (8)
            copyFieldsEx(src, dst, rule(cp(0, 4), cp(1, 5), cp(2, 6), cp(4,8)));
        };

        void copyFields(const DeviceArray<PointXYZRGBL>& src, DeviceArray<PointXYZ>& dst)
        {
            //PointXYZRGBL.x (0) -> PointXYZ.x (0)
            //PointXYZRGBL.y (1) -> PointXYZ.y (1)
            //PointXYZRGBL.z (2) -> PointXYZ.z (2)
            copyFieldsEx(src, dst, rule(cp(0, 0), cp(1, 1), cp(2, 2)));
        };

        void copyFields(const DeviceArray<PointXYZRGB>& src, DeviceArray<PointXYZ>& dst)
        {
            //PointXYZRGB.x (0) -> PointXYZ.x (0)
            //PointXYZRGB.y (1) -> PointXYZ.y (1)
            //PointXYZRGB.z (2) -> PointXYZ.z (2)
            copyFieldsEx(src, dst, rule(cp(0, 0), cp(1, 1), cp(2, 2)));
        };

        void copyFields(const DeviceArray<PointXYZRGBA>& src, DeviceArray<PointXYZ>& dst)
        {
            //PointXYZRGBA.x (0) -> PointXYZ.x (0)
            //PointXYZRGBA.y (1) -> PointXYZ.y (1)
            //PointXYZRGBA.z (2) -> PointXYZ.z (2)
            copyFieldsEx(src, dst, rule(cp(0, 0), cp(1, 1), cp(2, 2)));
        };

        void copyFieldsZ(const DeviceArray<PointXYZ>& src, DeviceArray<float>& dst)
        {
            //PointXYZRGBL.z (2) -> float (1)
            copyFieldsEx(src, dst, rule(cp(2, 0)));
        };

        void copyFieldsZ(const DeviceArray<PointXYZRGB>& src, DeviceArray<float>& dst)
        {
            //PointXYZRGBL.z (2) -> float (1)
            copyFieldsEx(src, dst, rule(cp(2, 0)));
        };
    }
}

#endif /* __PCL_GPU_UTILS_REPACKS_HPP__ */
