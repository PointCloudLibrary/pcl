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

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"

#include "pcl/gpu/containers/device_array.hpp"

namespace pcl
{
    namespace gpu
    {
        struct PCL_EXPORTS Repack
        {            
            template<typename In, typename Out>
            static void invoke(const DeviceArray<In>& input, DeviceArray<Out>& ouput);

            template<typename In, typename Out>
            static void invoke(const DeviceArray<In>& input, DeviceArray2D<Out>& ouput);

            template<typename In, typename Out>
            static void invoke(const DeviceArray2D<In>& input, DeviceArray<Out>& ouput);            
        };

        template<>
        PCL_EXPORTS void Repack::invoke<pcl::PointXYZ, float>(const DeviceArray<pcl::PointXYZ>& input, DeviceArray2D<float>& ouput);

        template<>
        PCL_EXPORTS void Repack::invoke<float, pcl::PointXYZ>(const DeviceArray2D<float>& input, DeviceArray<pcl::PointXYZ>& ouput);
    }
}

#endif /* __PCL_GPU_UTILS_REPACKS_HPP__ */