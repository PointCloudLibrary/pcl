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

#include <gtest/gtest.h>

#include <iostream>
#include <numeric>

#if defined _MSC_VER
    #pragma warning (disable: 4521)
#endif

#include <pcl/point_cloud.h>

#if defined _MSC_VER
    #pragma warning (default: 4521)
#endif

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>

#include "data_source.hpp"

using namespace pcl::gpu;


//TEST (PCL_GPU, DISABLED_bruteForceRadiusSeachGPU)
TEST (PCL_GPU, bruteForceRadiusSeachGPU)
{   
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 100;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/15.f;
    data.shared_radius = data.cube_size/20.f;
    data.printParams();  

    //generate
    data();
    
    // brute force radius search
    data.bruteForceSearch();

    //prepare gpu cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);
    
    pcl::gpu::DeviceArray<int> results_device, buffer(cloud_device.size());
    
    std::vector<int> results_host;
    std::vector<std::size_t> sizes;
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {
        pcl::gpu::bruteForceRadiusSearchGPU(cloud_device, data.queries[i], data.radiuses[i], results_device, buffer);

        results_device.download(results_host);
        std::sort(results_host.begin(), results_host.end());

        ASSERT_EQ ( (results_host == data.bfresutls[i]), true );
        sizes.push_back(results_device.size());      
    }
        
    float avg_size = std::accumulate(sizes.begin(), sizes.end(), (std::size_t)0) * (1.f/sizes.size());;

    std::cout << "avg_result_size = " << avg_size << std::endl;
    ASSERT_GT(avg_size, 5);    
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

