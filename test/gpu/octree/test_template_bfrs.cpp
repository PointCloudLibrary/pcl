/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
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


TEST (PCL_GPU, pclXYZ)
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
    pcl::gpu::OcTree<pcl::PointXYZ>::PointCloud cloud_device;
    cloud_device.upload(data.points);
    
    pcl::gpu::DeviceArray<int> results_device, buffer(cloud_device.size());
    
    std::vector<int> results_host;
    std::vector<std::size_t> sizes;
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {
        pcl::gpu::BruteForceRadiusSearchGPU<pcl::PointXYZ>(cloud_device, data.queries[i], data.radiuses[i], results_device, buffer);

        results_device.download(results_host);
        std::sort(results_host.begin(), results_host.end());

        ASSERT_EQ ( (results_host == data.bfresutls[i]), true );
        sizes.push_back(results_device.size());      
    }
        
    float avg_size = std::accumulate(sizes.begin(), sizes.end(), (std::size_t)0) * (1.f/sizes.size());;

    std::cout << "avg_result_size = " << avg_size << std::endl;
    ASSERT_GT(avg_size, 5);    
}

TEST (PCL_GPU, pclXYZRGB)
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
    std::vector<pcl::PointXYZRGB> tmp_storage;
    for (const auto& point :  data.points) {
        pcl::PointXYZRGB tmp{point.x, point.y, point.z, 0, 0, 1};
        tmp_storage.push_back(tmp);
    }

    //prepare gpu cloud
    pcl::gpu::OcTree<pcl::PointXYZRGB>::PointCloud cloud_device;
    cloud_device.upload(tmp_storage);
    
    pcl::gpu::DeviceArray<int> results_device, buffer(cloud_device.size());
    
    std::vector<int> results_host;
    std::vector<std::size_t> sizes;
    for (std::size_t i = 0; i < data.tests_num; ++i) {
      pcl::PointXYZRGB query{
          data.queries[i].x, data.queries[i].y, data.queries[i].z, 0, 0, 1};
      pcl::gpu::BruteForceRadiusSearchGPU<pcl::PointXYZRGB>(
          cloud_device, query, data.radiuses[i], results_device, buffer);

      results_device.download(results_host);
      std::sort(results_host.begin(), results_host.end());

      ASSERT_EQ((results_host == data.bfresutls[i]), true);
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

