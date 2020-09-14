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
#include <fstream>
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

//TEST(PCL_OctreeGPU, DISABLED_batchRadiusSearch)
TEST(PCL_OctreeGPU, batchRadiusSearch)
{   
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/30.f;
    data.shared_radius = data.cube_size/30.f;
    data.printParams();

    const int max_answers = 333;

    //generate
    data();
        
    //prepare gpu cloud

    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);

    //gpu build 
    pcl::gpu::Octree octree_device;                
    octree_device.setCloud(cloud_device);	    
    octree_device.build();

    //upload queries
    pcl::gpu::Octree::Queries queries_device;
    pcl::gpu::Octree::Radiuses radiuses_device;
    queries_device.upload(data.queries);                
    radiuses_device.upload(data.radiuses);
    
    //prepare output buffers on device

    pcl::gpu::NeighborIndices result_device1(queries_device.size(), max_answers);
    pcl::gpu::NeighborIndices result_device2(queries_device.size(), max_answers);
    pcl::gpu::NeighborIndices result_device3(data.indices.size(), max_answers);
            
    //prepare output buffers on host
    std::vector< std::vector<int> > host_search1(data.tests_num);
    std::vector< std::vector<int> > host_search2(data.tests_num);
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {
        host_search1[i].reserve(max_answers);
        host_search2[i].reserve(max_answers);
    }    
    
    //search GPU shared
    octree_device.radiusSearch(queries_device, data.shared_radius, max_answers, result_device1);

    //search GPU individual
    octree_device.radiusSearch(queries_device,    radiuses_device, max_answers, result_device2);

    //search GPU shared with indices
    pcl::gpu::Octree::Indices indices;
    indices.upload(data.indices);
    octree_device.radiusSearch(queries_device, indices, data.shared_radius, max_answers, result_device3);

    //search CPU
    octree_device.internalDownload();
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {
        octree_device.radiusSearchHost(data.queries[i], data.shared_radius, host_search1[i], max_answers);
        octree_device.radiusSearchHost(data.queries[i], data.radiuses[i],   host_search2[i], max_answers);
    }
    
    //download results
    std::vector<int> sizes1;
    std::vector<int> sizes2;
    std::vector<int> sizes3;
    result_device1.sizes.download(sizes1);
    result_device2.sizes.download(sizes2);
    result_device3.sizes.download(sizes3);

    std::vector<int> downloaded_buffer1, downloaded_buffer2, downloaded_buffer3, results_batch;    
    result_device1.data.download(downloaded_buffer1);
    result_device2.data.download(downloaded_buffer2);
    result_device3.data.download(downloaded_buffer3);
        
    //data.bruteForceSearch();

    //verify results    
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {        
        std::vector<int>& results_host = host_search1[i];        
        
        int beg = i * max_answers;
        int end = beg + sizes1[i];

        results_batch.assign(downloaded_buffer1.begin() + beg, downloaded_buffer1.begin() + end);

        std::sort(results_batch.begin(), results_batch.end());
        std::sort(results_host.begin(), results_host.end());

        if ((int)results_batch.size() == max_answers && results_batch.size() < results_host.size() && max_answers)
            results_host.resize(max_answers);
        
        ASSERT_EQ ( ( results_batch == results_host ), true );       
       
        //vector<int>& results_bf = data.bfresutls[i];
        //ASSERT_EQ ( ( results_bf == results_batch), true );        
        //ASSERT_EQ ( ( results_bf == results_host ), true );           
    }    

    float avg_size1 = std::accumulate(sizes1.begin(), sizes1.end(), 0) * (1.f/sizes1.size());

    std::cout << "avg_result_size1 = " << avg_size1 << std::endl;
    ASSERT_GT(avg_size1, 5);    


    //verify results    
    for(std::size_t i = 0; i < data.tests_num; ++i)
    {        
        std::vector<int>& results_host = host_search2[i];        
        
        int beg = i * max_answers;
        int end = beg + sizes2[i];

        results_batch.assign(downloaded_buffer2.begin() + beg, downloaded_buffer2.begin() + end);

        std::sort(results_batch.begin(), results_batch.end());
        std::sort(results_host.begin(), results_host.end());

        if ((int)results_batch.size() == max_answers && results_batch.size() < results_host.size() && max_answers)
            results_host.resize(max_answers);

        ASSERT_EQ ( ( results_batch == results_host ), true );       
       
        //vector<int>& results_bf = data.bfresutls[i];
        //ASSERT_EQ ( ( results_bf == results_batch), true );        
        //ASSERT_EQ ( ( results_bf == results_host ), true );           
    }    

    float avg_size2 = std::accumulate(sizes2.begin(), sizes2.end(), 0) * (1.f/sizes2.size());

    std::cout << "avg_result_size2 = " << avg_size2 << std::endl;
    ASSERT_GT(avg_size2, 5);


    //verify results    
    for(std::size_t i = 0; i < data.tests_num; i+=2)
    {                
        std::vector<int>& results_host = host_search1[i];        
        
        int beg = i/2 * max_answers;
        int end = beg + sizes3[i/2];

        results_batch.assign(downloaded_buffer3.begin() + beg, downloaded_buffer3.begin() + end);

        std::sort(results_batch.begin(), results_batch.end());
        std::sort(results_host.begin(), results_host.end());

        if ((int)results_batch.size() == max_answers && results_batch.size() < results_host.size() && max_answers)
            results_host.resize(max_answers);
        
        ASSERT_EQ ( ( results_batch == results_host ), true );       
       
        //vector<int>& results_bf = data.bfresutls[i];
        //ASSERT_EQ ( ( results_bf == results_batch), true );        
        //ASSERT_EQ ( ( results_bf == results_host ), true );           
    }

    float avg_size3 = std::accumulate(sizes3.begin(), sizes3.end(), 0) * (1.f/sizes3.size());

    std::cout << "avg_result_size3 = " << avg_size3 << std::endl;
    ASSERT_GT(avg_size3, 5);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

