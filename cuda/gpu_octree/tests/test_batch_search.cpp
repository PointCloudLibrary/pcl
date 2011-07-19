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

#pragma warning (disable: 4521)
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#pragma warning (default: 4521)

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/device_array.hpp"
#include "pcl/gpu/common/timers_opencv.hpp"

#include "data_gen.hpp"

using namespace std;
using namespace pcl::gpu;

//TEST(PCL_OctreeGPU, DISABLED_batchRadiusSearch)
TEST(PCL_OctreeGPU, batchRadiusSearch)
{   
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/15.f;
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
    pcl::gpu::Octree::BatchQueries queries_device;
    queries_device.upload(data.queries);                
    
    //prepare output buffers on device
    pcl::gpu::Octree::BatchResult      result_device(queries_device.size() * max_answers);
    pcl::gpu::Octree::BatchResultSizes  sizes_device(queries_device.size());
    
    //prepare output buffers on host
    vector< vector<int> > host_search(data.tests_num);
    for(size_t i = 0; i < data.tests_num; ++i)
        host_search[i].reserve(max_answers);
    
    //search GPU
    octree_device.radiusSearchBatchGPU(queries_device, data.shared_radius, max_answers, result_device, sizes_device);

    //search CPU
    octree_device.internalDownload();
    for(size_t i = 0; i < data.tests_num; ++i)
        octree_device.radiusSearchHost(data.queries[i], data.shared_radius, host_search[i], max_answers);
    
    //download results
    vector<int> sizes;
    sizes_device.download(sizes);

    vector<int> downloaded_buffer, results_batch;    
    result_device.download(downloaded_buffer);
        
    //data.bruteForceSearch();

    //verify results    
    for(size_t i = 0; i < data.tests_num; ++i)
    {        
        vector<int>& results_host = host_search[i];        
        
        int beg = i * max_answers;
        int end = beg + sizes[i];

        results_batch.assign(downloaded_buffer.begin() + beg, downloaded_buffer.begin() + end);

        std::sort(results_batch.begin(), results_batch.end());
        std::sort(results_host.begin(), results_host.end());

        if (results_batch.size() == max_answers && results_batch.size() < results_host.size() && max_answers)
            results_host.resize(max_answers);
        
        ASSERT_EQ ( ( results_batch == results_host ), true );       
       
        //vector<int>& results_bf = data.bfresutls[i];
        //ASSERT_EQ ( ( results_bf == results_batch), true );        
        //ASSERT_EQ ( ( results_bf == results_host ), true );           
    }    

    float avg_size = std::accumulate(sizes.begin(), sizes.end(), 0) * (1.f/sizes.size());;

    cout << "avg_result_size = " << avg_size << endl;
    ASSERT_GT(avg_size, 5);    
}
