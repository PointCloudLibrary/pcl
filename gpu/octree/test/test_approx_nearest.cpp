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

#if defined _MSC_VER
    #pragma warning (disable : 4996 4530)
#endif

#include <gtest/gtest.h>

#include<iostream>
#include<fstream>
#include<algorithm>

#if defined _MSC_VER
    #pragma warning (disable: 4521)
#endif
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#if defined _MSC_VER
    #pragma warning (default: 4521)
#endif

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>

#include "data_source.hpp"

using namespace pcl::gpu;
using namespace std;

//TEST(PCL_OctreeGPU, DISABLED_approxNearesSearch)
TEST(PCL_OctreeGPU, approxNearesSearch)
{   
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/30.f;
    data.shared_radius = data.cube_size/30.f;
    data.printParams();

    const float host_octree_resolution = 25.f;

    //generate
    data();
        
    //prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);


    //prepare host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host(new pcl::PointCloud<pcl::PointXYZ>);	
    cloud_host->width = data.points.size();
    cloud_host->height = 1;
    cloud_host->points.resize (cloud_host->width * cloud_host->height);    
    std::transform(data.points.begin(), data.points.end(), cloud_host->points.begin(), DataGenerator::ConvPoint<pcl::PointXYZ>());

    //gpu build 
    pcl::gpu::Octree octree_device;                
    octree_device.setCloud(cloud_device);	    
    octree_device.build();
    
    //build host octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(host_octree_resolution);
    octree_host.setInputCloud (cloud_host);    
    octree_host.addPointsFromInputCloud();
           
    //upload queries
    pcl::gpu::Octree::Queries queries_device;
    queries_device.upload(data.queries);
    
        
    //prepare output buffers on device
    pcl::gpu::NeighborIndices result_device(data.tests_num, 1);
    vector<int> result_host_pcl(data.tests_num);
    vector<int> result_host_gpu(data.tests_num);
    vector<float> dists_pcl(data.tests_num);
    vector<float> dists_gpu(data.tests_num);
    
    //search GPU shared
    octree_device.approxNearestSearch(queries_device, result_device);

    vector<int> downloaded;
    result_device.data.download(downloaded);
                
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        octree_host.approxNearestSearch(data.queries[i], result_host_pcl[i], dists_pcl[i]);
        octree_device.approxNearestSearchHost(data.queries[i], result_host_gpu[i], dists_gpu[i]);
    }

    ASSERT_EQ ( ( downloaded == result_host_gpu ), true );

    int count_gpu_better = 0;
    int count_pcl_better = 0;
    float diff_pcl_better = 0;
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        float diff = dists_pcl[i] - dists_gpu[i];
        bool gpu_better = diff > 0;

        ++(gpu_better ? count_gpu_better : count_pcl_better);

        if (!gpu_better)
            diff_pcl_better +=fabs(diff);
    }

    diff_pcl_better /=count_pcl_better;

    cout << "count_gpu_better: " << count_gpu_better << endl;
    cout << "count_pcl_better: " << count_pcl_better << endl;
    cout << "avg_diff_pcl_better: " << diff_pcl_better << endl;    

}
