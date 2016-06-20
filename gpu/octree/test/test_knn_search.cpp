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
#include <pcl/common/time.h>
#include "data_source.hpp"

using namespace pcl::gpu;
using namespace std;


struct PriorityPair
{    
    int index;
    float dist2;

    bool operator<(const PriorityPair& other) const { return dist2 < other.dist2; }

    bool operator==(const PriorityPair& other) const { return dist2 == other.dist2 && index == other.index; }
};

//TEST(PCL_OctreeGPU, DISABLED_exactNeighbourSearch)
TEST(PCL_OctreeGPU, exactNeighbourSearch)
{       
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;    
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/30.f;
    data.shared_radius = data.cube_size/30.f;
    data.printParams();

    const float host_octree_resolution = 25.f;
    const int k = 1; // only this is supported

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
    pcl::gpu::NeighborIndices result_device(data.tests_num, k);    

    //prepare output buffers on host
    vector<vector<  int> > result_host(data.tests_num);   
    vector<vector<float> >  dists_host(data.tests_num);    
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        result_host[i].reserve(k);
        dists_host[i].reserve(k);
    }
        
    //search GPU shared
    {
        pcl::ScopeTime time("1nn-gpu");
        octree_device.nearestKSearchBatch(queries_device, k, result_device);
    }

    vector<int> downloaded, downloaded_cur;
    result_device.data.download(downloaded);
                 
    {
        pcl::ScopeTime time("1nn-cpu");
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_host.nearestKSearch(data.queries[i], k, result_host[i], dists_host[i]);
    }

    //verify results    
    for(size_t i = 0; i < data.tests_num; ++i)    
    {           
        //cout << i << endl;
        vector<int>&   results_host_cur = result_host[i];
        vector<float>&   dists_host_cur = dists_host[i];
                
        int beg = i * k;
        int end = beg + k;

        downloaded_cur.assign(downloaded.begin() + beg, downloaded.begin() + end);
        
        vector<PriorityPair> pairs_host;
        vector<PriorityPair> pairs_gpu;
        for(int n = 0; n < k; ++n)
        {
            PriorityPair host;
            host.index = results_host_cur[n];
            host.dist2 = dists_host_cur[n];
            pairs_host.push_back(host);

            PriorityPair gpu;
            gpu.index = downloaded_cur[n];

            float dist = (data.queries[i].getVector3fMap() - data.points[gpu.index].getVector3fMap()).norm();
            gpu.dist2 = dist * dist;
            pairs_gpu.push_back(gpu);
        }
        
        std::sort(pairs_host.begin(),  pairs_host.end());
        std::sort(pairs_gpu.begin(), pairs_gpu.end());    

        while (pairs_host.size ())
        {
            ASSERT_EQ ( pairs_host.back().index , pairs_gpu.back().index );
            EXPECT_NEAR ( pairs_host.back().dist2 , pairs_gpu.back().dist2, 1e-2 );
            
            pairs_host.pop_back();
            pairs_gpu.pop_back();
        }             
    }     
}
