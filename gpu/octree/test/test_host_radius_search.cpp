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

#include <numeric>
#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#if defined _MSC_VER
    #pragma warning (disable: 4521)
#endif
    
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#if defined _MSC_VER
    #pragma warning (default: 4521)
#endif

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/initialization.h>

#include "data_source.hpp"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_OctreeGPU, DISABLED_hostRadiusSearch)
TEST(PCL_OctreeGPU, hostRadiusSearch)
{
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/15.f;
    data.shared_radius = data.cube_size/20.f;
    data.printParams();

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
    std::transform(data.points.begin(), data.points.end(),  cloud_host->points.begin(), DataGenerator::ConvPoint<pcl::PointXYZ>());
    
    // build device octree
    pcl::gpu::Octree octree_device;                
    octree_device.setCloud(cloud_device);	    
    octree_device.build();



    // build host octree
    float resolution = 25.f;
    cout << "[!]Octree resolution: " << resolution << endl;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(resolution);
    octree_host.setInputCloud (cloud_host);
    octree_host.addPointsFromInputCloud ();

    //perform bruteForceSearch    
    data.bruteForceSearch(true);    
    
    vector<int> sizes;
    sizes.reserve(data.tests_num);
    octree_device.internalDownload();
             
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        //search host on octree tha was built on device
        vector<int> results_host_gpu; //host search
        octree_device.radiusSearchHost(data.queries[i], data.radiuses[i], results_host_gpu);                        
        
        //search host
        vector<float> dists;
        vector<int> results_host;                
        octree_host.radiusSearch(pcl::PointXYZ(data.queries[i].x, data.queries[i].y, data.queries[i].z), data.radiuses[i], results_host, dists);                        
        
        std::sort(results_host_gpu.begin(), results_host_gpu.end());
        std::sort(results_host.begin(), results_host.end());

        ASSERT_EQ ( (results_host_gpu == results_host     ), true );
        ASSERT_EQ ( (results_host_gpu == data.bfresutls[i]), true );                
        sizes.push_back(results_host.size());      
    }    

    float avg_size = std::accumulate(sizes.begin(), sizes.end(), 0) * (1.f/sizes.size());;

    cout << "avg_result_size = " << avg_size << endl;
    ASSERT_GT(avg_size, 5);    
}


int main (int argc, char** argv)
{
    const int device = 0;
    pcl::gpu::setDevice(device);
    pcl::gpu::printShortCudaDeviceInfo(device);        
    testing::InitGoogleTest (&argc, argv);
    return (RUN_ALL_TESTS ());
}
