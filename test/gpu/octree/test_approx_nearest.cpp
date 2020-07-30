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

#include <iostream>
#include <fstream>
#include <algorithm>

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

using namespace pcl::gpu;

//TEST(PCL_OctreeGPU, DISABLED_approxNearesSearch)
TEST(PCL_OctreeGPU, approxNearesSearch)
{
    //generate custom pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in cloud data
    cloud->width    = 10000;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);


    float x_cords[cloud->size()] = {-1, -1, -1, 1, -1, 1, 1, 1, -0.9, -0.4};
    float y_cords[cloud->size()] = {-1, -1, 1, -1, 1, -1, 1, 1, -0.2, -0.6};
    float z_cords[cloud->size()] = {-1, 1, -1, -1, 1, 1, -1, 1, -0.75, -0.75};

    for (pcl::index_t i = 0; i < cloud->size (); ++i)
    {
        (*cloud)[i].x = x_cords[i%10];
        (*cloud)[i].y = y_cords[i%10];
        (*cloud)[i].z = z_cords[i%10];
    }

    std::vector<pcl::PointXYZ> queries;
    //While the GPU implementation has a fixed depth of 10 levels, octree depth in the CPU implementation can vary based on
    //the leaf size set by the user, which can affect the results. Therefore results would only tally if depths match.
    queries.push_back(pcl::PointXYZ(-0.4, -0.2, -0.75));     //should be different across CPU and GPU if different traversal methods are used
    queries.push_back(pcl::PointXYZ(-0.6, -0.2, -0.75));     //should be same across CPU and GPU
    queries.push_back(pcl::PointXYZ(1.1, 1.1, 1.1));         //out of range query

<<<<<<< HEAD:test/gpu/octree/test_approx_nearest.cpp
    //prepare host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host(new pcl::PointCloud<pcl::PointXYZ>);	
    cloud_host->width = data.points.size();
    cloud_host->height = 1;
    cloud_host->points.resize (cloud_host->width * cloud_host->height);    
    std::transform(data.points.begin(), data.points.end(), cloud_host->points.begin(), DataGenerator::ConvPoint<pcl::PointXYZ>());
=======
    //prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud->points);
>>>>>>> add new traversal for  gpuApproxNearestSearch & modify tests:gpu/octree/test/test_approx_nearest.cpp

    //gpu build
    pcl::gpu::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();

    //build host octree
    float host_octree_resolution = 0.05;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(host_octree_resolution);
    octree_host.setInputCloud (cloud);
    octree_host.addPointsFromInputCloud();

    //upload queries
    pcl::gpu::Octree::Queries queries_device;
    queries_device.upload(queries);
    //pcl::gpu::Octree::ResultSqrDists distances_device(queries.size());

    //prepare output buffers on device
    pcl::gpu::NeighborIndices result_device(queries.size(), 1);
    std::vector<int> result_host_pcl(queries.size());
    std::vector<int> result_host_gpu(queries.size());
    std::vector<float> dists_pcl(queries.size());
    std::vector<float> dists_gpu(queries.size());

    //search GPU shared
    octree_device.approxNearestSearch(queries_device, result_device);
    std::vector<int> downloaded;
    std::vector<float>distances;
    result_device.data.download(downloaded);
    //distances_device.download(dists_gpu_direct);

    for(size_t i = 0; i < queries.size(); ++i)
    {
        octree_host.approxNearestSearch(queries[i], result_host_pcl[i], dists_pcl[i]);
        octree_device.approxNearestSearchHost(queries[i], result_host_gpu[i], dists_gpu[i]);
    }

    ASSERT_EQ ((downloaded == result_host_gpu), true);

    //find inconsistencies with gpu and cpu cuda impementation
    int count_gpu_better = 0;
    int count_pcl_better = 0;
    int count_different = 0;
    for(size_t i = 0; i < queries.size(); ++i)
    {
        ASSERT_EQ ((dists_pcl[i] == dists_gpu[i]), true);
    }

}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

