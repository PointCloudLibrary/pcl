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
#include<algorithm>

#if defined _MSC_VER
    #pragma warning (disable: 4521)
#endif
    
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#if defined _MSC_VER
    #pragma warning (default: 4521)
#endif
    

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/common/time.h>
#include "data_source.hpp"

using namespace pcl::gpu;
using namespace std;

using pcl::ScopeTime;

#if defined HAVE_OPENCV
    #include "opencv2/contrib/contrib.hpp"
#endif

//TEST(PCL_OctreeGPU, DISABLED_perfomance)
TEST(PCL_OctreeGPU, perfomance)
{
    DataGenerator data;
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius    = data.cube_size/15.f;
    data.shared_radius = data.cube_size/15.f;
    data.printParams();

    //const int k = 32;

    cout << "sizeof(pcl::gpu::Octree::PointType): " << sizeof(pcl::gpu::Octree::PointType) << endl;    
    //cout << "k = " << k << endl;
    //generate data
    data();

    //prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);

    //prepare queries_device
    pcl::gpu::Octree::Queries queries_device;
    pcl::gpu::Octree::Radiuses radiuses_device;
     queries_device.upload(data.queries);  
    radiuses_device.upload(data.radiuses);

    //prepare host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host(new pcl::PointCloud<pcl::PointXYZ>);	
    cloud_host->width = data.points.size();
    cloud_host->height = 1;
    cloud_host->points.resize (cloud_host->width * cloud_host->height);    
    std::transform(data.points.begin(), data.points.end(), cloud_host->points.begin(), DataGenerator::ConvPoint<pcl::PointXYZ>());

    float host_octree_resolution = 25.f;    
    
    cout << "[!] Host octree resolution: " << host_octree_resolution << endl << endl;    

    cout << "======  Build perfomance =====" << endl;
    // build device octree
    pcl::gpu::Octree octree_device;                
    octree_device.setCloud(cloud_device);	    
    {        
        ScopeTime up("gpu-build");	                
        octree_device.build();
    }    
    {
        ScopeTime up("gpu-download");	
        octree_device.internalDownload();
    }

    //build host octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(host_octree_resolution);
    octree_host.setInputCloud (cloud_host);
    {
        ScopeTime t("host-build");	        
        octree_host.addPointsFromInputCloud();
    }

    // build opencv octree
#ifdef HAVE_OPENCV
    cv::Octree octree_opencv;
    const static int opencv_octree_points_per_leaf = 32;    
    vector<cv::Point3f> opencv_points(data.points.size());
    std::transform(data.points.begin(), data.points.end(), opencv_points.begin(), DataGenerator::ConvPoint<cv::Point3f>());
        
    {        
        ScopeTime t("opencv-build");	        
        octree_opencv.buildTree(opencv_points, 10, opencv_octree_points_per_leaf); 
    }
#endif
    
    //// Radius search perfomance ///

    const int max_answers = 500;
    float dist;
    int inds;

    //host buffers
    vector<int> indeces;
    vector<float> pointRadiusSquaredDistance;
#ifdef HAVE_OPENCV  
    vector<cv::Point3f> opencv_results;
#endif

    //reserve
    indeces.reserve(data.data_size);
    pointRadiusSquaredDistance.reserve(data.data_size);
#ifdef HAVE_OPENCV
    opencv_results.reserve(data.data_size);
#endif

    //device buffers
    pcl::gpu::DeviceArray<int> bruteforce_results_device, buffer(cloud_device.size());    
    pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);
    
    //pcl::gpu::Octree::BatchResult          distsKNN_device(queries_device.size() * k);
    //pcl::gpu::Octree::BatchResultSqrDists  indsKNN_device(queries_device.size() * k);
        
    cout << "======  Separate radius for each query =====" << endl;

    {
        ScopeTime up("gpu--radius-search-batch-all");	        
        octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);
    }

    {
        ScopeTime up("gpu-radius-search-{host}-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_device.radiusSearchHost(data.queries[i], data.radiuses[i], indeces, max_answers);                        
    }

    {                
        ScopeTime up("host-radius-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_host.radiusSearch(pcl::PointXYZ(data.queries[i].x, data.queries[i].y, data.queries[i].z), 
                data.radiuses[i], indeces, pointRadiusSquaredDistance, max_answers);                        
    }
     
    {
        ScopeTime up("gpu_bruteforce-radius-search-all");	         
        for(size_t i = 0; i < data.tests_num; ++i)
            pcl::gpu::bruteForceRadiusSearchGPU(cloud_device, data.queries[i], data.radiuses[i], bruteforce_results_device, buffer);
    }

    cout << "======  Shared radius (" << data.shared_radius << ") =====" << endl;
    
    {
        ScopeTime up("gpu-radius-search-batch-all");	        
        octree_device.radiusSearch(queries_device, data.shared_radius, max_answers, result_device);                        
    }

    {
        ScopeTime up("gpu-radius-search-{host}-all");
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_device.radiusSearchHost(data.queries[i], data.shared_radius, indeces, max_answers);                        
    }

    {                
        ScopeTime up("host-radius-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_host.radiusSearch(pcl::PointXYZ(data.queries[i].x, data.queries[i].y, data.queries[i].z), 
                data.radiuses[i], indeces, pointRadiusSquaredDistance, max_answers);                        
    }
     
    {
        ScopeTime up("gpu-radius-bruteforce-search-all");	         
        for(size_t i = 0; i < data.tests_num; ++i)
            pcl::gpu::bruteForceRadiusSearchGPU(cloud_device, data.queries[i], data.shared_radius, bruteforce_results_device, buffer);
    }

    cout << "======  Approx nearest search =====" << endl;

    {
        ScopeTime up("gpu-approx-nearest-batch-all");	        
        octree_device.approxNearestSearch(queries_device, result_device);                        
    }

    {        
        ScopeTime up("gpu-approx-nearest-search-{host}-all");
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_device.approxNearestSearchHost(data.queries[i], inds, dist);                        
    }

    {                
        ScopeTime up("host-approx-nearest-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_host.approxNearestSearch(data.queries[i], inds, dist);
    }

 /*   cout << "======  knn search ( k fixed to " << k << " ) =====" << endl;    
    {
        ScopeTime up("gpu-knn-batch-all");	        
        octree_device.nearestKSearchBatch(queries_device, k, distsKNN_device, indsKNN_device);                        
    }    

    {                
        ScopeTime up("host-knn-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            octree_host.nearestKSearch(data.queries[i], k, indeces, pointRadiusSquaredDistance);
    }*/
}
