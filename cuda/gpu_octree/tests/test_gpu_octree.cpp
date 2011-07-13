#include <numeric>
#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#pragma warning (disable: 4521)
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#pragma warning (default: 4521)

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/device_array.hpp"

#include "data_gen.hpp"

using namespace std;
using namespace pcl;

//TEST (PCL, DISABLED_OctreeGpuHostRadiusSearchVsBF)
TEST (PCL, OctreeGpuHostRadiusSearchVsBF)
{
    size_t data_size = 871000;
    size_t tests_num = 10000;
    float cube_size = 1024.f;
    float max_radius_part = 15.f;

    DataGenerator data(data_size, tests_num, cube_size, max_radius_part);
    data.printParams();
      
    data.bruteForceSearch();

    //my-build
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);

    pcl::gpu::Octree tree;        
    tree.setCloud(cloud_device);	    
    tree.build();	   
    tree.internalDownload();

    vector<int> sizes;
    sizes.reserve(tests_num);
             
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        vector<int> results;
        tree.radiusSearchHost(data.queries[i], data.radiuses[i], results);                        
    
        std::sort(results.begin(), results.end());

        ASSERT_EQ ( (results == data.bfresutls[i]), true );

        sizes.push_back(results.size());      
    }    

    float avg_size = std::accumulate(sizes.begin(), sizes.end(), 0) * (1.f/sizes.size());;

    cout << "avg_result_size = " << avg_size << endl;
    ASSERT_GT(avg_size, 10);    
}


//TEST (PCL, DISABLED_OctreeGpuHostRadiusSearchVsPCL)
TEST (PCL, OctreeGpuHostRadiusSearchVsPCL)
{
    size_t data_size = 871000;
    size_t tests_num = 10000;
    float cube_size = 1024.f;
    float max_radius_part = 15.f;

    DataGenerator data(data_size, tests_num, cube_size, max_radius_part);
    data.printParams();  

    //prepare gpu cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(data.points);

    //gpu build 
    pcl::gpu::Octree tree;        
    tree.setCloud(cloud_device);	    
    tree.build();	   
    tree.internalDownload();

    //create host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host;
    cloud_host.reset(new pcl::PointCloud<pcl::PointXYZ>);	
    cloud_host->width = data.points.size();
    cloud_host->height = 1;
    cloud_host->points.resize (cloud_host->width * cloud_host->height);
    for (size_t i = 0; i < cloud_host->points.size(); ++i)
    {
        cloud_host->points[i].x = data.points[i].x;
        cloud_host->points[i].y = data.points[i].y;
        cloud_host->points[i].z = data.points[i].z;
    }   

    //cpu build
    float resolution = 0.01f;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloud_host);
    octree.addPointsFromInputCloud ();

    vector<int> sizes;
    sizes.reserve(tests_num);

    //run tests
    for(size_t i = 0; i < data.tests_num; ++i)
    {
        vector<int> results_gpu;
        tree.radiusSearchHost(data.queries[i], data.radiuses[i], results_gpu);                            
        std::sort(results_gpu.begin(), results_gpu.end());

        vector<int> results_cpu;
        vector<float> dists;

        pcl::PointXYZ p = *(pcl::PointXYZ*)&data.queries[i];
        octree.radiusSearch(p, data.radiuses[i], results_cpu, dists);                        
        std::sort(results_cpu.begin(), results_cpu.end());

        ASSERT_EQ ( (results_cpu == results_gpu), true );

        sizes.push_back(results_cpu.size());        
    }        

    float avg_size = std::accumulate(sizes.begin(), sizes.end(), 0) * (1.f/sizes.size());;

    cout << "avg_result_size = " << avg_size << endl;
    ASSERT_GT(avg_size, 10);    
}


int main (int argc, char** argv)
{
    testing::InitGoogleTest (&argc, argv);
    return (RUN_ALL_TESTS ());
}