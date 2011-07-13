#pragma warning (disable : 4996 4530)

#include <gtest/gtest.h>

#include<iostream>

#pragma warning (disable: 4521)
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#pragma warning (default: 4521)

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/device_array.hpp"
#include "pcl/gpu/common/timers_opencv.hpp"

#include "data_gen.hpp"

#include <opencv2/contrib/contrib.hpp>

using namespace pcl::gpu;
using namespace std;

ostream& operator<<(ostream& out, pcl::gpu::PointXYZ p) { return out << "[" << p.x << ", " << p.y << ", " << p.x << "]"; }

class OctreeGpuTest : public testing::Test 
{
protected:
    DataGenerator data;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host;
    pcl::gpu::Octree::PointCloud cloud_device;

    float pcl_octree_resolution;

    const static int opencv_octree_points_per_leaf = 32;
        
    OctreeGpuTest() : pcl_octree_resolution(4.0f)
    {                   
        cout << "pcl::octree resolution = " << pcl_octree_resolution << endl;
        
        cloud_device.upload(data.points);

        //create PCL cloud
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
    }       
    
    virtual void SetUp()  {}    // virtual void SetUp() will be called before each test is run.  
    virtual void TearDown()  {} // virtual void TearDown() will be called after each test is run.     
};

//TEST_F(OctreeGpuTest, DISABLED_BuildPerfomance)
TEST_F(OctreeGpuTest, BuildPerfomance)
{
    pcl::gpu::Octree tree;    
    tree.setCloud(cloud_device);	

    {
        ScopeTimerCV up("gpu-build");	        
        tree.build();	
    }
    {
        ScopeTimerCV up("gpu-downloadInternal");	
        tree.internalDownload();
    }

    //pcl-build            
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (pcl_octree_resolution);
    {
        ScopeTimerCV t("pcl-build");	

        octree.setInputCloud (cloud_host);
        octree.addPointsFromInputCloud ();
    }

    {
        // another my (anatoly baksheev's) octree implementation contributed in 2008
        // The goal was not to implement fast build procedure

        ScopeTimerCV t("opencv-build");	
        cv::Octree opencv_octree;
        vector<cv::Point3f>& opencv_points = (vector<cv::Point3f>&)data.points;
        opencv_octree.buildTree(opencv_points, 10, opencv_octree_points_per_leaf); 
    }
    
}

//TEST_F(OctreeGpuTest, DISABLED_RadiusSearchHostPerfomance)
TEST_F(OctreeGpuTest, RadiusSearchHostPerfomance)
{
    pcl::gpu::Octree tree;    
    tree.setCloud(cloud_device);	
    tree.build();	
    tree.internalDownload();
    
    //pcl-build            
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (pcl_octree_resolution);
    octree.setInputCloud (cloud_host);
    octree.addPointsFromInputCloud ();

    cv::Octree opencv_octree;
    vector<cv::Point3f>& opencv_points = (vector<cv::Point3f>&)data.points;
    opencv_octree.buildTree(opencv_points, 10, opencv_octree_points_per_leaf); 

    vector<int> inds;
    vector<float> pointRadiusSquaredDistance;
    vector<cv::Point3f> opencv_results;
    inds.reserve(10000);
    pointRadiusSquaredDistance.reserve(10000);    
    opencv_results.reserve(10000);  

    {                
        ScopeTimerCV up("pcl-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
        {                                                
            pcl::PointXYZ searchPoint = *(pcl::PointXYZ*)&data.queries[i];                                        
            octree.radiusSearch (searchPoint, data.radiuses[i], inds, pointRadiusSquaredDistance);                        
        }
    }

    {
        ScopeTimerCV up("gpu-host-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
            tree.radiusSearchHost(data.queries[i], data.radiuses[i], inds);                        
    }

    
    {
        ScopeTimerCV up("opencv-search-all");	
        for(size_t i = 0; i < data.tests_num; ++i)
        {
            const cv::Point3f& center = (const cv::Point3f&)data.queries[i];
            opencv_octree.getPointsWithinSphere( center, data.radiuses[i], opencv_results );
        }            
    }    
}


TEST_F(OctreeGpuTest, DISABLED_batch)
{   
    float radius = data.cube_size/20;

    //my-build
    pcl::gpu::Octree tree;    
    tree.setCloud(cloud_device);	

    {
        ScopeTimerCV up("my-build");	        
        tree.build();	
    }
    {
        ScopeTimerCV up("my-downloadInternal");	
        tree.internalDownload();
    }
    //pcl-build            
  /*  float resolution = 4.f;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (resolution);
    {
        ScopeTimerCV t("pcl_build");	

        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();
    }

    vector< vector<int> > res_my(tests_num);
    vector< vector<int> > res_pcl(tests_num);
    std::vector<float> pointRadiusSquaredDistance;
    pointRadiusSquaredDistance.reserve(data.size());

    
    {
        ScopeTimerCV up("search-pcl-all");	
        for(size_t i = 0; i < tests_num; ++i)
        {                                    
            vector<int>& inds_pcl = res_pcl[i];
            pcl::PointXYZ searchPoint = *(pcl::PointXYZ*)&queries[i];                                        
            octree.radiusSearch (searchPoint, radius, inds_pcl, pointRadiusSquaredDistance);                        
        }
    }*/

    pcl::gpu::Octree::BatchQueries queries_device;
    queries_device.upload(data.queries);

    const int max_points = 500;
    pcl::gpu::Octree::BatchResult      result_device(queries_device.size(), max_points);
    pcl::gpu::Octree::BatchResultSizes  sizes_device(queries_device.size());

    cout << "queries = " << queries_device.size() << endl;
    cout << "sizes_device = " << sizes_device.size() << endl;

    {
        ScopeTimerCV up("search-gpu-batch");	        
        tree.radiusSearchBatchGPU(queries_device, radius, result_device, sizes_device);
    }
    vector<int> sizes;
    sizes_device.download(sizes);

    cout << "Size = " << sizes[0] << endl;

    /*for(int i = 0; i < tests_num; ++i)
    {
        size_t size_pcl = res_pcl[i].size();
        size_t size_my = (size_t)sizes[i];

        if (size_pcl != size_my)
            cout << "Different i = " << i << ", size_pcl = " << size_pcl << ", size_my = " << size_my << endl;
    }*/

    /*for(size_t i = 0; i < tests_num; ++i)
    {
        vector<int>& inds_my = res_my[i];
        vector<int>& inds_pcl = res_pcl[i];

        std::sort(inds_my.begin(), inds_my.end());
        std::sort(inds_pcl.begin(), inds_pcl.end());

        if (inds_my != inds_pcl)
        {
            cout << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\ntest failed, i = " << i << endl;
            return;
        }
        else        
            cout << "the same = " << inds_my.size() << endl;
    }*/
}


TEST_F(OctreeGpuTest, DISABLED_host)
{               
    //my-build
    pcl::gpu::Octree tree;    
    tree.setCloud(cloud_device);	
    {
        ScopeTimerCV up("my-build");	        
        tree.build();	
    }
    {
        ScopeTimerCV up("my-downloadInternal");	
        tree.internalDownload();
    }

    //pcl-build            
    float resolution = 4.f;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (resolution);
    {
        ScopeTimerCV t("pcl_build");	

        octree.setInputCloud (cloud_host);
        octree.addPointsFromInputCloud ();
    }

    vector< vector<int> > res_my(data.tests_num);
    vector< vector<int> > res_pcl(data.tests_num);
    {
        ScopeTimerCV up("search-pcl");	
        for(size_t i = 0; i < data.tests_num; ++i)
        {                        
            vector<int>& inds_pcl = res_pcl[i];

            pcl::PointXYZ searchPoint = *(pcl::PointXYZ*)&data.queries[i];
            std::vector<float> pointRadiusSquaredDistance;
            {
                //ScopeTimerCV up("search-pcl");	
                octree.radiusSearch(searchPoint, data.radiuses[i], inds_pcl, pointRadiusSquaredDistance);            
            }            
        }
    }
    {
        ScopeTimerCV up("search-my");	
        for(size_t i = 0; i < data.tests_num; ++i)
            tree.radiusSearchHost(data.queries[i], data.radiuses[i], res_my[i]);                        
    }

    for(size_t i = 0; i < data.tests_num; ++i)
    {
        vector<int>& inds_my = res_my[i];
        vector<int>& inds_pcl = res_pcl[i];

        std::sort(inds_my.begin(), inds_my.end());
        std::sort(inds_pcl.begin(), inds_pcl.end());

        if (inds_my != inds_pcl)
        {
            cout << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\ntest failed, i = " << i << endl;
            return;
        }
        else
        {
         //   cout << "the same = " << inds_my.size() << endl;
        }
    }
}

