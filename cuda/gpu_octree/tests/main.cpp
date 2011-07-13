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

using namespace pcl::gpu;
using namespace std;

ostream& operator<<(ostream& out, pcl::gpu::PointXYZ p) { return out << "[" << p.x << ", " << p.y << ", " << p.x << "]"; }

class OctreeGpuTest : public testing::Test 
{
protected:
    vector<pcl::gpu::PointXYZ> data;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::gpu::Octree::PointCloud cloud_device;


    vector<pcl::gpu::PointXYZ> queries;
    vector<float> radiues;
    
    const static size_t data_size = 871000;
    //const static size_t data_size = 300000;
        
    const static size_t tests_num = 1;

    const float cube_size;
    cv::RNG& rng;

    OctreeGpuTest() : cube_size(1024.f), rng(cv::theRNG()), data(data_size)
    {
        //generate data                                       
        for(size_t i = 0; i < data.size(); ++i)
        {            
            data[i].x = (float)rng * cube_size;  
            data[i].y = (float)rng * cube_size;  
            data[i].z = (float)rng * cube_size;
        }
        cout << "Points number = " << data.size() << endl;    

        //upload
        cloud_device.upload(data);

        //create PCL cloud
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);	
        cloud->width = data.size();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            cloud->points[i].x = data[i].x;
            cloud->points[i].y = data[i].y;
            cloud->points[i].z = data[i].z;
        }

        queries.resize(tests_num);
        radiues.resize(tests_num);
        for (size_t i = 0; i < tests_num; ++i)
        {
            pcl::gpu::PointXYZ p;
            radiues[i] = generate_query(p);
            queries[i] = p;
        }
    }       
    
    virtual void SetUp()  {}  // virtual void SetUp() will be called before each test is run.  
    virtual void TearDown()  {}     // virtual void TearDown() will be called after each test is run.

    float generate_query(PointXYZ& point)
    {
        point.x = (float)rng * cube_size;  
        point.y = (float)rng * cube_size;  
        point.z = (float)rng * cube_size;  		
        return (float)rng * cube_size / 15.f;	
    }

    void bruteForceSearch(const PointXYZ& p, float radius, vector<int>& out)
    {
        out.clear();
        
        for(size_t i = 0; i < data.size(); ++i)
        {
            float dx = data[i].x - p.x;
            float dy = data[i].y - p.y;
            float dz = data[i].z - p.z;                
            if (dx * dx + dy*dy + dz*dz  < radius * radius)
                out.push_back(i);                
        }
        std::sort(out.begin(), out.end());
    }
};

TEST_F(OctreeGpuTest, batch)
{   
    float radius = cube_size/20;

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
    queries_device.upload(queries);

    const int max_points = 500;
    pcl::gpu::Octree::BatchResult result_device(queries.size(), max_points);
    pcl::gpu::DeviceArray_<int> sizes_device(queries.size());

    cout << "queries = " << queries.size() << endl;
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

        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();
    }

    vector< vector<int> > res_my(tests_num);
    vector< vector<int> > res_pcl(tests_num);
    {
        ScopeTimerCV up("search-pcl");	
        for(size_t i = 0; i < tests_num; ++i)
        {                        
            vector<int>& inds_pcl = res_pcl[i];

            pcl::PointXYZ searchPoint = *(pcl::PointXYZ*)&queries[i];
            std::vector<float> pointRadiusSquaredDistance;
            {
                //ScopeTimerCV up("search-pcl");	
                octree.radiusSearch(searchPoint, radiues[i], inds_pcl, pointRadiusSquaredDistance);            
            }            
        }
    }
    {
        ScopeTimerCV up("search-my");	
        for(size_t i = 0; i < tests_num; ++i)
            tree.radiusSearchHost(queries[i], radiues[i], res_my[i]);                        
    }

    for(size_t i = 0; i < tests_num; ++i)
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


int main (int argc, char** argv)
{
    testing::InitGoogleTest (&argc, argv);
    return (RUN_ALL_TESTS ());
}
