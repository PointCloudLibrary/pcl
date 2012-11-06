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

#if (defined(__GNUC__) && !defined(__CUDACC__) && (GTEST_GCC_VER_ >= 40000)) 
    #define GTEST_USE_OWN_TR1_TUPLE 0
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1500)
    #define GTEST_USE_OWN_TR1_TUPLE 0
#endif

#include <gtest/gtest.h>
#include "data_source.hpp"
#include <iostream>

#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/search/search.h>

using namespace std;
using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_normals_lowlevel)
TEST(PCL_FeaturesGPU, normals_lowlevel)
{       
    DataSource source;
    cout << "Cloud size: " << source.cloud->points.size() << endl;
    cout << "Radius: " << source.radius << endl;
    cout << "K: " << source.k << endl;

    //source.runCloudViewer();

    source.estimateNormals();
    source.findKNNeghbors();    

    gpu::NormalEstimation::PointCloud cloud;    
    cloud.upload(source.cloud->points);

    // convert to single array format
    vector<int> neighbors_all(source.max_nn_size * cloud.size());
    PtrStep<int> ps(&neighbors_all[0], source.max_nn_size * PtrStep<int>::elem_size);    
    for(size_t i = 0; i < cloud.size(); ++i)
        copy(source.neighbors_all[i].begin(), source.neighbors_all[i].end(), ps.ptr(i));

    NeighborIndices indices;
    indices.upload(neighbors_all, source.sizes, source.max_nn_size);

    gpu::NormalEstimation::Normals normals;
    gpu::NormalEstimation::computeNormals(cloud, indices, normals);
    gpu::NormalEstimation::flipNormalTowardsViewpoint(cloud, 0.f, 0.f, 0.f, normals);

    vector<PointXYZ> downloaded;
    normals.download(downloaded);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = source.normals->points[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];               

        float abs_error = 0.01f;
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}

//TEST(PCL_FeaturesGPU, DISABLED_normals_highlevel_1)
TEST(PCL_FeaturesGPU, normals_highlevel_1)
{       
    DataSource source;
    cout << "Cloud size: " << source.cloud->points.size() << endl;
    cout << "Radius: " << source.radius << endl;
    cout << "Max_elems: " <<  source.max_elements << endl;

    cout << "!indices, !surface" << endl;
    
    //source.runCloudViewer();

    //source.generateSurface();
    //source.generateIndices();

    pcl::NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (source.cloud);
    ne.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //ne.setKSearch (k);
    ne.setRadiusSearch (source.radius);
    //ne.setSearchSurface(source.surface);
    //ne.setIndices(source.indices);

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    ne.compute (*normals);

    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(source.cloud->points);

    //pcl::gpu::NormalEstimation::PointCloud surface_device;
    //surface_device.upload(source.surface->points);

    //pcl::gpu::NormalEstimation::Indices indices_device;
    //indices_device.upload(source.indices);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(source.radius, source.max_elements);
    //ne_device.setSearchSurface(surface_device);
    //ne_device.setIndices(indices_device);

    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = normals->points[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];                        

        float abs_error = 0.01f;
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}

//TEST(PCL_FeaturesGPU, DISABLED_normals_highlevel_2)
TEST(PCL_FeaturesGPU, normals_highlevel_2)
{       
    DataSource source;
    cout << "Cloud size: " << source.cloud->points.size() << endl;
    cout << "Radius: " << source.radius << endl;
    cout << "Max_elems: " <<  source.max_elements << endl;    

    cout << "indices, !surface" << endl;
    
    //source.runCloudViewer();

    //source.generateSurface();
    source.generateIndices();

    pcl::NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (source.cloud);
    ne.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //ne.setKSearch (k);
    ne.setRadiusSearch (source.radius);
    //ne.setSearchSurface(source.surface);
    ne.setIndices(source.indices);

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    ne.compute (*normals);

    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(source.cloud->points);

    //pcl::gpu::NormalEstimation::PointCloud surface_device;
    //surface_device.upload(source.surface->points);

    pcl::gpu::NormalEstimation::Indices indices_device;
    indices_device.upload(*source.indices);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(source.radius, source.max_elements);
    //ne_device.setSearchSurface(surface_device);
    ne_device.setIndices(indices_device);


    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = normals->points[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];                        

        float abs_error = 0.01f;
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}

//TEST(PCL_FeaturesGPU, DISABLED_normals_highlevel_3)
TEST(PCL_FeaturesGPU, normals_highlevel_3)
{       
    DataSource source;
    cout << "Cloud size: " << source.cloud->points.size() << endl;
    cout << "Radius: " << source.radius << endl;
    cout << "Max_elems: " <<  source.max_elements << endl;

    cout << "!indices, surface" << endl;

    //source.runCloudViewer();

    source.generateSurface();
    //source.generateIndices();

    pcl::NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (source.cloud);
    ne.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //ne.setKSearch (k);
    ne.setRadiusSearch (source.radius);
    ne.setSearchSurface(source.surface);
    //ne.setIndices(source.indices);

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    ne.compute (*normals);

    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(source.cloud->points);

    pcl::gpu::NormalEstimation::PointCloud surface_device;
    surface_device.upload(source.surface->points);

    //pcl::gpu::NormalEstimation::Indices indices_device;
    //indices_device.upload(source.indices);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(source.radius, source.max_elements);
    ne_device.setSearchSurface(surface_device);
    //ne_device.setIndices(indices_device);


    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = normals->points[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];

        float abs_error = 0.01f;

        if (pcl_isnan(n.normal_x) || pcl_isnan(n.normal_y) || pcl_isnan(n.normal_z))
            continue;

        ASSERT_EQ(pcl_isnan(n.normal_x), pcl_isnan(xyz.x));
        ASSERT_EQ(pcl_isnan(n.normal_y), pcl_isnan(xyz.y));
        ASSERT_EQ(pcl_isnan(n.normal_z), pcl_isnan(xyz.z));
        
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_normals_highlevel_4)
TEST(PCL_FeaturesGPU, normals_highlevel_4)
{       
    DataSource source;
    cout << "Cloud size: " << source.cloud->points.size() << endl;
    cout << "Radius: " << source.radius << endl;
    cout << "Max_elems: " <<  source.max_elements << endl;
    
    cout << "indices, surface" << endl;

    //source.runCloudViewer();

    source.generateSurface();
    source.generateIndices();

    pcl::NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (source.cloud);
    ne.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //ne.setKSearch (k);
    ne.setRadiusSearch (source.radius);
    ne.setSearchSurface(source.surface);
    ne.setIndices(source.indices);

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    ne.compute (*normals);

    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(source.cloud->points);

    pcl::gpu::NormalEstimation::PointCloud surface_device;
    surface_device.upload(source.surface->points);

    pcl::gpu::NormalEstimation::Indices indices_device;
    indices_device.upload(*source.indices);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(source.radius, source.max_elements);
    ne_device.setSearchSurface(surface_device);
    ne_device.setIndices(indices_device);


    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

   for(size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = normals->points[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];

        float abs_error = 0.01f;

        if (pcl_isnan(n.normal_x) || pcl_isnan(n.normal_y) || pcl_isnan(n.normal_z))
            continue;

        ASSERT_EQ(pcl_isnan(n.normal_x), pcl_isnan(xyz.x));
        ASSERT_EQ(pcl_isnan(n.normal_y), pcl_isnan(xyz.y));
        ASSERT_EQ(pcl_isnan(n.normal_z), pcl_isnan(xyz.z));
        
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}


int main (int argc, char** argv)
{
    pcl::gpu::setDevice(0);
    pcl::gpu::printShortCudaDeviceInfo(0);
    testing::InitGoogleTest (&argc, argv);
    return (RUN_ALL_TESTS ());
}
