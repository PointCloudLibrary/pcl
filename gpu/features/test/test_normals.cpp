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
#include "data_source.hpp"
#include <iostream>

#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/search/search.h>

using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_normals_lowlevel)
TEST(PCL_FeaturesGPU, normals_lowlevel)
{       
    DataSource source;
    std::cout << "Cloud size: " << source.cloud->size() << std::endl;
    std::cout << "Radius: " << source.radius << std::endl;
    std::cout << "K: " << source.k << std::endl;

    //source.runCloudViewer();

    source.estimateNormals();
    source.findKNNeghbors();    

    gpu::NormalEstimation::PointCloud cloud;    
    cloud.upload(source.cloud->points);

    // convert to single array format
    std::vector<int> neighbors_all(source.max_nn_size * cloud.size());
    PtrStep<int> ps(&neighbors_all[0], source.max_nn_size * PtrStep<int>::elem_size);    
    for(std::size_t i = 0; i < cloud.size(); ++i)
        copy(source.neighbors_all[i].begin(), source.neighbors_all[i].end(), ps.ptr(i));

    NeighborIndices indices;
    indices.upload(neighbors_all, source.sizes, source.max_nn_size);

    gpu::NormalEstimation::Normals normals;
    gpu::NormalEstimation::computeNormals(cloud, indices, normals);
    gpu::NormalEstimation::flipNormalTowardsViewpoint(cloud, 0.f, 0.f, 0.f, normals);

    std::vector<PointXYZ> downloaded;
    normals.download(downloaded);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = (*source.normals)[i];

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
    std::cout << "Cloud size: " << source.cloud->size() << std::endl;
    std::cout << "Radius: " << source.radius << std::endl;
    std::cout << "Max_elems: " <<  source.max_elements << std::endl;

    std::cout << "!indices, !surface" << std::endl;
    
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

    std::vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = (*normals)[i];

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
    std::cout << "Cloud size: " << source.cloud->size() << std::endl;
    std::cout << "Radius: " << source.radius << std::endl;
    std::cout << "Max_elems: " <<  source.max_elements << std::endl;    

    std::cout << "indices, !surface" << std::endl;
    
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

    std::vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = (*normals)[i];

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
    std::cout << "Cloud size: " << source.cloud->size() << std::endl;
    std::cout << "Radius: " << source.radius << std::endl;
    std::cout << "Max_elems: " <<  source.max_elements << std::endl;

    std::cout << "!indices, surface" << std::endl;

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

    std::vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = (*normals)[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];

        float abs_error = 0.01f;

        if (std::isnan(n.normal_x) || std::isnan(n.normal_y) || std::isnan(n.normal_z))
            continue;

        ASSERT_EQ(std::isnan(n.normal_x), std::isnan(xyz.x));
        ASSERT_EQ(std::isnan(n.normal_y), std::isnan(xyz.y));
        ASSERT_EQ(std::isnan(n.normal_z), std::isnan(xyz.z));
        
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
    std::cout << "Cloud size: " << source.cloud->size() << std::endl;
    std::cout << "Radius: " << source.radius << std::endl;
    std::cout << "Max_elems: " <<  source.max_elements << std::endl;
    
    std::cout << "indices, surface" << std::endl;

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

    std::vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

   for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        Normal n = (*normals)[i];

        PointXYZ xyz = downloaded[i];
        float curvature = xyz.data[3];

        float abs_error = 0.01f;

        if (std::isnan(n.normal_x) || std::isnan(n.normal_y) || std::isnan(n.normal_z))
            continue;

        ASSERT_EQ(std::isnan(n.normal_x), std::isnan(xyz.x));
        ASSERT_EQ(std::isnan(n.normal_y), std::isnan(xyz.y));
        ASSERT_EQ(std::isnan(n.normal_z), std::isnan(xyz.z));
        
        ASSERT_NEAR(n.normal_x, xyz.x, abs_error);
        ASSERT_NEAR(n.normal_y, xyz.y, abs_error);
        ASSERT_NEAR(n.normal_z, xyz.z, abs_error);

        float abs_error_curv = 0.01f;
        ASSERT_NEAR(n.curvature, curvature, abs_error_curv);
    }
}

// Test from issue:
// - https://github.com/PointCloudLibrary/pcl/issues/2371#issuecomment-577727912
TEST(PCL_FeaturesGPU, issue_2371)
{
    // This number is magic, do not set to lower value.
    // It may affect error reproducibility.
    const std::size_t N = 1000;
    std::vector<pcl::PointXYZ> cloud_cpu(N, {0.0, 0.0, 0.0});

    pcl::gpu::NormalEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(cloud_cpu);

    pcl::gpu::NormalEstimation ne_gpu;
    ne_gpu.setInputCloud(cloud_gpu);

    const float radius_search = 2.0F;
    const int max_results = 500;
    ne_gpu.setRadiusSearch(radius_search, max_results);

    pcl::gpu::NormalEstimation::Normals normals_gpu;
    ne_gpu.compute(normals_gpu);
}

// See:
// - https://github.com/PointCloudLibrary/pcl/pull/3627#discussion_r375826172
TEST(PCL_FeaturesGPU, normals_nan_gpu)
{
    const std::size_t N = 5;

    PointCloud<PointXYZ> cloud;
    cloud.assign(N, {0.0, 0.0, 0.0});

    const float radius_search = 2.0F;
    const int max_results = 500;

    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    cloud_device.upload(cloud.points);

    pcl::gpu::NormalEstimation ne_device;
    ne_device.setInputCloud(cloud_device);
    ne_device.setRadiusSearch(radius_search, max_results);

    pcl::gpu::NormalEstimation::Normals normals_device;
    ne_device.compute(normals_device);

    std::vector<PointXYZ> downloaded;
    normals_device.download(downloaded);

    ASSERT_EQ(downloaded.size(), N);

    for(const auto& n : downloaded)
    {
        ASSERT_TRUE(std::isnan(n.x));
        ASSERT_TRUE(std::isnan(n.y));
        ASSERT_TRUE(std::isnan(n.z));
    }
}

int main (int argc, char** argv)
{
    pcl::gpu::setDevice(0);
    pcl::gpu::printShortCudaDeviceInfo(0);
    testing::InitGoogleTest (&argc, argv);
    return (RUN_ALL_TESTS ());
}
