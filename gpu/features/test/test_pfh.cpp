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

#include "gtest/gtest.h"

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/gpu/features/features.hpp>
#include "data_source.hpp"
#include <pcl/search/search.h>

using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_pfh_low_level)
TEST(PCL_FeaturesGPU, pfh_low_level)
{   
    DataSource source;

    source.estimateNormals();
    source.findRadiusNeghbors();
    std::cout << "max_radius_nn_size: " << source.max_nn_size << std::endl;
                   
    std::vector<int> data;
    source.getNeghborsArray(data);
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());
    
    //uploading data to GPU
    pcl::gpu::PFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::PFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             
    
    pcl::gpu::NeighborIndices indices;
    indices.upload(data, source.sizes, source.max_nn_size);
    
    DeviceArray2D<PFHSignature125> pfh125_features;
    
    gpu::PFHEstimation pfh_gpu;            
    pfh_gpu.compute(cloud_gpu, normals_gpu, indices, pfh125_features);

    int stub;
    std::vector<PFHSignature125> downloaded;
    pfh125_features.download(downloaded, stub);

    pcl::PFHEstimation<PointXYZ, Normal, PFHSignature125> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);

    PointCloud<PFHSignature125> pfhs;
    fe.compute (pfhs);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PFHSignature125& gpu = downloaded[i];
        PFHSignature125& cpu = pfhs[i];
        
        std::size_t FSize = sizeof(PFHSignature125)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];
            
            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }
        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
        //printf("norm_diff/norm = %f %f %f\n", norm_diff/norm, norm_diff, norm);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_pfh_high_level1)
TEST(PCL_FeaturesGPU, pfh_high_level1)
{       
    DataSource source;
    source.estimateNormals();    
  
    //source.generateSurface();
    //source.generateIndices();

    std::cout << "!indices, !surface" << std::endl;

    PointCloud<Normal>::Ptr& normals = source.normals;

    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(normals->points.begin(), normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());

    //uploading data to GPU
    pcl::gpu::PFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points); 

    pcl::gpu::PFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);                 

    //pcl::gpu::PFHEstimation::Indices indices_gpu;
    //indices_gpu.upload(*source.indices);

    //pcl::gpu::PFHEstimation::PointCloud surface_gpu;
    //surface_gpu.upload(source.surface->points);

    //GPU call
    pcl::gpu::PFHEstimation fe_gpu;
    fe_gpu.setInputCloud (cloud_gpu);
    fe_gpu.setInputNormals (normals_gpu);
    fe_gpu.setRadiusSearch (source.radius, source.max_elements);
    //fe_gpu.setIndices(indices_gpu);
    //fe_gpu.setSearchSurface(surface_gpu);  

    DeviceArray2D<PFHSignature125> fpfhs_gpu;
    fe_gpu.compute(fpfhs_gpu);
                               
      // CPU call
    pcl::PFHEstimation<PointXYZ, Normal, PFHSignature125> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);
    //fe.setIndices(source.indices);
    //fe.setSearchSurface(source.surface);

    PointCloud<PFHSignature125> fpfhs;
    fe.compute (fpfhs);

    int stub;
    std::vector<PFHSignature125> downloaded;
    fpfhs_gpu.download(downloaded, stub);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PFHSignature125& gpu = downloaded[i];
        PFHSignature125& cpu = fpfhs[i];
        
        std::size_t FSize = sizeof(PFHSignature125)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }
        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_pfh_high_level2)
TEST(PCL_FeaturesGPU, pfh_high_level2)
{       
    DataSource source;
    source.estimateNormals();    
  
    //source.generateSurface();
    source.generateIndices();

    std::cout << "indices, !surface" << std::endl;

    PointCloud<Normal>::Ptr& normals = source.normals;

    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(normals->points.begin(), normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());

    //uploading data to GPU
    pcl::gpu::PFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points); 

    pcl::gpu::PFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);                 

    pcl::gpu::PFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    //pcl::gpu::PFHEstimation::PointCloud surface_gpu;
    //surface_gpu.upload(source.surface->points);

    //GPU call
    pcl::gpu::PFHEstimation fe_gpu;
    fe_gpu.setInputCloud (cloud_gpu);
    fe_gpu.setInputNormals (normals_gpu);
    fe_gpu.setRadiusSearch (source.radius, source.max_elements);
    fe_gpu.setIndices(indices_gpu);
    //fe_gpu.setSearchSurface(surface_gpu);  

    DeviceArray2D<PFHSignature125> fpfhs_gpu;
    fe_gpu.compute(fpfhs_gpu);
                               
      // CPU call
    pcl::PFHEstimation<PointXYZ, Normal, PFHSignature125> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);
    fe.setIndices(source.indices);
    //fe.setSearchSurface(source.surface);

    PointCloud<PFHSignature125> fpfhs;
    fe.compute (fpfhs);

    int stub;
    std::vector<PFHSignature125> downloaded;
    fpfhs_gpu.download(downloaded, stub);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PFHSignature125& gpu = downloaded[i];
        PFHSignature125& cpu = fpfhs[i];
        
        std::size_t FSize = sizeof(PFHSignature125)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }
        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_pfh_high_level3)
TEST(PCL_FeaturesGPU, pfh_high_level3)
{       
    DataSource source;
    source.estimateNormals();    
   
    source.generateSurface();
    //source.generateIndices();

    std::cout << "!indices, surface" << std::endl;

    PointCloud<Normal>::Ptr& normals = source.normals_surface;

    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(normals->points.begin(), normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());

    //uploading data to GPU
    pcl::gpu::PFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points); 

    pcl::gpu::PFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);                 

    //pcl::gpu::PFHEstimation::Indices indices_gpu;
    //indices_gpu.upload(*source.indices);

    pcl::gpu::PFHEstimation::PointCloud surface_gpu;
    surface_gpu.upload(source.surface->points);

    //GPU call
    pcl::gpu::PFHEstimation fe_gpu;
    fe_gpu.setInputCloud (cloud_gpu);
    fe_gpu.setInputNormals (normals_gpu);
    fe_gpu.setRadiusSearch (source.radius, source.max_elements);
    //fe_gpu.setIndices(indices_gpu);
    fe_gpu.setSearchSurface(surface_gpu);  

    DeviceArray2D<PFHSignature125> fpfhs_gpu;
    fe_gpu.compute(fpfhs_gpu);
                               
      // CPU call
    pcl::PFHEstimation<PointXYZ, Normal, PFHSignature125> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);
    //fe.setIndices(source.indices);
    fe.setSearchSurface(source.surface);

    PointCloud<PFHSignature125> fpfhs;
    fe.compute (fpfhs);

    int stub;
    std::vector<PFHSignature125> downloaded;
    fpfhs_gpu.download(downloaded, stub);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PFHSignature125& gpu = downloaded[i];
        PFHSignature125& cpu = fpfhs[i];
        
        std::size_t FSize = sizeof(PFHSignature125)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }                            
        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_pfh_high_level4)
TEST(PCL_FeaturesGPU, pfh_high_level4)
{       
    DataSource source;
    source.estimateNormals();    
    
    source.generateSurface();
    source.generateIndices();

    std::cout << "indices, surface" << std::endl;

    PointCloud<Normal>::Ptr& normals = source.normals_surface;

    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(normals->points.begin(), normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());

    //uploading data to GPU
    pcl::gpu::PFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points); 

    pcl::gpu::PFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);                 

    pcl::gpu::PFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    pcl::gpu::PFHEstimation::PointCloud surface_gpu;
    surface_gpu.upload(source.surface->points);

    //GPU call
    pcl::gpu::PFHEstimation fe_gpu;
    fe_gpu.setInputCloud (cloud_gpu);
    fe_gpu.setInputNormals (normals_gpu);
    fe_gpu.setRadiusSearch (source.radius, source.max_elements);
    fe_gpu.setIndices(indices_gpu);
    fe_gpu.setSearchSurface(surface_gpu);  

    DeviceArray2D<PFHSignature125> fpfhs_gpu;
    fe_gpu.compute(fpfhs_gpu);
                               
      // CPU call
    pcl::PFHEstimation<PointXYZ, Normal, PFHSignature125> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);
    fe.setIndices(source.indices);
    fe.setSearchSurface(source.surface);

    PointCloud<PFHSignature125> fpfhs;
    fe.compute (fpfhs);

    int stub;
    std::vector<PFHSignature125> downloaded;
    fpfhs_gpu.download(downloaded, stub);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PFHSignature125& gpu = downloaded[i];
        PFHSignature125& cpu = fpfhs[i];
        
        std::size_t FSize = sizeof(PFHSignature125)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }
        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}

//TEST(PCL_FeaturesGPU, DISABLED_pfhrgb)
TEST(PCL_FeaturesGPU, pfhrgb)
{       
    DataSource source;
    source.generateColor();
    source.estimateNormals();    
  
    //source.generateSurface();
    //source.generateIndices();

    std::cout << "!indices, !surface" << std::endl;

    PointCloud<Normal>::Ptr& normals = source.normals;

    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(normals->points.begin(), normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());

    //uploading data to GPU
    pcl::gpu::PFHRGBEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points); 

    pcl::gpu::PFHRGBEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);                 

    //pcl::gpu::PFHEstimation::Indices indices_gpu;
    //indices_gpu.upload(*source.indices);

    //pcl::gpu::PFHEstimation::PointCloud surface_gpu;
    //surface_gpu.upload(source.surface->points);

    //GPU call
    pcl::gpu::PFHRGBEstimation fe_gpu;
    fe_gpu.setInputCloud (cloud_gpu);
    fe_gpu.setInputNormals (normals_gpu);
    fe_gpu.setRadiusSearch (source.radius, source.max_elements);
    //fe_gpu.setIndices(indices_gpu);
    //fe_gpu.setSearchSurface(surface_gpu);  

    DeviceArray2D<PFHRGBSignature250> fpfhs_gpu;
    fe_gpu.compute(fpfhs_gpu);
                               
      // CPU call
    pcl::PFHRGBEstimation<PointXYZRGB, Normal, PFHRGBSignature250> fe;

    PointCloud<PointXYZRGB>::Ptr cloud_XYZRGB(new PointCloud<PointXYZRGB>());
    cloud_XYZRGB->points.clear();
    for (const auto& p: *source.cloud)
    {
        PointXYZRGB o;

        int color = *(int*)&p.data[3];
        int r =  color        & 0xFF;
        int g = (color >>  8) & 0xFF;
        int b = (color >> 16) & 0xFF;

        o.x = p.x; o.y = p.y; o.z = p.z;
        o.r = r; o.g = g; o.b = b;
        
        cloud_XYZRGB->points.push_back(o);
    }
    cloud_XYZRGB->width = cloud_XYZRGB->size();
    cloud_XYZRGB->height = 1;
            
    fe.setInputCloud (cloud_XYZRGB);
    fe.setInputNormals (normals);
    fe.setSearchMethod (pcl::search::KdTree<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    //fe.setKSearch (k);
    fe.setRadiusSearch (source.radius);
    //fe.setIndices(source.indices);
    //fe.setSearchSurface(source.surface);

    PointCloud<PFHRGBSignature250> fpfhs;
    fe.compute (fpfhs);

    int stub;
    std::vector<PFHRGBSignature250> downloaded;
    fpfhs_gpu.download(downloaded, stub);

    for(std::size_t i = 170; i < downloaded.size(); ++i)
    {
        PFHRGBSignature250& gpu = downloaded[i];
        PFHRGBSignature250& cpu = fpfhs[i];
        
        std::size_t FSize = sizeof(PFHRGBSignature250)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(std::size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
        }
        if (norm != 0)            
            ASSERT_LE(norm_diff/norm, 0.01f);
    }
}
