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
#include <pcl/features/vfh.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/common/time.h>

#include "data_source.hpp"

using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_vfh1)
TEST(PCL_FeaturesGPU, vfh1)
{   
    DataSource source;
    
    source.estimateNormals();
    source.generateIndices(3);
                   
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    gpu::VFHEstimation pc_gpu;    
    pc_gpu.setSearchSurface(cloud_gpu);
    pc_gpu.setInputNormals(normals_gpu);
    pc_gpu.setIndices(indices_gpu);
    pc_gpu.setRadiusSearch(source.radius, source.max_elements);

    DeviceArray<VFHSignature308> vfh_features;
    
    {
        //ScopeTime up("gpu");
        pc_gpu.compute(vfh_features);
    }

    std::vector<VFHSignature308> downloaded;
    vfh_features.download(downloaded);

    pcl::VFHEstimation<PointXYZ, Normal, VFHSignature308> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);
    fe.setRadiusSearch(source.radius);
    fe.setKSearch(0);

    PointCloud<VFHSignature308> vfh;
    {
        //ScopeTime up("cpu");
        fe.compute (vfh);
    }

    VFHSignature308& gpu = downloaded[0];
    VFHSignature308& cpu = vfh[0];
        
    std::size_t FSize = sizeof(VFHSignature308)/sizeof(gpu.histogram[0]);                                
        
    float norm = 0, norm_diff = 0;
    for(std::size_t j = 0; j < FSize; ++j)
    {
        norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
        norm += cpu.histogram[j] * cpu.histogram[j];

        //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
    }
    if (norm != 0)
        ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    else
        FAIL();
}

//TEST(PCL_FeaturesGPU, DISABLED_vfh_norm_bins_false)
TEST(PCL_FeaturesGPU, vfh_norm_bins_false)
{   
    DataSource source;
    
    source.estimateNormals();
    source.generateIndices(3);
                   
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    gpu::VFHEstimation pc_gpu;    
    pc_gpu.setNormalizeBins(false);
    pc_gpu.setSearchSurface(cloud_gpu);
    pc_gpu.setInputNormals(normals_gpu);
    pc_gpu.setIndices(indices_gpu);
    pc_gpu.setRadiusSearch(source.radius, source.max_elements);

    DeviceArray<VFHSignature308> vfh_features;
    pc_gpu.compute(vfh_features);

    std::vector<VFHSignature308> downloaded;
    vfh_features.download(downloaded);

    pcl::VFHEstimation<PointXYZ, Normal, VFHSignature308> fe;
    fe.setNormalizeBins(false);
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);
    fe.setRadiusSearch(source.radius);
    fe.setKSearch(0);

    PointCloud<VFHSignature308> vfh;
    fe.compute (vfh);

    VFHSignature308& gpu = downloaded[0];
    VFHSignature308& cpu = vfh[0];
        
    std::size_t FSize = sizeof(VFHSignature308)/sizeof(gpu.histogram[0]);                                
        
    float norm = 0, norm_diff = 0;
    for(std::size_t j = 0; j < FSize; ++j)
    {
        norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
        norm += cpu.histogram[j] * cpu.histogram[j];

        //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
    }
    if (norm != 0)
        ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    else
        FAIL();
}

//TEST(PCL_FeaturesGPU, DISABLED_vfh_norm_distance_true)
TEST(PCL_FeaturesGPU, vfh_norm_distance_true)
{   
    DataSource source;
    
    source.estimateNormals();
    source.generateIndices(3);
                   
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    gpu::VFHEstimation pc_gpu;    
    pc_gpu.setNormalizeDistance(true);
    pc_gpu.setSearchSurface(cloud_gpu);
    pc_gpu.setInputNormals(normals_gpu);
    pc_gpu.setIndices(indices_gpu);
    pc_gpu.setRadiusSearch(source.radius, source.max_elements);

    DeviceArray<VFHSignature308> vfh_features;
    pc_gpu.compute(vfh_features);

    std::vector<VFHSignature308> downloaded;
    vfh_features.download(downloaded);

    pcl::VFHEstimation<PointXYZ, Normal, VFHSignature308> fe;
    fe.setNormalizeDistance(true);
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);
    fe.setRadiusSearch(source.radius);
    fe.setKSearch(0);

    PointCloud<VFHSignature308> vfh;
    fe.compute (vfh);

    VFHSignature308& gpu = downloaded[0];
    VFHSignature308& cpu = vfh[0];
        
    std::size_t FSize = sizeof(VFHSignature308)/sizeof(gpu.histogram[0]);                                
        
    float norm = 0, norm_diff = 0;
    for(std::size_t j = 0; j < FSize; ++j)
    {
        norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
        norm += cpu.histogram[j] * cpu.histogram[j];

        //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
    }
    if (norm != 0)
        ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    else
        FAIL();
}


//TEST(PCL_FeaturesGPU, DISABLED_vfh_fill_size_component_true)
TEST(PCL_FeaturesGPU, vfh_fill_size_component_true)
{   
    DataSource source;
    
    source.estimateNormals();
    source.generateIndices(3);
                   
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    gpu::VFHEstimation pc_gpu;    
    pc_gpu.setFillSizeComponent(true);
    pc_gpu.setSearchSurface(cloud_gpu);
    pc_gpu.setInputNormals(normals_gpu);
    pc_gpu.setIndices(indices_gpu);
    pc_gpu.setRadiusSearch(source.radius, source.max_elements);

    DeviceArray<VFHSignature308> vfh_features;
    pc_gpu.compute(vfh_features);

    std::vector<VFHSignature308> downloaded;
    vfh_features.download(downloaded);

    pcl::VFHEstimation<PointXYZ, Normal, VFHSignature308> fe;
    fe.setFillSizeComponent(true);
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);
    fe.setRadiusSearch(source.radius);
    fe.setKSearch(0);

    PointCloud<VFHSignature308> vfh;
    fe.compute (vfh);

    VFHSignature308& gpu = downloaded[0];
    VFHSignature308& cpu = vfh[0];
        
    std::size_t FSize = sizeof(VFHSignature308)/sizeof(gpu.histogram[0]);                                
        
    float norm = 0, norm_diff = 0;
    for(std::size_t j = 0; j < FSize; ++j)
    {
        norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
        norm += cpu.histogram[j] * cpu.histogram[j];

        //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);
    }
    if (norm != 0)
        ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    else
        FAIL();
}