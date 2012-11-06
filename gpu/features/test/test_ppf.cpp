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

#include "gtest/gtest.h"

#include <pcl/point_types.h>
#include <pcl/features/ppf.h>
#include <pcl/features/impl/ppf.hpp>
#include <pcl/features/ppfrgb.h>
#include <pcl/features/impl/ppfrgb.hpp>
#include <pcl/gpu/features/features.hpp>

#include "data_source.hpp"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_ppf)
TEST(PCL_FeaturesGPU, ppf)
{   
    DataSource source;

    source.generateIndices();
    source.estimateNormals();
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::PPFEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::PPFEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::PPFEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    DeviceArray<PPFSignature> ppf_features;
    
    gpu::PPFEstimation pph_gpu;    
    pph_gpu.setInputCloud(cloud_gpu);
    pph_gpu.setInputNormals(normals_gpu);
    pph_gpu.setIndices(indices_gpu);
    pph_gpu.compute(ppf_features);


    vector<PPFSignature> downloaded;
    ppf_features.download(downloaded);

    pcl::PPFEstimation<PointXYZ, Normal, PPFSignature> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);

    PointCloud<PPFSignature> ppfs;
    fe.compute (ppfs);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        PPFSignature& gpu = downloaded[i];
        PPFSignature& cpu = ppfs.points[i];        

        ASSERT_NEAR(gpu.f1, cpu.f1, 0.01f);
        ASSERT_NEAR(gpu.f2, cpu.f2, 0.01f);
        ASSERT_NEAR(gpu.f3, cpu.f3, 0.01f);
        ASSERT_NEAR(gpu.f4, cpu.f4, 0.01f);
        ASSERT_NEAR(gpu.alpha_m, cpu.alpha_m, 0.01f);              
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_ppfrgb)
TEST(PCL_FeaturesGPU, ppfrgb)
{   
    DataSource source;
    source.generateColor();

    source.generateIndices();
    source.estimateNormals();
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::PPFRGBEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::PPFRGBEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::PPFRGBEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    DeviceArray<PPFRGBSignature> ppf_features;
    
    gpu::PPFRGBEstimation pph_gpu;    
    pph_gpu.setInputCloud(cloud_gpu);
    pph_gpu.setInputNormals(normals_gpu);
    pph_gpu.setIndices(indices_gpu);
    pph_gpu.compute(ppf_features);

    vector<PPFRGBSignature> downloaded;
    ppf_features.download(downloaded);

    pcl::PPFRGBEstimation<PointXYZRGB, Normal, PPFRGBSignature> fe;
    
    PointCloud<PointXYZRGB>::Ptr cloud_XYZRGB(new PointCloud<PointXYZRGB>());
    cloud_XYZRGB->points.clear();
    for(size_t i = 0; i < source.cloud->points.size(); ++i)               
    {
        const PointXYZ& p = source.cloud->points[i];        
        int color = *(int*)&p.data[3];
        int r =  color        & 0xFF;
        int g = (color >>  8) & 0xFF;
        int b = (color >> 16) & 0xFF;

        PointXYZRGB o;
        o.x = p.x; o.y = p.y; o.z = p.z; o.r = r; o.g = g; o.b = b;        
        cloud_XYZRGB->points.push_back(o);
    }
    cloud_XYZRGB->width = cloud_XYZRGB->points.size();
    cloud_XYZRGB->height = 1;


    fe.setInputCloud (cloud_XYZRGB);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);

    PointCloud<PPFRGBSignature> ppfs;
    fe.compute (ppfs);

    for(size_t i = 207025; i < downloaded.size(); ++i)
    {
        PPFRGBSignature& gpu = downloaded[i];
        PPFRGBSignature& cpu = ppfs.points[i];        

        ASSERT_NEAR(gpu.f1, cpu.f1, 0.01f);
        ASSERT_NEAR(gpu.f2, cpu.f2, 0.01f);
        ASSERT_NEAR(gpu.f3, cpu.f3, 0.01f);
        ASSERT_NEAR(gpu.f4, cpu.f4, 0.01f);
        ASSERT_NEAR(gpu.alpha_m, cpu.alpha_m, 0.01f); 

        if (pcl_isnan(gpu.r_ratio) || pcl_isnan(gpu.g_ratio) || pcl_isnan(gpu.b_ratio) || 
            pcl_isnan(cpu.r_ratio) || pcl_isnan(cpu.g_ratio) || pcl_isnan(cpu.b_ratio))
            continue;
        
        ASSERT_NEAR(gpu.r_ratio, cpu.r_ratio, 0.01f);
        ASSERT_NEAR(gpu.g_ratio, cpu.g_ratio, 0.01f);
        ASSERT_NEAR(gpu.b_ratio, cpu.b_ratio, 0.01f);     
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_ppfrgb_region)
TEST(PCL_FeaturesGPU, ppfrgb_region)
{   
    DataSource source;

    source.generateColor();

    source.generateIndices();
    source.radius/=2.f;

    source.estimateNormals();
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());
    
    //uploading data to GPU
    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::PPFRGBRegionEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::PPFRGBRegionEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::PPFRGBRegionEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);

    DeviceArray<PPFRGBSignature> ppf_features;
       
    gpu::PPFRGBRegionEstimation pph_gpu;    
    pph_gpu.setInputCloud(cloud_gpu);
    pph_gpu.setInputNormals(normals_gpu);
    pph_gpu.setIndices(indices_gpu);

    pph_gpu.setRadiusSearch(source.radius, source.max_elements);

    pph_gpu.compute(ppf_features);

    vector<PPFRGBSignature> downloaded;
    ppf_features.download(downloaded);

    pcl::PPFRGBRegionEstimation<PointXYZRGB, Normal, PPFRGBSignature> fe;
    
    PointCloud<PointXYZRGB>::Ptr cloud_XYZRGB(new PointCloud<PointXYZRGB>());
    cloud_XYZRGB->points.clear();
    for(size_t i = 0; i < source.cloud->points.size(); ++i)               
    {
        const PointXYZ& p = source.cloud->points[i];        
        int color = *(int*)&p.data[3];
        int r =  color        & 0xFF;
        int g = (color >>  8) & 0xFF;
        int b = (color >> 16) & 0xFF;

        PointXYZRGB o;
        o.x = p.x; o.y = p.y; o.z = p.z; o.r = r; o.g = g; o.b = b;        
        cloud_XYZRGB->points.push_back(o);
    }
    cloud_XYZRGB->width = cloud_XYZRGB->points.size();
    cloud_XYZRGB->height = 1;


    fe.setInputCloud (cloud_XYZRGB);
    fe.setInputNormals (source.normals);
    fe.setIndices(source.indices);
    fe.setRadiusSearch(source.radius);
    
    PointCloud<PPFRGBSignature> ppfs;
    fe.compute (ppfs);

    for(size_t i = 0; i < downloaded.size(); ++i)
    {
        PPFRGBSignature& gpu = downloaded[i];
        PPFRGBSignature& cpu = ppfs.points[i];        

        ASSERT_NEAR(gpu.f1, cpu.f1, 0.01f);
        ASSERT_NEAR(gpu.f2, cpu.f2, 0.01f);
        ASSERT_NEAR(gpu.f3, cpu.f3, 0.01f);
        ASSERT_NEAR(gpu.f4, cpu.f4, 0.01f);
        ASSERT_NEAR(gpu.alpha_m, cpu.alpha_m, 0.01f); 

        if (pcl_isnan(gpu.r_ratio) || pcl_isnan(gpu.g_ratio) || pcl_isnan(gpu.b_ratio) || 
            pcl_isnan(cpu.r_ratio) || pcl_isnan(cpu.g_ratio) || pcl_isnan(cpu.b_ratio))
            continue;
        
        ASSERT_NEAR(gpu.r_ratio, cpu.r_ratio, 0.01f);
        ASSERT_NEAR(gpu.g_ratio, cpu.g_ratio, 0.01f);
        ASSERT_NEAR(gpu.b_ratio, cpu.b_ratio, 0.01f);     
    }
}
