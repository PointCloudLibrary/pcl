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
#include <pcl/features/spin_image.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/common/time.h>

#include "data_source.hpp"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;

typedef pcl::Histogram<153> SpinImage;

//TEST(PCL_FeaturesGPU, DISABLED_spinImages_rectangular)
TEST(PCL_FeaturesGPU, spinImages_rectangular)
{   
    DataSource source;
    	
    source.estimateNormals();
    source.generateIndices(5);
	//source.indices->resize(1);
	source.radius *= 2;

	const int min_beighbours = 15;
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    
    //uploading data to GPU
    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);
    	
	//////////// GPU ////////////
	gpu::SpinImageEstimation se_gpu;        
    se_gpu.setInputWithNormals(cloud_gpu, normals_gpu);
    se_gpu.setIndices(indices_gpu);
    se_gpu.setRadiusSearch(source.radius, source.max_elements*10);
    se_gpu.setMinPointCountInNeighbourhood(min_beighbours);
	//se_gpu.setRadialStructure();
	//se_gpu.setAngularDomain();	

	DeviceArray2D<SpinImage> spin_images_device;
	DeviceArray<unsigned char> mask_device;
    
    {
        ScopeTime up("gpu");
        se_gpu.compute(spin_images_device, mask_device);
    }

	int c;
    vector<SpinImage> downloaded;
	vector<unsigned char> downloaded_mask;
    spin_images_device.download(downloaded, c);
	mask_device.download(downloaded_mask);

	//////////// CPU ////////////
	pcl::SpinImageEstimation<PointXYZ, Normal, SpinImage> se(8, 0.0, 16);   
	se.setInputCloud (source.cloud);
	se.setInputNormals (source.normals);
	se.setIndices (source.indices);	
	se.setRadiusSearch (source.radius);    
	se.setMinPointCountInNeighbourhood(min_beighbours);
	//se.setRadialStructure();
	//se.setAngularDomain();	
	  
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage> ());
	{
        ScopeTime up("cpu");
		se.compute (*spin_images);
	}
	
    for(size_t i = 0; i < downloaded.size(); ++i)
    {
		if(!downloaded_mask[i]) // less than min neighbours, so spinimage wasn't computed
			continue;  

        SpinImage& gpu = downloaded[i];
        SpinImage& cpu = spin_images->points[i];
        
        size_t FSize = sizeof(SpinImage)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);        
        }        

        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_spinImages_radial)
TEST(PCL_FeaturesGPU, spinImages_radial)
{   
    DataSource source;
    	
    source.estimateNormals();
    source.generateIndices(5);
	//source.indices->resize(1);
	source.radius *= 2;

	const int min_beighbours = 15;
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    

    //uploading data to GPU
    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);
    	
	//////////// GPU ////////////
	gpu::SpinImageEstimation se_gpu;        
    se_gpu.setInputWithNormals(cloud_gpu, normals_gpu);
    se_gpu.setIndices(indices_gpu);
    se_gpu.setRadiusSearch(source.radius, source.max_elements*10);
    se_gpu.setMinPointCountInNeighbourhood(min_beighbours);
	se_gpu.setRadialStructure();
	//se_gpu.setAngularDomain();	

	DeviceArray2D<SpinImage> spin_images_device;
	DeviceArray<unsigned char> mask_device;
    
    {
        ScopeTime up("gpu");
        se_gpu.compute(spin_images_device, mask_device);
    }

	int c;
    vector<SpinImage> downloaded;
	vector<unsigned char> downloaded_mask;
    spin_images_device.download(downloaded, c);
	mask_device.download(downloaded_mask);

	//////////// CPU ////////////
	pcl::SpinImageEstimation<PointXYZ, Normal, SpinImage> se(8, 0.0, 16);   
	se.setInputCloud (source.cloud);
	se.setInputNormals (source.normals);
	se.setIndices (source.indices);	
	se.setRadiusSearch (source.radius);    
	se.setMinPointCountInNeighbourhood(min_beighbours);
	se.setRadialStructure();
	//se.setAngularDomain();	
	  
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage> ());
	{
        ScopeTime up("cpu");
		se.compute (*spin_images);
	}
	
    for(size_t i = 0; i < downloaded.size(); ++i)
    {
		if(!downloaded_mask[i]) // less than min neighbours, so spinimage wasn't computed
			continue;  

        SpinImage& gpu = downloaded[i];
        SpinImage& cpu = spin_images->points[i];
        
        size_t FSize = sizeof(SpinImage)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);        
        }        

        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_spinImages_rectangular_angular)
TEST(PCL_FeaturesGPU, spinImages_rectangular_angular)
{   
    DataSource source;
    	
    source.estimateNormals();
    source.generateIndices(5);
	//source.indices->resize(1);
	source.radius *= 2;

	const int min_beighbours = 15;
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    

    //uploading data to GPU
    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);
    	
	//////////// GPU ////////////
	gpu::SpinImageEstimation se_gpu;        
    se_gpu.setInputWithNormals(cloud_gpu, normals_gpu);
    se_gpu.setIndices(indices_gpu);
    se_gpu.setRadiusSearch(source.radius, source.max_elements*10);
    se_gpu.setMinPointCountInNeighbourhood(min_beighbours);
	//se_gpu.setRadialStructure();
	se_gpu.setAngularDomain();

	DeviceArray2D<SpinImage> spin_images_device;
	DeviceArray<unsigned char> mask_device;
    
    {
        ScopeTime up("gpu");
        se_gpu.compute(spin_images_device, mask_device);
    }

	int c;
    vector<SpinImage> downloaded;
	vector<unsigned char> downloaded_mask;
    spin_images_device.download(downloaded, c);
	mask_device.download(downloaded_mask);

	//////////// CPU ////////////
	pcl::SpinImageEstimation<PointXYZ, Normal, SpinImage> se(8, 0.0, 16);   
	se.setInputCloud (source.cloud);
	se.setInputNormals (source.normals);
	se.setIndices (source.indices);	
	se.setRadiusSearch (source.radius);    
	se.setMinPointCountInNeighbourhood(min_beighbours);
	//se.setRadialStructure();
	se.setAngularDomain();
	  
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage> ());
	{
        ScopeTime up("cpu");
		se.compute (*spin_images);
	}
	
    for(size_t i = 0; i < downloaded.size(); ++i)
    {
		if(!downloaded_mask[i]) // less than min neighbours, so spinimage wasn't computed
			continue;  

        SpinImage& gpu = downloaded[i];
        SpinImage& cpu = spin_images->points[i];
        
        size_t FSize = sizeof(SpinImage)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);        
        }        

        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.02f/FSize);
    }
}


//TEST(PCL_FeaturesGPU, DISABLED_spinImages_radial_angular)
TEST(PCL_FeaturesGPU, spinImages_radial_angular)
{   
    DataSource source;
    	
    source.estimateNormals();
    source.generateIndices(5);
	//source.indices->resize(1);
	source.radius *= 2;

	const int min_beighbours = 15;
                   
    vector<PointXYZ> normals_for_gpu(source.normals->points.size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());        
    

    //uploading data to GPU
    pcl::gpu::VFHEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::VFHEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    pcl::gpu::VFHEstimation::Indices indices_gpu;
    indices_gpu.upload(*source.indices);
    	
	//////////// GPU ////////////
	gpu::SpinImageEstimation se_gpu;        
    se_gpu.setInputWithNormals(cloud_gpu, normals_gpu);
    se_gpu.setIndices(indices_gpu);
    se_gpu.setRadiusSearch(source.radius, source.max_elements*10);
    se_gpu.setMinPointCountInNeighbourhood(min_beighbours);
	se_gpu.setRadialStructure();
	se_gpu.setAngularDomain();

	DeviceArray2D<SpinImage> spin_images_device;
	DeviceArray<unsigned char> mask_device;
    
    {
        ScopeTime up("gpu");
        se_gpu.compute(spin_images_device, mask_device);
    }

	int c;
    vector<SpinImage> downloaded;
	vector<unsigned char> downloaded_mask;
    spin_images_device.download(downloaded, c);
	mask_device.download(downloaded_mask);

	//////////// CPU ////////////
	pcl::SpinImageEstimation<PointXYZ, Normal, SpinImage> se(8, 0.0, 16);   
	se.setInputCloud (source.cloud);
	se.setInputNormals (source.normals);
	se.setIndices (source.indices);	
	se.setRadiusSearch (source.radius);    
	se.setMinPointCountInNeighbourhood(min_beighbours);
	se.setRadialStructure();
	se.setAngularDomain();
	  
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage> ());
	{
        ScopeTime up("cpu");
		se.compute (*spin_images);
	}
	
    for(size_t i = 0; i < downloaded.size(); ++i)
    {
		if(!downloaded_mask[i]) // less than min neighbours, so spinimage wasn't computed
			continue;  

        SpinImage& gpu = downloaded[i];
        SpinImage& cpu = spin_images->points[i];
        
        size_t FSize = sizeof(SpinImage)/sizeof(gpu.histogram[0]);                                
        
        float norm = 0, norm_diff = 0;
        for(size_t j = 0; j < FSize; ++j)
        {
            norm_diff += (gpu.histogram[j] - cpu.histogram[j]) * (gpu.histogram[j] - cpu.histogram[j]);
            norm += cpu.histogram[j] * cpu.histogram[j];

            //ASSERT_NEAR(gpu.histogram[j], cpu.histogram[j], 0.03f);        
        }        

        if (norm != 0)
            ASSERT_LE(norm_diff/norm, 0.01f/FSize);
    }
}
