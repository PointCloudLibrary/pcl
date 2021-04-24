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
#include <pcl/features/principal_curvatures.h>
#include <pcl/gpu/features/features.hpp>

#include "data_source.hpp"

using namespace pcl;
using namespace pcl::gpu;

//TEST(PCL_FeaturesGPU, DISABLED_PrincipalCurvatures)
TEST(PCL_FeaturesGPU, PrincipalCurvatures)
{   
    DataSource source;
    
    source.estimateNormals();
                   
    std::vector<PointXYZ> normals_for_gpu(source.normals->size());    
    std::transform(source.normals->points.begin(), source.normals->points.end(), normals_for_gpu.begin(), DataSource::Normal2PointXYZ());
    
    //uploading data to GPU

    //////////////////////////////////////////////////////////////////////////////////////////////  

    pcl::gpu::PrincipalCurvaturesEstimation::PointCloud cloud_gpu;
    cloud_gpu.upload(source.cloud->points);

    pcl::gpu::PrincipalCurvaturesEstimation::Normals normals_gpu;
    normals_gpu.upload(normals_for_gpu);             

    DeviceArray<PrincipalCurvatures> pc_features;

    gpu::PrincipalCurvaturesEstimation pc_gpu;    
    pc_gpu.setInputCloud(cloud_gpu);
    pc_gpu.setInputNormals(normals_gpu);
    pc_gpu.setRadiusSearch(source.radius, source.max_elements);
    pc_gpu.compute(pc_features);

    std::vector<PrincipalCurvatures> downloaded;
    pc_features.download(downloaded);

    pcl::PrincipalCurvaturesEstimation<PointXYZ, Normal, PrincipalCurvatures> fe;
    fe.setInputCloud (source.cloud);
    fe.setInputNormals (source.normals);
    fe.setRadiusSearch(source.radius);

    PointCloud<PrincipalCurvatures> pc;
    fe.compute (pc);

    for(std::size_t i = 0; i < downloaded.size(); ++i)
    {
        PrincipalCurvatures& gpu = downloaded[i];
        PrincipalCurvatures& cpu = pc[i];        

        ASSERT_NEAR(gpu.principal_curvature_x, cpu.principal_curvature_x, 0.01f);
        ASSERT_NEAR(gpu.principal_curvature_y, cpu.principal_curvature_y, 0.01f);
        ASSERT_NEAR(gpu.principal_curvature_z, cpu.principal_curvature_z, 0.01f);
        ASSERT_NEAR(gpu.pc1, cpu.pc1, 0.01f);
        ASSERT_NEAR(gpu.pc2, cpu.pc2, 0.01f);        
    }
}