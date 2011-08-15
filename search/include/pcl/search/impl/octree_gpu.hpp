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
 * Author: Siddharth Choudhary (itzsid@gmail.com)
 */

#include <iostream>
#include <fstream>
#include <numeric>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/search/octree_gpu.h>

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/device_array.hpp"
#include "pcl/gpu/common/timers_opencv.hpp"

#ifndef PCL_SEARCH_IMPL_OCTREE_GPU_H_
#define PCL_SEARCH_IMPL_OCTREE_GPU_H_

namespace pcl
{
typedef pcl::gpu::Octree::PointType PointType;
std::vector<PointType> cloud_gpu;




using namespace std;
using namespace pcl::gpu;

template <typename PointT> void
OctreeGPU<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{
	for(int i=0;i<cloud->points.size();i++)
	{
		cloud_gpu.push_back(PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));

	}
	

}

	template <typename PointT> int
		OctreeGPU<PointT>::	    radiusSearchGPU (std::vector<PointT>& point, std::vector < double >&radiuses, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_distances, int max_nn ) const

		{
    const int max_answers = 333;

    std::vector<PointType> pointT;

	std:: vector < std:: vector < float > > query_vec;
	
		std::vector<float> temp;
	for(int i=0;i<point.size();i++)
	{
		temp.clear();
		temp.push_back(point[i].x);
		temp.push_back(point[i].y);
		temp.push_back(point[i].z);
		query_vec.push_back(temp);
	//	std::cout << "check" << std::endl;
	}
	pointT.resize(query_vec.size());
	for(int i=0;i<query_vec.size();i++)
	{

		pointT[i].x = query_vec[i][0];
		pointT[i].y = query_vec[i][1];
		pointT[i].z = query_vec[i][2];
	}
    
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud_gpu);

    //gpu build 
    pcl::gpu::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();

    //upload queries
    pcl::gpu::Octree::BatchQueries queries_device;
    pcl::gpu::Octree::BatchRadiuses radiuses_device;
    queries_device.upload(pointT);

    std::vector < float > radius_queries;
    for(int i=0;i<radiuses.size();i++)
	{
		radius_queries.push_back((float)radiuses[i]);
	}
    radiuses_device.upload(radius_queries);

    //prepare output buffers on device
    pcl::gpu::Octree::BatchResult      result_device1(queries_device.size() * max_answers);
    pcl::gpu::Octree::BatchResultSizes  sizes_device1(queries_device.size());

    //prepare output buffers on host


    //search GPU individual
    octree_device.radiusSearchBatch(queries_device, radiuses_device, max_answers, result_device1, sizes_device1);


    //download results
    vector<int> sizes1;
    sizes_device1.download(sizes1);

    vector<int> downloaded_buffer1; 
    result_device1.download(downloaded_buffer1);

    //fill in k_indices and k_distances


return 0;

		};




}
#define PCL_INSTANTIATE_OctreeGPU(T) template class PCL_EXPORTS pcl::OctreeGPU<T>;

#endif
