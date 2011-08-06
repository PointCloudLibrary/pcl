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

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/device_array.hpp"
#include "pcl/gpu/common/timers_opencv.hpp"


namespace pcl
{
PointCloud<PointXYZ> cloud, cloud_big;
void
  init ()
{
  float resolution = 0.1;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (PointXYZ (x, y, z));
  cloud.width  = cloud.points.size ();
  cloud.height = 1;

  cloud_big.width  = 640;
  cloud_big.height = 480;
  srand (time (NULL));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (PointXYZ (1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0)));
}




using namespace std;
using namespace pcl::gpu;

	template <typename PointT> int
		OctreeGPU<PointT>::	    radiusSearchGPU (std::vector<const PointT>& point, const double radius, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_distances, int max_nn ) const
		{

								
  // pcl::gpu::Octree octree; 

		};



}

#define PCL_INSTANTIATE_OctreeGPU(T) template class PCL_EXPORTS pcl::OctreeGPU<T>;


