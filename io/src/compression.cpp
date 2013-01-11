

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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/compression/entropy_range_coder.h>
#include <pcl/compression/impl/entropy_range_coder.hpp>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/impl/octree_pointcloud_compression.hpp>

template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZ>;
template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>;
template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>;

#ifdef HAVE_PNG
#ifdef HAVE_OPENNI
#include <pcl/compression/organized_pointcloud_compression.h>
#include <pcl/compression/impl/organized_pointcloud_compression.hpp>

template class PCL_EXPORTS pcl::io::OrganizedPointCloudCompression<pcl::PointXYZ>;
template class PCL_EXPORTS pcl::io::OrganizedPointCloudCompression<pcl::PointXYZRGB>;
template class PCL_EXPORTS pcl::io::OrganizedPointCloudCompression<pcl::PointXYZRGBA>;
#endif //HAVE_OPENNI
#endif //HAVE_PNG

