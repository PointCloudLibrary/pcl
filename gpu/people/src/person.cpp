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
 * @author: Koen Buys
 */

#ifndef PCL_GPU_PEOPLE_PERSON_HPP_
#define PCL_GPU_PEOPLE_PERSON_HPP_

#include <pcl/gpu/people/person.h>
#include <pcl/gpu/utils/repacks.hpp>

void
pcl::gpu::people::Person::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  cloud_device_.upload(cloud->points);
  pcl::gpu::copyFieldsZ(cloud_device_, float_depth_device_);
  //TODO: convert float_depth_device_ to depth_device_ here

  //TODO: create version of process that takes deviceArray as input and output
  //multi_tree_live_proc_.process(depth_device_, label_device_);

  //TODO: make sure that smoothLabelImage can work with in and out beeing the same array
  //pcl::gpu::smoothLabelImage(label_device_, depth_device_, label_device_);

  //TODO: create copyFields to take labels from one array and XYZRGB from other array
  //pcl::gpu::copyFields(label_device_, labeled_device_);
  //pcl::gpu::copyFields(cloud_device_, labeled_device_):

  //TODO: bring back to CPU space here

  //Resume normal operation from here:
/*
  // Make all the clusters
  optimized_elec(cloud_in, lmap, CLUST_TOL, cluster_indices, AREA_THRES, MAX_CLUST_SIZE, NUM_PARTS, false, 1.f);

  // Create a new struct to put the results in
  std::vector<std::vector<pcl::gpu::people::label_skeleton::Blob2, Eigen::aligned_allocator<pcl::gpu::people::label_skeleton::Blob2> > >       sorted;
  //clear out our matrix before starting again with it
  sorted.clear();
  //Set fixed size of outer vector length = number of parts
  sorted.resize(NUM_PARTS);

  //create the blob2 matrix
  pcl::gpu::people::label_skeleton::sortIndicesToBlob2 ( cloud_labels, AREA_THRES, sorted, cluster_indices );
  //Build relationships between the blobs
  pcl::gpu::people::label_skeleton::buildRelations ( sorted );
*/
}

#endif // PCL_GPU_PEOPLE_PERSON_HPP_
