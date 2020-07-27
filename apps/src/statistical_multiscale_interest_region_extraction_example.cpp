/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *
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
 */

#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;

const float subsampling_leaf_size = 0.003f;
const float base_scale = 0.005f;

int
main(int, char** argv)
{
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());

  PCDReader reader;
  reader.read(argv[1], *cloud);
  PCL_INFO("Cloud read: %s\n", argv[1]);
  std::cerr << "cloud has #points: " << cloud->size() << std::endl;

  PointCloud<PointXYZ>::Ptr cloud_subsampled(new PointCloud<PointXYZ>());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(
      subsampling_leaf_size, subsampling_leaf_size, subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);
  std::cerr << "subsampled cloud has #points: " << cloud_subsampled->size()
            << std::endl;

  StatisticalMultiscaleInterestRegionExtraction<PointXYZ> region_extraction;
  std::vector<float> scale_vector;
  PCL_INFO("Scale values that will be used: ");
  float base_scale_aux = base_scale;
  for (std::size_t scales = 0; scales < 7; ++scales) {
    PCL_INFO("%f ", base_scale_aux);
    scale_vector.push_back(base_scale_aux);
    base_scale_aux *= 1.6f;
  }
  PCL_INFO("\n");
  region_extraction.setInputCloud(cloud_subsampled);
  region_extraction.setScalesVector(scale_vector);
  std::list<IndicesPtr> rois;
  region_extraction.computeRegionsOfInterest(rois);

  PCL_INFO("Regions of interest found: %d\n", rois.size());
  pcl::ExtractIndices<PointXYZ> extract_indices_filter;
  unsigned int roi_count = 0;
  for (const auto& roi : rois) {
    PointCloud<PointXYZ> roi_points;
    extract_indices_filter.setInputCloud(cloud_subsampled);
    extract_indices_filter.setIndices(roi);
    extract_indices_filter.filter(roi_points);

    char filename[512];
    sprintf(filename, "roi_%03d.pcd", ++roi_count);
    io::savePCDFileASCII(filename, roi_points);
  }

  io::savePCDFileASCII("subsampled_input.pcd", *cloud_subsampled);

  return 0;
}
