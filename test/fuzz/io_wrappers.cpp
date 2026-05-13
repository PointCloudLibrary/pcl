/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

void test_ply_reader(const std::string& filename) {
  pcl::PCLPointCloud2 cloud_blob;
  pcl::PLYReader reader;
  reader.read(filename, cloud_blob);
}
