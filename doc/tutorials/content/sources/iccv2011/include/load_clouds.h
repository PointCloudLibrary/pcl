/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#pragma once

#include "typedefs.h"

#include <pcl/io/pcd_io.h>
  
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
loadPointCloud (std::string filename, std::string suffix)
{
  typename pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
  filename.append (suffix);
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr
loadPoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append ("_points.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

SurfaceNormalsPtr
loadSurfaceNormals(std::string filename)
{
  SurfaceNormalsPtr output (new SurfaceNormals);
  filename.append ("_normals.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr
loadKeypoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append ("_keypoints.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

LocalDescriptorsPtr
loadLocalDescriptors (std::string filename)
{
  LocalDescriptorsPtr output (new LocalDescriptors);
  filename.append ("_localdesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

GlobalDescriptorsPtr
loadGlobalDescriptors (std::string filename)
{
  GlobalDescriptorsPtr output (new GlobalDescriptors);
  filename.append ("_globaldesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}


#endif
