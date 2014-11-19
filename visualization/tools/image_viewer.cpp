/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <boost/thread/thread.hpp>
#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>

using namespace std;

int
main (int, char ** argv)
{
  pcl::PCDReader reader;
  pcl::PCLPointCloud2 cloud;
  reader.read (argv[1], cloud);
  
  pcl::PointCloud<pcl::PointXYZ> xyz;
  pcl::fromPCLPointCloud2 (cloud, xyz);
  
  pcl::visualization::ImageViewer depth_image_viewer_;
  float* img = new float[cloud.width * cloud.height];

  for (int i = 0; i < static_cast<int> (xyz.points.size ()); ++i)
    img[i] = xyz.points[i].z;
  
  depth_image_viewer_.showFloatImage (img, 
                                      cloud.width, cloud.height,
                                      std::numeric_limits<float>::min (), 
                                      // Scale so that the colors look brigher on screen
                                      std::numeric_limits<float>::max () / 10, 
                                      true);
  depth_image_viewer_.spin ();
  return (0);
}
