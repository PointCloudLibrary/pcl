/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/* \author Bastian Steder */

/* ---[ */


#include <iostream>
using namespace std;

#include <pcl/visualization/range_image_visualizer.h>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>

using namespace pcl;

typedef PointWithViewpoint PointType;

int 
  main (int argc, char** argv) 
{
  pcl::PointCloud<PointType> point_cloud;
  
  bool use_point_cloud_from_file = argc >= 2;
  if (use_point_cloud_from_file) 
  {
    string fileName = argv[1];
    PCL_INFO ("Trying to open %s...", fileName.c_str ());
    if (pcl::io::loadPCDFile (fileName, point_cloud) == -1) 
    {
      PCL_ERROR ("Was not able to open file %s!", fileName.c_str ());
      use_point_cloud_from_file = false;
    }
  }
  else 
  {
    PCL_INFO ("\n\nUsage: %s <file.pcl> <angular resolution (default 0.5)> <coordinate frame (default 0)>\n", argv[0]);
  }
  
  if (!use_point_cloud_from_file) 
  {
    PCL_INFO ("No file given or could not read file => Genarating example point cloud.");
    for (float y=-0.5f; y<=0.5f; y+=0.01f) 
    {
      for (float z=-0.5f; z<=0.5f; z+=0.01f) 
      {
        PointType point;
        point.x = 2.0f - y;
        point.y = y;
        point.z = z;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = point_cloud.points.size ();
    point_cloud.height = 1;
  }

  float angular_resolution = DEG2RAD (0.5f);
  if (argc >= 3) 
  {
    float tmp_angular_resolution = strtod (argv[2], NULL);
    if (tmp_angular_resolution >= 0.1f)  // Check for too small numbers (or we might get a memory problem)
    {
      PCL_INFO ("Using angular resolution %f deg from command line.", tmp_angular_resolution); 
      angular_resolution = DEG2RAD(tmp_angular_resolution);
    }
  }
  
  RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
  if (argc >= 4) 
  {
    coordinate_frame = (RangeImage::CoordinateFrame)strtol (argv[3], NULL, 0);
    cout << "Using coordinate frame "<<coordinate_frame<<".\n";
  }
  
  // We now want to create a range image from the above point cloud, with a 1° angular resolution
  float max_angle_width  = DEG2RAD(360.0f);  // 360.0 degree in radians
  float max_angle_height = DEG2RAD(180.0f);  // 180.0 degree in radians
  float noise_level=0.0f;
  float min_range = 0.0f;
  int border_size = 2;
  
  RangeImage range_image;
  //Eigen::Transform3f sensor_pose = (Eigen::Transform3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  //range_image.createFromPointCloud(point_cloud, angular_resolution, max_angle_width, max_angle_height, sensor_pose, coordinate_frame,
                                   //noise_level, min_range, border_size);
  range_image.createFromPointCloudWithViewpoints(point_cloud, angular_resolution, max_angle_width, max_angle_height,
                                                 coordinate_frame, noise_level, min_range, border_size);
  
  cout << PVARN(range_image);
  
  pcl_visualization::RangeImageVisualizer range_image_widget ("Range Image");
  range_image_widget.setRangeImage (range_image);
  //range_image_widget.markPoint (range_image.width/2, range_image.height/2, wxRED_PEN);
  //range_image_widget.markLine (0.0f, 0.0f, range_image.width/2, range_image.height/2, wxRED_PEN);
  
  pcl_visualization::RangeImageVisualizer::spin ();  // process GUI events
  
  // OR, if you want your own main loop instead:
  //while(range_imageWidget.isShown()) {
    //RangeImageVisualizer::spinOnce();  // process GUI events
    //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  //}
  
  wxEntryCleanup();  // Free memory allocated by wxEntryStart
  
  return (0);
}

/* ]--- */
