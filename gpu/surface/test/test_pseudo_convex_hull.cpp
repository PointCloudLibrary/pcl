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

#if defined _MSC_VER
  #pragma warning (disable : 4996 4530)
#endif

#include <gtest/gtest.h>

#include<iostream>
#include<fstream>
#include<algorithm>

#if defined _MSC_VER
  #pragma warning (disable: 4521)
#endif

#include <pcl/point_cloud.h>

#if defined _MSC_VER
  #pragma warning (default: 4521)
#endif


#include <pcl/gpu/surface/convex_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

using namespace pcl::gpu;
using namespace std;


int loadCloud(const std::string& file, pcl::PointCloud<pcl::PointXYZ>& cloud)
{  
  int result = pcl::io::loadPCDFile(file, cloud);
  if (result != -1)
      return result;

  string name = file.substr(0, file.find_last_of("."));
      
  result = pcl::io::loadPCDFile(name + ".pcd", cloud);
  if (result != -1)
    return result;
    
  result = pcl::io::loadPLYFile(name + ".ply", cloud);
  if (result != -1)
    pcl::io::savePCDFile(name + ".pcd", cloud, true);
  return result;  
};

//TEST(PCL_SurfaceGPU, DISABGLED_pseudoConvexHull)
TEST(PCL_SurfaceGPU, pseudoConvexHull)
{
  PseudoConvexHull3D pch(1e5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr( new pcl::PointCloud<pcl::PointXYZ>() );    
  //ASSERT_TRUE(loadCloud("d:/dragon.ply", *cloud_ptr) != -1);
  //ASSERT_TRUE(loadCloud("d:/horse.ply", *cloud_ptr) != -1);
  ASSERT_TRUE(loadCloud("d:/Ramesses.ply", *cloud_ptr) != -1);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr convex_ptr;


  PseudoConvexHull3D::Cloud cloud_device;
  PseudoConvexHull3D::Cloud convex_device;
  cloud_device.upload(cloud_ptr->points);

  pcl::PolygonMesh mesh;

  {
    pcl::ScopeTime time("cpu"); 
    pcl::ConvexHull<pcl::PointXYZ> ch;
    ch.setInputCloud(cloud_ptr); 
    ch.reconstruct(mesh);
  }

  {
    pcl::ScopeTime time("gpu+cpu"); 
    { 
      pcl::ScopeTime time("gpu"); 
      pch.reconstruct(cloud_device, convex_device);
    }

    convex_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>((int)convex_device.size(), 1));
    convex_device.download(convex_ptr->points);

    pcl::ConvexHull<pcl::PointXYZ> ch;
    ch.setInputCloud(convex_ptr);    
    ch.reconstruct(mesh);
  }
    
  pcl::visualization::PCLVisualizer viewer("Viewer");
  viewer.addPointCloud<pcl::PointXYZ> (cloud_ptr, "sample cloud");

  viewer.spinOnce(1, true);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(convex_ptr, 0, 255, 0);

  viewer.addPointCloud<pcl::PointXYZ> (convex_ptr, single_color, "convex");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex");


  viewer.addPolylineFromPolygonMesh(mesh);

  viewer.spin();    
}
