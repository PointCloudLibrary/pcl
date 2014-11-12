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


#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::Normal;
using pcl::PointXYZRGBNormal;

using pcl::PCDReader;
using pcl::PassThrough;
using pcl::NormalEstimation;
using pcl::KdTreeFLANN;

int 
  main (int argc, char **argv)
{
  srand (time (0));

  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

  PCDReader pcd;
  if (pcd.read (argv[1], *cloud) == -1)
    return (-1);

  // Filter the data
  std::cerr << "Filtering..." << std::endl;
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
  pass.filter (*cloud_filtered);

  // Estimate surface normals
  std::cerr << "Estimating normals..." << std::endl;
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (20);
  KdTreeFLANN<PointXYZRGB>::Ptr tree (new KdTreeFLANN<PointXYZRGB>);
  ne.setSearchMethod (tree);
  PointCloud<Normal> normals;
  ne.compute (normals);

  // Concatenate points and normals
  PointCloud<PointXYZRGBNormal> cloud_normals;
  pcl::concatenateFields (cloud_filtered, normals, cloud_normals);

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("test");
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (0.1);

  pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler (cloud_normals);
  // Geometry handler demo
  {
    std::cerr << "PointCloudGeometryHandlerSurfaceNormal demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointXYZRGBNormal> geometry_handler (cloud_normals);
    p.addPointCloud (cloud_normals, geometry_handler, "cloud_normal");
    p.spin ();
    p.removePointCloud ("cloud_normal");

    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_normal");
    p.spin ();
    p.removePointCloud ("cloud_normal");
  }

  {
    std::cerr << "PointCloudGeometryHandlerXYZ demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGBNormal> geometry_handler (cloud_normals);
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
  }

  {
    std::cerr << "PointCloudGeometryHandlerXYZ demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerCustom<PointXYZRGBNormal> geometry_handler (cloud_normals, "x", "y", "z");
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
    
    geometry_handler = pcl::visualization::PointCloudGeometryHandlerCustom<PointXYZRGBNormal> (cloud_normals, "x", "x", "y");
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
   }
  p.spin ();
}
