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


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using pcl::PointCloud;
using pcl::PointXYZ;

int 
main (int , char **)
{
  srand (unsigned (time (0)));

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  cloud->points.resize (5);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = float (i); 
    cloud->points[i].y = float (i / 2);
    cloud->points[i].z = 0.0f;
  }

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("test_shapes");
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (1.0, "first");

  //p.addPolygon (cloud, "polygon");
  p.addPolygon<PointXYZ> (cloud, 1.0, 0.0, 0.0, "polygon", 0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
  
  p.addLine<PointXYZ, PointXYZ> (cloud->points[0], cloud->points[1], 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

  p.addSphere<PointXYZ> (cloud->points[0], 1, 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere");
//  p.removePolygon ("poly");

  p.addText ("text", 200, 200, 1.0, 0, 0, "text");
  
  p.addText3D ("text3D", cloud->points[0], 1.0, 1.0, 0.0, 0.0);
  p.spin ();
  p.removeCoordinateSystem ("first", 0);
  p.spin ();
  p.addCoordinateSystem (1.0, 5, 3, 1, "second");
  p.spin ();
  p.removeCoordinateSystem ("second", 0);
  p.spin ();
}
