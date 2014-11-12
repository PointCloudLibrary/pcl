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

typedef pcl::PointXYZRGB Point;

int 
  main (int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Needs a PCD file as input." << std::endl;
    return (-1);
  }

  srand (time (0));

  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);

  pcl::PCDReader pcd;
  if (pcd.read (argv[1], *cloud) == -1)
    return (-1);

  pcl::visualization::PCLVisualizer p ("test");
  p.setBackgroundColor (1, 1, 1);

  // Handler random color demo
  {
    std::cerr << "PointCloudColorHandlerRandom demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerRandom<Point> handler (cloud);
    
    p.addPointCloud<Point> (cloud, "cloud_random");      // no need to add the handler, we use a random handler by default
    p.spin ();
    p.removePointCloud ("cloud_random");

    p.addPointCloud (cloud, handler, "cloud_random");
    p.spin ();
    p.removePointCloud ("cloud_random");
  }

  // Handler custom demo
  {
    std::cerr << "PointCloudColorHandlerCustom demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<Point> handler (cloud, 255, 0, 0);
    
    p.addPointCloud (cloud, handler);             // the default id is "cloud"
    p.spin ();
    p.removePointCloud ();                        // the default id is "cloud"

    handler = pcl::visualization::PointCloudColorHandlerCustom<Point> (cloud, 255, 0, 0);
    p.addPointCloud (cloud, handler, "cloud");
    p.spin ();
    p.removePointCloud ("cloud");
  }

  // Handler RGB demo
  {
    std::cerr << "PointCloudColorHandlerRGBField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<Point> handler (cloud);

    p.addPointCloud (cloud, handler, "cloud_rgb");
    p.spin ();
    p.removePointCloud ("cloud_rgb");
   }
  
  // Handler generic field demo
  {
    std::cerr << "PointCloudColorHandlerGenericField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_z (cloud, "z");
    pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_x (cloud, "x");
    
    p.addPointCloud (cloud, handler_x, "cloud_x");
    p.spin ();
    p.removePointCloud ("cloud_x");
    
    p.addPointCloud (cloud, handler_z, "cloud_z");
    p.spin ();
    p.removePointCloud ("cloud_z");
  }

  p.addCoordinateSystem (0.1);
  
  // Demonstrate usage of spinOnce()
  p.resetStoppedFlag();
  while (!p.wasStopped())
  {
    static int counter = 0;
    cout << "spinOnce was called "<<++counter<<" times.\n";
    p.spinOnce(1000);  // Give the GUI 1000ms to handle events, then return
  }

  //p.removePointCloud ("cloud");
  //p.spin ();
}
