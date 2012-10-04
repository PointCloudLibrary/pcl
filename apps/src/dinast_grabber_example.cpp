/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <string>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/dinast_grabber.h>

#define FPS_CALC(_WHAT_) \
static unsigned count = 0;\
static double last = pcl::getTime ();\
double now = pcl::getTime (); \
++count; \
if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \

void
savePGM (const unsigned char *image, const std::string& filename)
{
  FILE *file = fopen (filename.c_str (), "w");
  if (!file)
  {
    std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
    return;
  }

  // PGM header
  fprintf (file, "P2\n%d %d\n255\n", IMAGE_WIDTH, IMAGE_HEIGHT);

  // Write data as ASCII
  for (int i = 0; i < IMAGE_HEIGHT; ++i)
  {
    for (int j = 0; j < IMAGE_WIDTH; ++j)
    {
      fprintf (file, "%3d ", int(*image++));
    }
    fprintf (file, "\n");
  }

  fclose (file);
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* data)
{
  static int NUM_IMAGES = 0;
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    char filename[16];
    snprintf (filename, sizeof(filename), "image%.2d.pgm", NUM_IMAGES++);
    unsigned char *image = reinterpret_cast<unsigned char*> (data);
    savePGM (image, filename);
    printf ("Wrote %s\n", filename);
  }
}

int
main (int argc, char** argv) 
{

  pcl::DinastGrabber grabber;
  
  grabber.findDevice (1);
  
  grabber.openDevice();

  
  std::cerr << "Device version/revision number: " << grabber.getDeviceVersion () << "Hz"<< std::endl;
  
  grabber.start ();

  pcl::visualization::ImageViewer vis_img ("Dinast Image Viewer");
  pcl::visualization::PCLVisualizer vis_cld (argc, argv, "Dinast Cloud Viewer");

  unsigned char *image = (unsigned char*)malloc (IMAGE_SIZE);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  while (true)
  {

    grabber.getData(image, cloud);
    
    FPS_CALC ("grabber + visualization");
    
    vis_img.showMonoImage (image, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler (cloud, "intensity");
    if (!vis_cld.updatePointCloud (cloud, handler, "DinastCloud"))
    {
      vis_cld.addPointCloud (cloud, handler, "DinastCloud");
      vis_cld.resetCameraViewpoint ("DinastCloud");
    }

    vis_img.spinOnce ();
    vis_cld.spinOnce ();
    
  }
  
  grabber.stop ();
  grabber.closeDevice ();

}
