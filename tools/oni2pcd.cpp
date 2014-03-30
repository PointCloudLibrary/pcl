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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <vector>

using namespace std;
using namespace pcl;
using namespace pcl::console;

typedef PointCloud<PointXYZRGBA> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;

int i = 0;
char buf[4096];

//////////////////////////////////////////////////////////////////////////////
void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.oni\n", argv[0]);
}

//////////////////////////////////////////////////////////////////////////////
void
cloud_cb (const CloudConstPtr& cloud)
{
  PCDWriter w;
  sprintf (buf, "frame_%06d.pcd", i);
  w.writeBinaryCompressed (buf, *cloud);
  PCL_INFO ("Wrote a cloud with %lu (%ux%u) points in %s.\n", 
            cloud->size (), cloud->width, cloud->height, buf);
  ++i;
}

/* ---[ */
int
main (int argc, char **argv)
{
  print_info ("Convert an ONI file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  pcl::ONIGrabber* grabber = new pcl::ONIGrabber (argv[1], false, false);
  boost::function<void (const CloudConstPtr&) > f = boost::bind (&cloud_cb, _1);
  boost::signals2::connection c = grabber->registerCallback (f);

  while (grabber->hasDataLeft ())
  {
    grabber->start ();
  }
  PCL_INFO ("Successfully processed %d frames.\n", i);

  delete grabber;
  return (0);
}
