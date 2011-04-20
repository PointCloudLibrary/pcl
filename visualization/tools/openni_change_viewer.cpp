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
 * Author: Nico Blodow (blodow@cs.tum.edu)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>

class OpenNIChangeViewer
{
  public:
    OpenNIChangeViewer (double resolution) 
      : viewer ("PCL OpenNI Viewer")
    {
      octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(resolution);
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      std::cerr << cloud->points.size() << " -- ";
     

      // assign point cloud to octree
      octree->setInputCloud (cloud);

      // add points from cloud to octree
      octree->addPointsFromInputCloud ();

      std::cerr << octree->getLeafCount() << " -- ";
      boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);

      // get a vector of new points, which did not exist in previous buffer
      octree->getPointIndicesFromNewVoxels (*newPointIdxVector, 10);

      std::cerr << newPointIdxVector->size() << std::endl;

      //// extract points from pointcloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB> (*cloud));
      //pcl::ExtractIndices<pcl::PointXYZRGB> ei;
      //ei.setInputCloud (cloud);
      //ei.setIndices (newPointIdxVector);
      //ei.filter (filtered_cloud);

      for (std::vector<int>::iterator it = newPointIdxVector->begin (); it != newPointIdxVector->end (); it++)
        filtered_cloud->points[*it].rgb = 255<<16; 
      
      if (!viewer.wasStopped())
        viewer.showCloud (filtered_cloud);
      
      // switch buffers - reset tree
      octree->switchBuffers ();
    }
    
    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = 
        boost::bind (&OpenNIChangeViewer::cloud_cb_, this, _1);

      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree;
    pcl::visualization::CloudViewer viewer;
};

int 
main (int argc, char* argv[])
{
  double resolution = 0.05;
  if (argc > 1)
    resolution = atof (argv[1]);

  OpenNIChangeViewer v (resolution);
  v.run ();
  return 0;
}


