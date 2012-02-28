/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <boost/make_shared.hpp>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointT;

class OpenNIOrganizedMultiPlaneSegmentation
{
  private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<PointT>::ConstPtr prev_cloud;
    boost::mutex cloud_mutex;

  public:
    OpenNIOrganizedMultiPlaneSegmentation ()
    {

    }
    ~OpenNIOrganizedMultiPlaneSegmentation ()
    {
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
    {
      boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
      viewer->addPointCloud < PointT > (cloud, single_color, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      return (viewer);
    }

    void
    cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!viewer->wasStopped ())
      {
        cloud_mutex.lock ();
        prev_cloud = cloud;
        cloud_mutex.unlock ();
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void
      (const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&OpenNIOrganizedMultiPlaneSegmentation::cloud_cb_, this, _1);

      //make a viewer
      pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);
      viewer = cloudViewer (init_cloud_ptr);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();

      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

      while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);

        if (prev_cloud && cloud_mutex.try_lock ())
        {
          printf ("Computing normals...\n");
          pcl::PointCloud < pcl::Normal > normal_cloud;
          double normal_start = pcl::getTime ();
          pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
          ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
          ne.setMaxDepthChangeFactor (0.02f);
          ne.setNormalSmoothingSize (10.0f);
          ne.setInputCloud (prev_cloud);
          ne.compute (normal_cloud);
          double normal_end = pcl::getTime ();
          std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

          printf ("Extracting planes...\n");
          std::vector < pcl::ModelCoefficients > plane_models;
          std::vector < pcl::PointIndices > planes_inliers;
          pcl::PointCloud<pcl::Label> labels;
          std::vector<pcl::PointIndices> label_indices;
          double plane_extract_start = pcl::getTime ();
          pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
          mps.setMinInliers (10000);
          mps.setAngularThreshold (0.017453 * 2.0); //3 degrees
          mps.setDistanceThreshold (0.02); //2cm
          mps.setInputNormals (boost::make_shared < pcl::PointCloud<pcl::Normal> > (normal_cloud));
          mps.setInputCloud (prev_cloud);
          mps.segment (plane_models, planes_inliers,labels,label_indices);
          double plane_extract_end = pcl::getTime ();
          std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
          std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;

          char name [1024];
          pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
          viewer->removeAllPointClouds(0);
          viewer->removeAllShapes ();
          if(!viewer->updatePointCloud < PointT > (boost::make_shared < pcl::PointCloud<PointT> > (*prev_cloud),"cloud")){
            viewer->addPointCloud < PointT > (boost::make_shared < pcl::PointCloud<PointT> > (*prev_cloud),"cloud");
          }

          //Draw Visualization
          for (size_t i = 0; i < planes_inliers.size (); i++)
          {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid (*prev_cloud, planes_inliers[i], centroid);
            pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
            pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5 * plane_models[i].values[0]),
                                               centroid[1] + (0.5 * plane_models[i].values[1]),
                                               centroid[2] + (0.5 * plane_models[i].values[2]));
            sprintf (name, "normal_%d", (unsigned)i);
            viewer->addArrow (pt2, pt1, 1.0, 0, 0, name);

            pcl::copyPointCloud (*prev_cloud,planes_inliers[i],*cluster);
            sprintf (name, "plane_%02d", (int)i);
            pcl::visualization::PointCloudColorHandlerCustom <PointT> color (cluster, red[i], grn[i], blu[i]);
            viewer->addPointCloud (cluster, color, name);
          
          }
          cloud_mutex.unlock ();
        }
      }

      interface->stop ();
    }
};

int
main ()
{
  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;
  multi_plane_app.run ();
  return 0;
}
