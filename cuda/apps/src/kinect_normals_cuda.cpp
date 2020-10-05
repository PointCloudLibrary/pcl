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
 * $Id$
 *
 */

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <functional>
#include <iostream>
#include <mutex>


using namespace pcl::cuda;

class NormalEstimation
{
  public:
    NormalEstimation () : viewer ("PCL CUDA - NormalEstimation"), new_cloud(false), go_on(true) {}

    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      std::string cloud_name ("cloud");
      std::lock_guard<std::mutex> l(m_mutex);
      if (new_cloud)
      {
        //using ColorHandler = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>;
        using ColorHandler = pcl::visualization::PointCloudColorHandlerGenericField <pcl::PointXYZRGBNormal>;
        //ColorHandler Color_handler (normal_cloud);
        ColorHandler Color_handler (normal_cloud,"curvature");
        static bool first_time = true;
        double psize = 1.0,opacity = 1.0,linesize =1.0;
        if (!first_time)
        {
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
          //viz.removePointCloud ("normalcloud");
          viz.removePointCloud ("cloud");
        }
        else
          first_time = false;

        //viz.addPointCloudNormals<pcl::PointXYZRGBNormal> (normal_cloud, 139, 0.1, "normalcloud");
        viz.addPointCloud<pcl::PointXYZRGBNormal> (normal_cloud, Color_handler, std::string("cloud"), 0);
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
        new_cloud = false;
      }
    }

    template <template <typename> class Storage> void 
    file_cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) 
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      PointCloudAOS<Host> data_host;
      data_host.resize (cloud->points.size());
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
      {
        PointXYZRGB pt;
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        // Pack RGB into a float
        pt.rgb = *(float*)(&(*cloud)[i].rgb);
        data_host[i] = pt;
      }
      data_host.width = cloud->width;
      data_host.height = cloud->height;
      data_host.is_dense = cloud->is_dense;
      typename PointCloudAOS<Storage>::Ptr data = toStorage<Host, Storage> (data_host);

      // we got a cloud in device..

      shared_ptr<typename Storage<float4>::type> normals;
      {
        ScopeTimeCPU time ("Normal Estimation");
        float focallength = 580/2.0;
        normals = computePointNormals<Storage, typename PointIterator<Storage,PointXYZRGB>::type > (data->points.begin (), data->points.end (), focallength, data, 0.05, 30);
      }
      go_on = false;

      std::lock_guard<std::mutex> l(m_mutex);
      normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      toPCL (*data, *normals, *normal_cloud);
      new_cloud = true;
    }

    template <template <typename> class Storage> void 
    cloud_cb (const openni_wrapper::Image::Ptr& image,
              const openni_wrapper::DepthImage::Ptr& depth_image,
              float constant)
    {
      static int smoothing_nr_iterations = 10;
      static int smoothing_filter_size = 2;
      cv::namedWindow("Parameters", CV_WINDOW_NORMAL);
      cvCreateTrackbar ("iterations", "Parameters", &smoothing_nr_iterations, 50, NULL);
      cvCreateTrackbar ("filter_size", "Parameters", &smoothing_filter_size, 10, NULL);
      cv::waitKey (2);

      static unsigned count = 0;
      static double last = getTime ();
      double now = getTime ();
      if (++count == 30 || (now - last) > 5)
      {
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz..........................................." <<  std::endl;
        count = 0;
        last = now;
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      typename PointCloudAOS<Storage>::Ptr data;

      ScopeTimeCPU timer ("All: ");
      // Compute the PointCloud on the device
      d2c.compute<Storage> (depth_image, image, constant, data, false, 1, smoothing_nr_iterations, smoothing_filter_size);
      //d2c.compute<Storage> (depth_image, image, constant, data, true, 2);

      shared_ptr<typename Storage<float4>::type> normals;      
      {
        ScopeTimeCPU time ("Normal Estimation");
        normals = computeFastPointNormals<Storage> (data);
        //float focallength = 580/2.0;
        //normals = computePointNormals<Storage, typename PointIterator<Storage,PointXYZRGB>::type > (data->points.begin (), data->points.end (), focallength, data, 0.05, 30);
      }

      std::lock_guard<std::mutex> l(m_mutex);
      normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      toPCL (*data, *normals, *normal_cloud);
      new_cloud = true;
    }
    
    void 
    run (bool use_device, bool use_file)
    {
      if (use_file)
      {

        float frames_per_second = 1;
        bool repeat = false;

        std::string path = "./frame_0.pcd";
        pcl::PCDGrabber<pcl::PointXYZRGB > filegrabber {path, frames_per_second, repeat};
        
        if (use_device)
        {
          std::cerr << "[NormalEstimation] Using GPU..." << std::endl;
          std::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = std::bind (&NormalEstimation::file_cloud_cb<Device>, this, _1);
          filegrabber.registerCallback (f);
        }
        else
        {
          std::cerr << "[NormalEstimation] Using CPU..." << std::endl;
          std::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = std::bind (&NormalEstimation::file_cloud_cb<Host>, this, _1);
          filegrabber.registerCallback (f);
        }

        filegrabber.start ();
        while (go_on)//!viewer.wasStopped () && go_on)
        {
          pcl_sleep (1);
        }
        filegrabber.stop ();
      }
      else
      {
        pcl::OpenNIGrabber grabber {};

        if (use_device)
        {
          std::cerr << "[NormalEstimation] Using GPU..." << std::endl;
          std::function<void (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float)> f = std::bind (&NormalEstimation::cloud_cb<Device>, this, _1, _2, _3);
          grabber.registerCallback (f);
        }
        else
        {
          std::cerr << "[NormalEstimation] Using CPU..." << std::endl;
          std::function<void (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float)> f = std::bind (&NormalEstimation::cloud_cb<Host>, this, _1, _2, _3);
          grabber.registerCallback (f);
        }

        viewer.runOnVisualizationThread (std::bind(&NormalEstimation::viz_cb, this, _1), "viz_cb");

        grabber.start ();
        
        while (!viewer.wasStopped ())
        {
          pcl_sleep (1);
        }

        grabber.stop ();
      }
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    DisparityToCloud d2c;
    pcl::visualization::CloudViewer viewer;
    std::mutex m_mutex;
    bool new_cloud, go_on;
};

int 
main (int argc, char **argv)
{
  bool use_device = false;
  bool use_file = false;
  if (argc >= 2)
    use_device = true;
  if (argc >= 3)
    use_file = true;
  NormalEstimation v;
  v.run (use_device, use_file);
  return 0;
}

