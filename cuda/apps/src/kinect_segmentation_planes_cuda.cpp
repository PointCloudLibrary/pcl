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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>

#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/segmentation/connected_components.h>
#include <pcl/cuda/segmentation/mssegmentation.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/gpu/gpu.hpp"

#include <iostream>
#include <fstream>

using namespace pcl::cuda;

template <template <typename> class Storage>
struct ImageType
{
  typedef void type;
};

template <>
struct ImageType<Device>
{
  typedef cv::gpu::GpuMat type;
  static void createContinuous (int h, int w, int typ, type &mat)
  {
    cv::gpu::createContinuous (h, w, typ, mat);
  }
};

template <>
struct ImageType<Host>
{
  typedef cv::Mat type;
  static void createContinuous (int h, int w, int typ, type &mat)
  {
    mat = cv::Mat (h, w, typ); // assume no padding at the end of line
  }
};

class Segmentation
{
  public:
    Segmentation () 
      : viewer ("PCL CUDA - Segmentation"),
      new_cloud(false), go_on(true) {}

    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      static bool first_time = true;
      boost::mutex::scoped_lock l(m_mutex);
      if (new_cloud)
      {
        //typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> ColorHandler;
        typedef pcl::visualization::PointCloudColorHandlerGenericField <pcl::PointXYZRGBNormal> ColorHandler;
        ColorHandler Color_handler (normal_cloud,"curvature");
        if (!first_time)
        {
          viz.removePointCloud ("normalcloud");
          viz.removePointCloud ("cloud");
        }
        else
          first_time = false;

        viz.addPointCloudNormals<pcl::PointXYZRGBNormal> (normal_cloud, 200, 0.1, "normalcloud");
        viz.addPointCloud<pcl::PointXYZRGBNormal> (normal_cloud, Color_handler, std::string("cloud"), 0);
        new_cloud = false;
      }
    }

    template <template <typename> class Storage> void 
    file_cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) 
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      PointCloudAOS<Host> data_host;
      data_host.points.resize (cloud->points.size());
      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
        PointXYZRGB pt;
        pt.x = cloud->points[i].x;
        pt.y = cloud->points[i].y;
        pt.z = cloud->points[i].z;
        // Pack RGB into a float
        pt.rgb = *(float*)(&cloud->points[i].rgb);
        data_host.points[i] = pt;
      }
      data_host.width = cloud->width;
      data_host.height = cloud->height;
      data_host.is_dense = cloud->is_dense;
      typename PointCloudAOS<Storage>::Ptr data = toStorage<Host, Storage> (data_host);

      // we got a cloud in device..

      boost::shared_ptr<typename Storage<float4>::type> normals;
      float focallength = 580/2.0;
      {
        ScopeTimeCPU time ("TIMING: Normal Estimation");
        normals = computePointNormals<Storage, typename PointIterator<Storage,PointXYZRGB>::type > (data->points.begin (), data->points.end (), focallength, data, 0.05, 30);
      }
      go_on = false;

      boost::mutex::scoped_lock l(m_mutex);
      normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      toPCL (*data, *normals, *normal_cloud);
      new_cloud = true;
    }

    template <template <typename> class Storage> void 
    cloud_cb (const boost::shared_ptr<openni_wrapper::Image>& image,
              const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, 
              float constant)
    {
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

      //ScopeTimeCPU timer ("All: ");
      ScopeTimeCPU time ("TIMING: everything");
      // Compute the PointCloud on the device
      d2c.compute<Storage> (depth_image, image, constant, data, true, 2);

      boost::shared_ptr<typename Storage<float4>::type> normals;
      {
        ScopeTimeCPU time ("TIMING: Normal Estimation");
        normals = computeFastPointNormals<Storage> (data);
      }

      // retrieve normals as an image..
      // typename StoragePointer<Storage,char4>::type ptr = StorageAllocator<Storage,char4>::alloc (data->width * data->height);
      // createNormalsImage<Storage> (ptr, normals);
      // typename ImageType<Storage>::type normal_image (data->height, data->width, CV_8UC4, thrust::raw_pointer_cast<char4>(ptr), data->width);
      typename ImageType<Storage>::type normal_image;
      typename StoragePointer<Storage,char4>::type ptr;
      {
        ScopeTimeCPU time ("TIMING: Matrix Creation");
      ImageType<Storage>::createContinuous ((int)data->height, (int)data->width, CV_8UC4, normal_image);
      ptr = typename StoragePointer<Storage,char4>::type ((char4*)normal_image.data);
      createNormalsImage<Storage> (ptr, *normals);
      }

      //TODO: this breaks for pcl::cuda::Host
      pcl::cuda::detail::DjSets comps(0);
      cv::Mat seg;
      {
        ScopeTimeCPU time ("TIMING: Mean Shift");
        pcl::cuda::meanShiftSegmentation (normal_image, seg, 8, 20, 100, comps);
      }
      std::cerr << "time since callback: " << getTime () - now << std::endl;
//      std::cout << "comps " << comps.parent.size () << " " << comps.rank.size () << " " << comps.size.size () << std::endl;
//      std::cout << "parent" << std::endl;
//      std::copy (comps.parent.begin (), comps.parent.end (), std::ostream_iterator<int>(std::cout, " "));
//      std::cout << "rank"   << std::endl;
//      std::copy (comps.rank.begin (), comps.rank.end (), std::ostream_iterator<int>(std::cout, " "));
//      std::cout << "size"   << std::endl;
//      std::copy (comps.size.begin (), comps.size.end (), std::ostream_iterator<int>(std::cout, " "));

      {
        ScopeTimeCPU time ("TIMING: Vis");
      cv::namedWindow("NormalImage", CV_WINDOW_NORMAL);
      //cv::imshow ("NormalImage", cv::Mat(normal_image));
      cv::imshow ("NormalImage", seg);
      cv::waitKey (2);

      boost::mutex::scoped_lock l(m_mutex);
      normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      toPCL (*data, *normals, *normal_cloud);
      new_cloud = true;
      }
    }
    
    void 
    run (bool use_device, bool use_file)
    {
      if (use_file)
      {
        pcl::Grabber* filegrabber = 0;

        float frames_per_second = 1;
        bool repeat = false;

        std::string path = "./frame_0.pcd";
        filegrabber = new pcl::PCDGrabber<pcl::PointXYZRGB > (path, frames_per_second, repeat);
        
        if (use_device)
        {
          std::cerr << "[Segmentation] Using GPU..." << std::endl;
          boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&Segmentation::file_cloud_cb<Device>, this, _1);
          filegrabber->registerCallback (f);
        }
        else
        {
//          std::cerr << "[Segmentation] Using CPU..." << std::endl;
//          boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&Segmentation::file_cloud_cb<Host>, this, _1);
//          filegrabber->registerCallback (f);
        }

        filegrabber->start ();
        while (go_on)//!viewer.wasStopped () && go_on)
        {
          pcl_sleep (1);
        }
        filegrabber->stop ();
      }
      else
      {
        pcl::Grabber* grabber = new pcl::OpenNIGrabber();

        boost::signals2::connection c;
        if (use_device)
        {
          std::cerr << "[Segmentation] Using GPU..." << std::endl;
          boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)> f = boost::bind (&Segmentation::cloud_cb<Device>, this, _1, _2, _3);
          c = grabber->registerCallback (f);
        }
        else
        {
//          std::cerr << "[Segmentation] Using CPU..." << std::endl;
//          boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)> f = boost::bind (&Segmentation::cloud_cb<Host>, this, _1, _2, _3);
//          c = grabber->registerCallback (f);
        }

        viewer.runOnVisualizationThread (boost::bind(&Segmentation::viz_cb, this, _1), "viz_cb");

        grabber->start ();
        
        while (!viewer.wasStopped ())
        {
          pcl_sleep (1);
        }

        grabber->stop ();
      }
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    DisparityToCloud d2c;
    pcl::visualization::CloudViewer viewer;
    boost::mutex m_mutex;
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
  Segmentation s;
  s.run (use_device, use_file);
  return 0;
}

