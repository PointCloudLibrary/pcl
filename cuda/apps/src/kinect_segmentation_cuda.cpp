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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/gpu/gpu.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

#include <boost/shared_ptr.hpp>

#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/sample_consensus/multi_ransac.h>
#include <pcl/cuda/segmentation/connected_components.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

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
      new_cloud(false), go_on(true), enable_color(1), normal_method(0), nr_neighbors (36), radius_cm (5), normal_viz_step(200)
      , enable_normal_viz(false)
    {}

    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      static bool first_time = true;
      static bool last_enable_normal_viz = enable_normal_viz;
      boost::mutex::scoped_lock l(m_mutex);
      if (new_cloud)
      {
        double psize = 1.0,opacity = 1.0,linesize =1.0;
        std::string cloud_name ("cloud");

        if (!first_time)
        {
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
          viz.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
          if (last_enable_normal_viz)
            viz.removePointCloud ("normalcloud");
          viz.removePointCloud ("cloud");
        }
        else
          first_time = false;

        if (enable_normal_viz)
          viz.addPointCloudNormals<pcl::PointXYZRGBNormal> (normal_cloud, normal_viz_step, 0.1, "normalcloud");

        if (enable_color == 1)
        {
          typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> ColorHandler;
          ColorHandler Color_handler (normal_cloud);
          viz.addPointCloud<pcl::PointXYZRGBNormal> (normal_cloud, Color_handler, std::string(cloud_name));
        }
        else
        {
          typedef pcl::visualization::PointCloudColorHandlerGenericField <pcl::PointXYZRGBNormal> ColorHandler;
          ColorHandler Color_handler (normal_cloud,"curvature");
          viz.addPointCloud<pcl::PointXYZRGBNormal> (normal_cloud, Color_handler, cloud_name, 0);
        }
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
        new_cloud = false;
      }
      last_enable_normal_viz = enable_normal_viz;
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
        ScopeTimeCPU time ("Normal Estimation");
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
      //if (++count == 30 || (now - last) > 5)
      {
        std::cout << std::endl;
        count = 1;
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz --- ";
        last = now;
      }

      static int smoothing_nr_iterations = 10;
      static int smoothing_filter_size = 2;
      static int enable_visualization = 1;
      static int enable_mean_shift = 0;
      static int enable_plane_fitting = 0;
      static int meanshift_sp=8;
      static int meanshift_sr=20;
      static int meanshift_minsize=100;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      typename PointCloudAOS<Storage>::Ptr data;

      //ScopeTimeCPU timer ("All: ");
      ScopeTimeCPU time ("everything");
      // Compute the PointCloud on the device
      {
        ScopeTimeCPU time ("disparity smoothing");
        d2c.compute<Storage> (depth_image, image, constant, data, false, 1, smoothing_nr_iterations, smoothing_filter_size);
      }

      boost::shared_ptr<typename Storage<float4>::type> normals;
      float focallength = 580/2.0;
      {
        ScopeTimeCPU time ("Normal Estimation");
        if (normal_method == 1)
          normals = computeFastPointNormals<Storage> (data);
        else
          normals = computePointNormals<Storage> (data->points.begin (), data->points.end (), focallength, data, radius_cm / 100.0f, nr_neighbors);
        cudaThreadSynchronize ();
      }

      // retrieve normals as an image..
      typename ImageType<Storage>::type normal_image;
      typename StoragePointer<Storage,char4>::type ptr;
      {
        ScopeTimeCPU time ("Matrix Creation");
        ImageType<Storage>::createContinuous ((int)data->height, (int)data->width, CV_8UC4, normal_image);
        ptr = typename StoragePointer<Storage,char4>::type ((char4*)normal_image.data);
        createNormalsImage<Storage> (ptr, *normals);
      }

      //TODO: this breaks for pcl::cuda::Host
      cv::Mat seg;
      {
        ScopeTimeCPU time ("Mean Shift");
        if (enable_mean_shift == 1)
        {
          cv::gpu::meanShiftSegmentation (normal_image, seg, meanshift_sp, meanshift_sr, meanshift_minsize);
          typename Storage<char4>::type new_colors ((char4*)seg.datastart, (char4*)seg.dataend);
          colorCloud<Storage> (data, new_colors);
        }
      }

      typename SampleConsensusModel1PointPlane<Storage>::Ptr sac_model;
      if (enable_plane_fitting == 1)
      {
        // Create sac_model
        {
          ScopeTimeCPU t ("creating sac_model");
          sac_model.reset (new SampleConsensusModel1PointPlane<Storage> (data));
        }
        sac_model->setNormals (normals);

        MultiRandomSampleConsensus<Storage> sac (sac_model);
        sac.setMinimumCoverage (0.90); // at least 95% points should be explained by planes
        sac.setMaximumBatches (1);
        sac.setIerationsPerBatch (1024);
        sac.setDistanceThreshold (0.05);

        {
          ScopeTimeCPU timer ("computeModel: ");
          if (!sac.computeModel (0))
          {
            std::cerr << "Failed to compute model" << std::endl;
          }
          else
          {
            if (enable_visualization)
            {
//              std::cerr << "getting inliers.. ";
              
              std::vector<typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr> planes;
              typename Storage<int>::type region_mask;
              markInliers<Storage> (data, region_mask, planes);
              thrust::host_vector<int> regions_host;
              std::copy (regions_host.begin (), regions_host.end(), std::ostream_iterator<int>(std::cerr, " "));
              {
                ScopeTimeCPU t ("retrieving inliers");
                planes = sac.getAllInliers ();
              }
              std::vector<int> planes_inlier_counts = sac.getAllInlierCounts ();
              std::vector<float4> coeffs = sac.getAllModelCoefficients ();
              std::vector<float3> centroids = sac.getAllModelCentroids ();
              std::cerr << "Found " << planes_inlier_counts.size () << " planes" << std::endl;
              int best_plane = 0;
              int best_plane_inliers_count = -1;

              for (unsigned int i = 0; i < planes.size (); i++)
              {
                if (planes_inlier_counts[i] > best_plane_inliers_count)
                {
                  best_plane = i;
                  best_plane_inliers_count = planes_inlier_counts[i];
                }

                typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr inliers_stencil;
                inliers_stencil = planes[i];//sac.getInliersStencil ();

                OpenNIRGB color;
                //double trand = 255 / (RAND_MAX + 1.0);

                //color.r = (int)(rand () * trand);
                //color.g = (int)(rand () * trand);
                //color.b = (int)(rand () * trand);
                color.r = (1.0f + coeffs[i].x) * 128;
                color.g = (1.0f + coeffs[i].y) * 128;
                color.b = (1.0f + coeffs[i].z) * 128;
                {
                  ScopeTimeCPU t ("coloring planes");
                  colorIndices<Storage> (data, inliers_stencil, color);
                }
              }
            }
          }
        }
      }

      {
        ScopeTimeCPU time ("Vis");
        cv::namedWindow("NormalImage", CV_WINDOW_NORMAL);
        cv::namedWindow("Parameters", CV_WINDOW_NORMAL);
        cvCreateTrackbar( "iterations", "Parameters", &smoothing_nr_iterations, 50, NULL);
        cvCreateTrackbar( "filter_size", "Parameters", &smoothing_filter_size, 10, NULL);
        cvCreateTrackbar( "enable_visualization", "Parameters", &enable_visualization, 1, NULL);
        cvCreateTrackbar( "enable_color", "Parameters", &enable_color, 1, NULL);
        cvCreateTrackbar( "normal_method", "Parameters", &normal_method, 1, NULL);
        cvCreateTrackbar( "neighborhood_radius", "Parameters", &radius_cm, 50, NULL);
        cvCreateTrackbar( "nr_neighbors", "Parameters", &nr_neighbors, 400, NULL);
        cvCreateTrackbar( "normal_viz_step", "Parameters", &normal_viz_step, 1000, NULL);
        cvCreateTrackbar( "enable_mean_shift", "Parameters", &enable_mean_shift, 1, NULL);
        cvCreateTrackbar( "meanshift_sp", "Parameters", &meanshift_sp, 100, NULL);
        cvCreateTrackbar( "meanshift_sr", "Parameters", &meanshift_sr, 100, NULL);
        cvCreateTrackbar( "meanshift_minsize", "Parameters", &meanshift_minsize, 500, NULL);
        cvCreateTrackbar( "enable_plane_fitting", "Parameters", &enable_plane_fitting, 1, NULL);
        cvCreateTrackbar( "enable_normal_viz", "Parameters", &enable_normal_viz, 1, NULL);
        if (enable_visualization == 1)
        {

          cv::Mat temp;
          if (enable_mean_shift == 1)
            cv::cvtColor(seg,temp,CV_BGR2RGB);
          else
            cv::cvtColor(cv::Mat(normal_image),temp,CV_BGR2RGB);
          cv::imshow ("NormalImage", temp);

          boost::mutex::scoped_lock l(m_mutex);
          normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          toPCL (*data, *normals, *normal_cloud);
          new_cloud = true;
        }
        cv::waitKey (2);
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
    int enable_color;
    int normal_method;
    int nr_neighbors;
    int radius_cm;
    int normal_viz_step;
    int enable_normal_viz;
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

