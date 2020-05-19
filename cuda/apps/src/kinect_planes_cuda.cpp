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

// pcl::cuda includes
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/sample_consensus/multi_ransac.h>
#include <pcl/cuda/segmentation/connected_components.h>
#include <pcl/cuda/features/normal_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>

#include <functional>
#include <iostream>
#include <mutex>

using namespace pcl::cuda;

#undef interface

class MultiRansac
{
  public:
    MultiRansac () : viewer ("PCL CUDA - MultiSac"), new_cloud(false), use_viewer(true) {}

    void logo_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
    {
      //logo_cloud_ = cloud;
      //viewer.showCloud (logo_cloud_, "logo");
      std::cerr << "got logo" << std::endl;
    }

    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      std::lock_guard<std::mutex> l(m_mutex);
      if (new_cloud)
      {
        double psize = 1.0,opacity = 1.0,linesize =1.0;
        std::string cloud_name ("cloud");
        using ColorHandler = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>;

        ColorHandler Color_handler (normal_cloud);
        static bool first_time = true;
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
    cloud_cb (const openni_wrapper::Image::Ptr& image,
              const openni_wrapper::DepthImage::Ptr& depth_image,
              float constant)
    {
      static unsigned count = 0;
      static double last = pcl::cuda::getTime ();
      double now = pcl::cuda::getTime ();
      if (++count == 30 || (now - last) > 5)
      {
        std::cerr << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
      //static unsigned int nr_planes_last_time = 0;

      std::cerr << "received callback" << std::endl;
      typename PointCloudAOS<Storage>::Ptr data;
      typename SampleConsensusModel1PointPlane<Storage>::Ptr sac_model;
      {
      ScopeTimeCPU timer ("All: ");
      static int threshold_x100 = 25;
      static int smoothing_nr_iterations = 10;
      static int smoothing_filter_size = 2;
      static int enable_coloring = 1;

      // Compute the PointCloud on the device
      //d2c.compute<Storage> (depth_image, image, constant, data, true, 2);
      d2c.compute<Storage> (depth_image, image, constant, data, true, 2, smoothing_nr_iterations, smoothing_filter_size);

      // Compute normals
      shared_ptr<typename Storage<float4>::type> normals;
      {
        ScopeTimeCPU time ("Normal Estimation");
        //normals = computeFastPointNormals<Storage> (data);
        float focallength = 580/2.0;
        //normals = computePointNormals<Storage> (data->points.begin (), data->points.end (), focallength, data, 0.11f, 36);
        normals = computeFastPointNormals<Storage> (data);
      }

      // Create sac_model
      {
        ScopeTimeCPU t ("creating sac_model");
        sac_model.reset (new SampleConsensusModel1PointPlane<Storage> (data));
      }
      sac_model->setNormals (normals);

      MultiRandomSampleConsensus<Storage> sac (sac_model);
      sac.setMinimumCoverage (0.90); // at least 95% points should be explained by planes
      sac.setMaximumBatches (5);
      sac.setIerationsPerBatch (1000);
      sac.setDistanceThreshold (threshold_x100 / 100.0);

      {
        ScopeTimeCPU timer ("computeModel: ");
        if (!sac.computeModel (0))
        {
          std::cerr << "Failed to compute model" << std::endl;
        }
        else
        {
          if (use_viewer)
          {
            std::cerr << "getting inliers.. ";
            
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
            int best_plane_inliers_count = -1;

            for (unsigned int i = 0; i < planes.size (); i++)
            {
              if (planes_inlier_counts[i] > best_plane_inliers_count)
              {
                best_plane_inliers_count = planes_inlier_counts[i];
              }

              typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr inliers_stencil;
              inliers_stencil = planes[i];//sac.getInliersStencil ();

              OpenNIRGB color;
              double trand = 255 / (RAND_MAX + 1.0);

              color.r = (int)(rand () * trand);
              color.g = (int)(rand () * trand);
              color.b = (int)(rand () * trand);
              if (enable_coloring)
              {
                ScopeTimeCPU t ("coloring planes");
                colorIndices<Storage> (data, inliers_stencil, color);
              }
            }
          }

          cv::namedWindow("Parameters", CV_WINDOW_NORMAL);
          cvCreateTrackbar( "iterations", "Parameters", &smoothing_nr_iterations, 50, NULL);
          cvCreateTrackbar( "filter_size", "Parameters", &smoothing_filter_size, 10, NULL);
          cvCreateTrackbar( "enable_coloring", "Parameters", &enable_coloring, 1, NULL);
          cvCreateTrackbar( "threshold", "Parameters", &threshold_x100, 100, NULL);
          cv::waitKey (2);
        }
      }

    }
      if (use_viewer)
      {
        auto normals = sac_model->getNormals ();

        std::lock_guard<std::mutex> l(m_mutex);
        normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        toPCL (*data, *normals, *normal_cloud);
        new_cloud = true;
      }

      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      //toPCL (*data, *output);
      //viewer.showCloud (output);
    }
    
    void 
    run (bool use_device, bool use_viewer)
    {
      this->use_viewer = use_viewer;
      //cudaDeviceSetCacheConfig (cudaFuncCachePreferL1);
      pcl::OpenNIGrabber interface {};

      if (use_device)
      {
        std::cerr << "[RANSAC] Using GPU..." << std::endl;
        std::function<void (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float)> f = std::bind (&MultiRansac::cloud_cb<Device>, this, _1, _2, _3);
        interface.registerCallback (f);
      }
      else
      {
        std::cerr << "[RANSAC] Using CPU..." << std::endl;
        std::function<void (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float)> f = std::bind (&MultiRansac::cloud_cb<Host>, this, _1, _2, _3);
        interface.registerCallback (f);
      }

      if (use_viewer)
        viewer.runOnVisualizationThread (std::bind(&MultiRansac::viz_cb, this, _1), "viz_cb");

      interface.start ();

      //--------------------- load pcl logo file
      //float frames_per_second = 1;
      //bool repeat = false;

      //std::string path = "./pcl_logo.pcd";
      //if (path.empty() || !boost::filesystem::exists (path))
      //{
      //  std::cerr << "did not find file" << std::endl;
      //}
      //pcl::PCDGrabber<pcl::PointXYZRGB> filegrabber {path, frames_per_second, repeat};
      //
      //std::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&) > f = std::bind (&MultiRansac::logo_cb, this, _1);
      //boost::signals2::connection c1 = filegrabber.registerCallback (f);

      //filegrabber.start ();
      //------- END --------- load pcl logo file
      //
      while (!viewer.wasStopped ())
      {
        pcl_sleep (1);
      }

      //filegrabber.stop ();
      interface.stop ();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr logo_cloud_;
    DisparityToCloud d2c;
    pcl::visualization::CloudViewer viewer;
   
    std::mutex m_mutex;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    bool new_cloud;
    bool use_viewer;
};

int 
main (int argc, char **argv)
{
  pcl_sleep (1);
  bool use_device = false;
  bool use_viewer = true;
  if (argc >= 2)
    use_device = true;
  if (argc >= 3)
    use_viewer = false;
  MultiRansac v;
  v.run (use_device, use_viewer);
  return 0;
}
