/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>


#define FPS_CALC(_WHAT_) \
do \
{ \
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
}while(false)

const float default_subsampling_leaf_size = 0.02f;
const float default_normal_search_radius = 0.041f;
const double aux [] = {0.21, 0.32};
const std::vector<double> default_scales_vector (aux, aux + 2);
const float default_alpha = 1.2f;

template <typename PointType>
class OpenNIFeaturePersistence
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIFeaturePersistence (float &subsampling_leaf_size,
                              double &normal_search_radius,
                              std::vector<float> &scales_vector,
                              float &alpha,
                              const std::string& device_id = "")
      : viewer ("PCL OpenNI Feature Persistence Viewer")
    , device_id_(device_id)
    {
      std::cout << "Launching with parameters:\n"
                << "    octree_leaf_size = " << subsampling_leaf_size << "\n"
                << "    normal_search_radius = " << normal_search_radius << "\n"
                << "    persistence_alpha = " << alpha << "\n"
                << "    scales = "; for (size_t i = 0; i < scales_vector.size (); ++i) std::cout << scales_vector[i] << " ";
      std::cout << "\n";

      subsampling_filter_.setLeafSize (subsampling_leaf_size, subsampling_leaf_size, subsampling_leaf_size);
      typename pcl::search::KdTree<PointType>::Ptr normal_search_tree (new typename pcl::search::KdTree<PointType>);
      normal_estimation_filter_.setSearchMethod (normal_search_tree);
      normal_estimation_filter_.setRadiusSearch (normal_search_radius);

      feature_persistence_.setScalesVector (scales_vector);
      feature_persistence_.setAlpha (alpha);

      fpfh_estimation_.reset (new typename pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> ());
      typename pcl::search::KdTree<PointType>::Ptr fpfh_tree (new typename pcl::search::KdTree<PointType> ());
      fpfh_estimation_->setSearchMethod (fpfh_tree);
      feature_persistence_.setFeatureEstimator (fpfh_estimation_);
      feature_persistence_.setDistanceMetric (pcl::CS);

      new_cloud_ = false;
    }


    void
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");

      // Create temporary clouds
      cloud_subsampled_.reset (new typename pcl::PointCloud<PointType> ());
      normals_.reset (new pcl::PointCloud<pcl::Normal> ());
      features_.reset (new pcl::PointCloud<pcl::FPFHSignature33> ());
      feature_indices_.reset (new std::vector<int> ());
      feature_locations_.reset (new typename pcl::PointCloud<PointType> ());

      // Subsample input cloud
      subsampling_filter_.setInputCloud (cloud);
      subsampling_filter_.filter (*cloud_subsampled_);

      // Estimate normals
      normal_estimation_filter_.setInputCloud (cloud_subsampled_);
      normal_estimation_filter_.compute (*normals_);

      // Compute persistent features
      fpfh_estimation_->setInputCloud (cloud_subsampled_);
      fpfh_estimation_->setInputNormals (normals_);
      feature_persistence_.determinePersistentFeatures (*features_, feature_indices_);

      // Extract feature locations by using indices
      extract_indices_filter_.setInputCloud (cloud_subsampled_);
      extract_indices_filter_.setIndices (feature_indices_);
      extract_indices_filter_.filter (*feature_locations_);

      PCL_INFO ("Persistent feature locations %d\n", feature_locations_->points.size ());

      cloud_ = cloud;

      new_cloud_ = true;
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      boost::mutex::scoped_lock lock (mtx_);
      if (!cloud_)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        return;
      }

      CloudConstPtr temp_cloud;
      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that

//      if (!viz.updatePointCloud (temp_cloud, "OpenNICloud"))
//      {
//        viz.addPointCloud (temp_cloud, "OpenNICloud");
//        viz.resetCameraViewpoint ("OpenNICloud");
//      }
      // Render the data
      if (new_cloud_ && feature_locations_)
      {
        viz.removePointCloud ("featurecloud");
        viz.removePointCloud ("OpenNICloud");
        colors_.reset (new typename pcl::visualization::PointCloudColorHandlerCustom<PointType> (feature_locations_, 255.0, 0.0, 0.0));
        viz.addPointCloud (feature_locations_, *colors_, "featurecloud");
        viz.addPointCloud (temp_cloud, "OpenNICloud");
        new_cloud_ = false;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIFeaturePersistence::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&OpenNIFeaturePersistence::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }


    pcl::VoxelGrid<PointType> subsampling_filter_;
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimation_filter_;
    typename pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation_;
    pcl::MultiscaleFeaturePersistence<PointType, pcl::FPFHSignature33> feature_persistence_;
    pcl::ExtractIndices<PointType> extract_indices_filter_;

    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    CloudPtr feature_locations_, cloud_subsampled_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_;
    typename pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr colors_;
    pcl::IndicesPtr feature_indices_;
    CloudConstPtr cloud_;
    bool new_cloud_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n"
            << "         -octree_leaf_size X = size of the leaf for the octree-based subsampling filter (default: " << default_subsampling_leaf_size << "\n"
            << "         -normal_search_radius X = size of the neighborhood to consider for calculating the local plane and extracting the normals (default: " << default_normal_search_radius << "\n"
            << "         -persistence_alpha X = value of alpha for the multiscale feature persistence (default: " << default_alpha << "\n"
            << "         -scales X1 X2 ... = values for the multiple scales for extracting features (default: ";
  for (size_t i = 0; i < default_scales_vector.size (); ++i) std::cout << default_scales_vector[i] << " ";
  std::cout << "\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char **argv)
{
  std::cout << "OpenNIFeaturePersistence - show persistent features based on the MultiscaleFeaturePersistence class using the FPFH features\n"
            << "Use \"-h\" to get more info about the available options.\n";

  if (pcl::console::find_argument (argc, argv, "-h") == -1)
  {
    usage (argv);
    return 1;
  }

  // Parse arguments
  float subsampling_leaf_size = default_subsampling_leaf_size;
  pcl::console::parse_argument (argc, argv, "-octree_leaf_size", subsampling_leaf_size);
  double normal_search_radius = default_normal_search_radius;
  pcl::console::parse_argument (argc, argv, "-normal_search_radius", normal_search_radius);
  std::vector<double> scales_vector_double = default_scales_vector;
  pcl::console::parse_multiple_arguments (argc, argv, "-scales", scales_vector_double);
  std::vector<float> scales_vector (scales_vector_double.size ());
  for (size_t i = 0; i < scales_vector_double.size (); ++i) scales_vector[i] = float (scales_vector_double[i]);

  float alpha = default_alpha;
  pcl::console::parse_argument (argc, argv, "-persistence_alpha", alpha);


  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNIFeaturePersistence<pcl::PointXYZRGBA> v (subsampling_leaf_size,
                                                   normal_search_radius,
                                                   scales_vector,
                                                   alpha,
                                                   "");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIFeaturePersistence<pcl::PointXYZ> v (subsampling_leaf_size,
                                               normal_search_radius,
                                               scales_vector,
                                               alpha,
                                               "");
    v.run ();
  }

  return (0);
}

