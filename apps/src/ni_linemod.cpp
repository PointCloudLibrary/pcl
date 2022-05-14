/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */

#include <pcl/apps/timer.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <mutex>
#include <thread>

using namespace pcl;
using namespace std::chrono_literals;

using PointT = PointXYZRGBA;

#define SHOW_FPS 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class NILinemod {
public:
  using Cloud = PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;
  using CloudConstPtr = Cloud::ConstPtr;
  bool added;

  NILinemod(Grabber& grabber)
  : cloud_viewer_("PointCloud")
  , grabber_(grabber)
  , image_viewer_("Image")
  , first_frame_(true)
  {
    added = false;

    // Set the parameters for normal estimation
    ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
    ne_.setMaxDepthChangeFactor(0.02f);
    ne_.setNormalSmoothingSize(20.0f);

    // Set the parameters for planar segmentation
    plane_comparator_.reset(new EdgeAwarePlaneComparator<PointT, Normal>);
    plane_comparator_->setDistanceThreshold(0.01f, false);
    mps_.setMinInliers(5000);
    mps_.setAngularThreshold(pcl::deg2rad(3.0)); // 3 degrees
    mps_.setDistanceThreshold(0.02);             // 2 cm
    mps_.setMaximumCurvature(0.001);             // a small curvature
    mps_.setProjectPoints(true);
    mps_.setComparator(plane_comparator_);
  }

  /////////////////////////////////////////////////////////////////////////
  void
  cloud_callback(const CloudConstPtr& cloud)
  {
    FPS_CALC("cloud callback");
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_ = cloud;
    search_.setInputCloud(cloud);

    // Subsequent frames are segmented by "tracking" the parameters of the previous
    // frame We do this by using the estimated inliers from previous frames in the
    // current frame, and refining the coefficients

    if (!first_frame_) {
      if (!plane_indices_ || plane_indices_->indices.empty() ||
          !search_.getInputCloud()) {
        PCL_ERROR("Lost tracking. Select the object again to continue.\n");
        first_frame_ = true;
        return;
      }
      SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(SACMODEL_PLANE);
      seg.setMethodType(SAC_RANSAC);
      seg.setMaxIterations(1000);
      seg.setDistanceThreshold(0.01);
      seg.setInputCloud(search_.getInputCloud());
      seg.setIndices(plane_indices_);
      ModelCoefficients coefficients;
      PointIndices inliers;
      seg.segment(inliers, coefficients);

      if (inliers.indices.empty()) {
        PCL_ERROR("No planar model found. Select the object again to continue.\n");
        first_frame_ = true;
        return;
      }

      // Visualize the object in 3D...
      CloudPtr plane_inliers(new Cloud);
      pcl::copyPointCloud(*search_.getInputCloud(), inliers.indices, *plane_inliers);
      if (plane_inliers->points.empty()) {
        PCL_ERROR("No planar model found. Select the object again to continue.\n");
        first_frame_ = true;
        return;
      }
      plane_.reset(new Cloud);

      // Compute the convex hull of the plane
      ConvexHull<PointT> chull;
      chull.setDimension(2);
      chull.setInputCloud(plane_inliers);
      chull.reconstruct(*plane_);
    }
  }

  /////////////////////////////////////////////////////////////////////////
  CloudConstPtr
  getLatestCloud()
  {
    // Lock while we swap our cloud and reset it.
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    CloudConstPtr temp_cloud;
    temp_cloud.swap(cloud_);
    return temp_cloud;
  }

  /////////////////////////////////////////////////////////////////////////
  void
  keyboard_callback(const visualization::KeyboardEvent&, void*)
  {
    // if (event.getKeyCode())
    //  std::cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode()
    //  << ") was";
    // else
    //  std::cout << "the special key \'" << event.getKeySym() << "\' was";
    // if (event.keyDown())
    //  std::cout << " pressed" << std::endl;
    // else
    //  std::cout << " released" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////
  void
  mouse_callback(const visualization::MouseEvent&, void*)
  {
    // if (mouse_event.getType() == visualization::MouseEvent::MouseButtonPress &&
    // mouse_event.getButton() == visualization::MouseEvent::LeftButton)
    //{
    //  std::cout << "left button pressed @ " << mouse_event.getX () << " , " <<
    //  mouse_event.getY () << std::endl;
    //}
  }

  /////////////////////////////////////////////////////////////////////////
  /**
   * \brief Given a plane, and the set of inlier indices representing it,
   * segment out the object of intererest supported by it.
   *
   * \param[in] picked_idx the index of a point on the object
   * \param[in] cloud the full point cloud dataset
   * \param[in] plane_indices a set of indices representing the plane supporting the
   * object of interest
   * \param[in] plane_boundary_indices a set of indices representing
   * the boundary of the plane
   * \param[out] object the segmented resultant object
   */
  void
  segmentObject(pcl::index_t picked_idx,
                const CloudConstPtr& cloud,
                const PointIndices::Ptr& plane_indices,
                const PointIndices::Ptr& plane_boundary_indices,
                Cloud& object)
  {
    CloudPtr plane_hull(new Cloud);

    // Compute the convex hull of the plane
    ConvexHull<PointT> chull;
    chull.setDimension(2);
    chull.setInputCloud(cloud);
    chull.setIndices(plane_boundary_indices);
    chull.reconstruct(*plane_hull);

    // Remove the plane indices from the data
    PointIndices::Ptr everything_but_the_plane(new PointIndices);
    if (indices_fullset_.size() != cloud->size()) {
      indices_fullset_.resize(cloud->size());
      for (int p_it = 0; p_it < static_cast<int>(indices_fullset_.size()); ++p_it)
        indices_fullset_[p_it] = p_it;
    }
    pcl::Indices indices_subset = plane_indices->indices;
    std::sort(indices_subset.begin(), indices_subset.end());
    set_difference(indices_fullset_.begin(),
                   indices_fullset_.end(),
                   indices_subset.begin(),
                   indices_subset.end(),
                   inserter(everything_but_the_plane->indices,
                            everything_but_the_plane->indices.begin()));

    // Extract all clusters above the hull
    PointIndices::Ptr points_above_plane(new PointIndices);
    ExtractPolygonalPrismData<PointT> exppd;
    exppd.setInputCloud(cloud);
    exppd.setInputPlanarHull(plane_hull);
    exppd.setIndices(everything_but_the_plane);
    exppd.setHeightLimits(0.0, 0.5); // up to half a meter
    exppd.segment(*points_above_plane);

    // Use an organized clustering segmentation to extract the individual clusters
    EuclideanClusterComparator<PointT, Label>::Ptr euclidean_cluster_comparator(
        new EuclideanClusterComparator<PointT, Label>);
    euclidean_cluster_comparator->setInputCloud(cloud);
    euclidean_cluster_comparator->setDistanceThreshold(0.03f, false);
    // Set the entire scene to false, and the inliers of the objects located on top of
    // the plane to true
    Label l;
    l.label = 0;
    PointCloud<Label>::Ptr scene(new PointCloud<Label>(cloud->width, cloud->height, l));
    // Mask the objects that we want to split into clusters
    for (const auto& index : points_above_plane->indices)
      (*scene)[index].label = 1;
    euclidean_cluster_comparator->setLabels(scene);

    OrganizedConnectedComponentSegmentation<PointT, Label> euclidean_segmentation(
        euclidean_cluster_comparator);
    euclidean_segmentation.setInputCloud(cloud);

    PointCloud<Label> euclidean_labels;
    std::vector<PointIndices> euclidean_label_indices;
    euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

    // For each cluster found
    bool cluster_found = false;
    for (const auto& euclidean_label_index : euclidean_label_indices) {
      if (cluster_found)
        break;
      // Check if the point that we picked belongs to it
      for (std::size_t j = 0; j < euclidean_label_index.indices.size(); ++j) {
        if (picked_idx != euclidean_label_index.indices[j])
          continue;
        // pcl::PointCloud<PointT> cluster;
        pcl::copyPointCloud(*cloud, euclidean_label_index.indices, object);
        cluster_found = true;
        break;
        // object_indices = euclidean_label_indices[i].indices;
        // clusters.push_back (cluster);
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////
  void
  segment(const PointT& picked_point,
          pcl::index_t picked_idx,
          PlanarRegion<PointT>& region,
          PointIndices&,
          CloudPtr& object)
  {
    // First frame is segmented using an organized multi plane segmentation approach
    // from points and their normals
    if (!first_frame_)
      return;

    // Estimate normals in the cloud
    PointCloud<Normal>::Ptr normal_cloud(new PointCloud<Normal>);
    ne_.setInputCloud(search_.getInputCloud());
    ne_.compute(*normal_cloud);

    plane_comparator_->setDistanceMap(ne_.getDistanceMap());

    // Segment out all planes
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(search_.getInputCloud());

    // Use one of the overloaded segmentAndRefine calls to get all the information that
    // we want out
    std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT>>>
        regions;
    std::vector<ModelCoefficients> model_coefficients;
    std::vector<PointIndices> inlier_indices;
    PointCloud<Label>::Ptr labels(new PointCloud<Label>);
    std::vector<PointIndices> label_indices;
    std::vector<PointIndices> boundary_indices;
    mps_.segmentAndRefine(regions,
                          model_coefficients,
                          inlier_indices,
                          labels,
                          label_indices,
                          boundary_indices);
    PCL_DEBUG("Number of planar regions detected: %zu for a cloud of %zu points and "
              "%zu normals.\n",
              static_cast<std::size_t>(regions.size()),
              static_cast<std::size_t>(search_.getInputCloud()->size()),
              static_cast<std::size_t>(normal_cloud->size()));

    double max_dist = std::numeric_limits<double>::max();
    // Compute the distances from all the planar regions to the picked point, and select
    // the closest region
    int idx = -1;
    for (std::size_t i = 0; i < regions.size(); ++i) {
      double dist = pointToPlaneDistance(picked_point, regions[i].getCoefficients());
      if (dist < max_dist) {
        max_dist = dist;
        idx = static_cast<int>(i);
      }
    }

    PointIndices::Ptr plane_boundary_indices;
    // Get the plane that holds the object of interest
    if (idx != -1) {
      region = regions[idx];
      plane_indices_.reset(new PointIndices(inlier_indices[idx]));
      plane_boundary_indices.reset(new PointIndices(boundary_indices[idx]));
    }

    // Segment the object of interest
    if (plane_boundary_indices && !plane_boundary_indices->indices.empty()) {
      object.reset(new Cloud);
      segmentObject(picked_idx,
                    search_.getInputCloud(),
                    plane_indices_,
                    plane_boundary_indices,
                    *object);

      // Save to disk
      // pcl::io::saveTARPointCloud ("output.ltm", *object, "object.pcd");
    }
    first_frame_ = false;
  }

  /////////////////////////////////////////////////////////////////////////
  /**
   * \brief Point picking callback. This gets called when the user selects
   * a 3D point on screen (in the PCLVisualizer window) using Shift+click.
   *
   * \param[in] event the event that triggered the call
   */
  void
  pp_callback(const visualization::PointPickingEvent& event, void*)
  {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex();
    if (idx == -1)
      return;

    pcl::Indices indices(1);
    std::vector<float> distances(1);

    // Use mutices to make sure we get the right cloud
    std::lock_guard<std::mutex> lock1(cloud_mutex_);

    // Get the point that was picked
    PointT picked_pt;
    event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

    // Add a sphere to it in the PCLVisualizer window
    const std::string sphere_name = "sphere_" + std::to_string(idx);
    cloud_viewer_.addSphere(picked_pt, 0.01, 1.0, 0.0, 0.0, sphere_name);

    // Check to see if we have access to the actual cloud data. Use the previously built
    // search object.
    if (!search_.isValid())
      return;

    // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we
    // must search for the real point
    search_.nearestKSearch(picked_pt, 1, indices, distances);

    // Get the [u, v] in pixel coordinates for the ImageViewer. Remember that 0,0 is
    // bottom left.
    std::uint32_t width = search_.getInputCloud()->width;
    //               height = search_.getInputCloud ()->height;
    int v = indices[0] / width, u = indices[0] % width;

    // Add some marker to the image
    image_viewer_.addCircle(u, v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
    image_viewer_.addFilledRectangle(
        u - 5, u + 5, v - 5, v + 5, 0.0, 1.0, 0.0, "boxes", 0.5);
    image_viewer_.markPoint(
        u, v, visualization::red_color, visualization::blue_color, 10);

    // Segment the region that we're interested in, by employing a two step process:
    //  * first, segment all the planes in the scene, and find the one closest to our
    //  picked point
    //  * then, use euclidean clustering to find the object that we clicked on and
    //  return it
    PlanarRegion<PointT> region;
    CloudPtr object;
    PointIndices region_indices;
    segment(picked_pt, indices[0], region, region_indices, object);

    // If no region could be determined, exit
    if (region.getContour().empty()) {
      PCL_ERROR("No planar region detected. Please select another point or relax the "
                "thresholds and continue.\n");
      return;
    }
    // Else, draw it on screen
    // cloud_viewer_.addPolygon (region, 1.0, 0.0, 0.0, "region");
    // cloud_viewer_.setShapeRenderingProperties
    // (visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "region");

    PlanarRegion<PointT> refined_region;
    pcl::approximatePolygon(region, refined_region, 0.01, false, true);
    PCL_INFO("Planar region: %zu points initial, %zu points after refinement.\n",
             static_cast<std::size_t>(region.getContour().size()),
             static_cast<std::size_t>(refined_region.getContour().size()));
    cloud_viewer_.addPolygon(refined_region, 0.0, 0.0, 1.0, "refined_region");
    cloud_viewer_.setShapeRenderingProperties(
        visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "refined_region");

    // Draw in image space
    image_viewer_.addPlanarPolygon(
        search_.getInputCloud(), refined_region, 0.0, 0.0, 1.0, "refined_region", 1.0);

    // If no object could be determined, exit
    if (!object) {
      PCL_ERROR("No object detected. Please select another point or relax the "
                "thresholds and continue.\n");
      return;
    }
    // Visualize the object in 3D...
    visualization::PointCloudColorHandlerCustom<PointT> red(object, 255, 0, 0);
    if (!cloud_viewer_.updatePointCloud(object, red, "object"))
      cloud_viewer_.addPointCloud(object, red, "object");
    // ...and 2D
    image_viewer_.removeLayer("object");
    image_viewer_.addMask(search_.getInputCloud(), *object, "object");

    // Compute the min/max of the object
    PointT min_pt, max_pt;
    getMinMax3D(*object, min_pt, max_pt);
    const std::string cube_name = "cube_" + std::to_string(idx);
    // Visualize the bounding box in 3D...
    cloud_viewer_.addCube(min_pt.x,
                          max_pt.x,
                          min_pt.y,
                          max_pt.y,
                          min_pt.z,
                          max_pt.z,
                          0.0,
                          1.0,
                          0.0,
                          cube_name);
    cloud_viewer_.setShapeRenderingProperties(
        visualization::PCL_VISUALIZER_LINE_WIDTH, 10, cube_name);

    // ...and 2D
    image_viewer_.addRectangle(search_.getInputCloud(), *object);
  }

  /////////////////////////////////////////////////////////////////////////
  void
  init()
  {
    cloud_viewer_.registerMouseCallback(&NILinemod::mouse_callback, *this);
    cloud_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
    cloud_viewer_.registerPointPickingCallback(&NILinemod::pp_callback, *this);
    std::function<void(const CloudConstPtr&)> cloud_cb =
        [this](const CloudConstPtr& cloud) { cloud_callback(cloud); };
    cloud_connection = grabber_.registerCallback(cloud_cb);

    image_viewer_.registerMouseCallback(&NILinemod::mouse_callback, *this);
    image_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
  }

  /////////////////////////////////////////////////////////////////////////
  void
  run()
  {
    grabber_.start();

    bool image_init = false, cloud_init = false;

    while (!cloud_viewer_.wasStopped() && !image_viewer_.wasStopped()) {
      if (cloud_) {
        CloudConstPtr cloud = getLatestCloud();
        if (!cloud_init) {
          cloud_viewer_.setPosition(0, 0);
          cloud_viewer_.setSize(cloud->width, cloud->height);
          cloud_init = true;
        }

        if (!cloud_viewer_.updatePointCloud(cloud, "OpenNICloud")) {
          cloud_viewer_.addPointCloud(cloud, "OpenNICloud");
          cloud_viewer_.resetCameraViewpoint("OpenNICloud");
        }

        if (!image_init) {
          image_viewer_.setPosition(cloud->width, 0);
          image_viewer_.setSize(cloud->width, cloud->height);
          image_init = true;
        }

        image_viewer_.showRGBImage<PointT>(cloud);
      }

      // Add the plane that we're tracking to the cloud visualizer
      CloudPtr plane(new Cloud);
      if (plane_)
        *plane = *plane_;
      visualization::PointCloudColorHandlerCustom<PointT> blue(plane, 0, 255, 0);
      if (!cloud_viewer_.updatePointCloud(plane, blue, "plane"))
        cloud_viewer_.addPointCloud(plane, "plane");
      cloud_viewer_.spinOnce();

      image_viewer_.spinOnce();
      std::this_thread::sleep_for(100us);
    }

    grabber_.stop();

    cloud_connection.disconnect();
  }

  visualization::PCLVisualizer cloud_viewer_;
  Grabber& grabber_;
  std::mutex cloud_mutex_;
  CloudConstPtr cloud_;

  visualization::ImageViewer image_viewer_;

  search::OrganizedNeighbor<PointT> search_;

private:
  boost::signals2::connection cloud_connection, image_connection;
  bool first_frame_;

  // Segmentation
  pcl::Indices indices_fullset_;
  PointIndices::Ptr plane_indices_;
  CloudPtr plane_;
  IntegralImageNormalEstimation<PointT, Normal> ne_;
  OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps_;
  EdgeAwarePlaneComparator<PointT, Normal>::Ptr plane_comparator_;
};

/* ---[ */
int
main(int, char**)
{
  std::string device_id("#1");
  OpenNIGrabber grabber(device_id);
  NILinemod openni_viewer(grabber);

  openni_viewer.init();
  openni_viewer.run();

  return 0;
}
/* ]--- */
