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

#include <pcl/common/centroid.h> // for computeMeanAndCovarianceMatrix
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/stereo/stereo_matching.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>

#include <boost/filesystem.hpp> // for directory_iterator

#include <mutex>

using PointT = pcl::PointXYZRGB;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

/** \brief StereoGroundSegmentation is a demonstration application for using PCL's
 * stereo tools and segmentation tools to detect smooth surfaces suitable for driving.
 *
 * \author Alex Trevor
 */
class HRCSSegmentation {
private:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::ImageViewer::Ptr image_viewer;
  pcl::PointCloud<PointT>::ConstPtr prev_cloud;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  pcl::PointCloud<PointT>::ConstPtr prev_ground_image;
  pcl::PointCloud<PointT>::ConstPtr prev_label_image;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  std::mutex cloud_mutex;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int files_idx;
  int images_idx;

  pcl::AdaptiveCostSOStereoMatching stereo;
  bool trigger;
  bool continuous;
  bool display_normals;
  bool detect_obstacles;
  int smooth_weak;
  int smooth_strong;

public:
  HRCSSegmentation(const std::vector<std::string>& left_images,
                   const std::vector<std::string>& right_images)
  : viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
  , image_viewer(new pcl::visualization::ImageViewer("Image Viewer"))
  , prev_cloud(new pcl::PointCloud<PointT>)
  , prev_normal_cloud(new pcl::PointCloud<pcl::Normal>)
  , prev_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  , prev_ground_image(new pcl::PointCloud<PointT>)
  , prev_label_image(new pcl::PointCloud<PointT>)
  , road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>)
  , road_segmentation(road_comparator)
  {
    trigger = true;
    continuous = false;
    display_normals = false;
    detect_obstacles = false;

    this->left_images = left_images;
    this->right_images = right_images;
    files_idx = 0;
    images_idx = 0;

    // Set up a 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(
        &HRCSSegmentation::keyboardCallback, *this, nullptr);

    // Set up the stereo matching
    stereo.setMaxDisparity(60);
    stereo.setXOffset(0);
    stereo.setRadius(5);

    smooth_weak = 20;
    smooth_strong = 100;
    stereo.setSmoothWeak(smooth_weak);
    stereo.setSmoothStrong(smooth_strong);
    stereo.setGammaC(25);
    stereo.setGammaS(10);

    stereo.setRatioFilter(20);
    stereo.setPeakFilter(0);

    stereo.setLeftRightCheck(true);
    stereo.setLeftRightCheckThreshold(1);

    stereo.setPreProcessing(true);

    // Set up the normal estimation
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.03f);
    ne.setNormalSmoothingSize(40.0f); // 20.0f

    // Set up the groundplane comparator
    // If the camera was pointing straight out, the normal would be:
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
    // Adjust for camera tilt:
    Eigen::Vector3f tilt_road_normal =
        Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) *
        nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);
    road_comparator->setGroundAngularThreshold(pcl::deg2rad(10.0f));
    road_comparator->setAngularThreshold(pcl::deg2rad(3.0f));
  }

  ~HRCSSegmentation() = default;

  void
  cloud_cb_(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    if (!viewer->wasStopped()) {
      cloud_mutex.lock();
      prev_cloud = cloud;
      cloud_mutex.unlock();
    }
  }

  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp()) {
      switch (event.getKeyCode()) {
      case ' ':
        trigger = true;
        break;
      case '1':
        smooth_strong -= 10;
        PCL_INFO("smooth_strong: %d\n", smooth_strong);
        stereo.setSmoothStrong(smooth_strong);
        break;
      case '2':
        smooth_strong += 10;
        PCL_INFO("smooth_strong: %d\n", smooth_strong);
        stereo.setSmoothStrong(smooth_strong);
        break;
      case '3':
        smooth_weak -= 10;
        PCL_INFO("smooth_weak: %d\n", smooth_weak);
        stereo.setSmoothWeak(smooth_weak);
        break;
      case '4':
        smooth_weak += 10;
        PCL_INFO("smooth_weak: %d\n", smooth_weak);
        stereo.setSmoothWeak(smooth_weak);
        break;
      case 'c':
        continuous = !continuous;
        break;
      case 'n':
        display_normals = !display_normals;
        break;
      case 'o':
        detect_obstacles = !detect_obstacles;
        break;
      }
    }
  }

  void
  processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image,
                    const pcl::PointCloud<pcl::RGB>::Ptr& right_image,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
  {
    stereo.compute(*left_image, *right_image);
    stereo.medianFilter(4);
    stereo.getPointCloud(
        318.112200f, 224.334900f, 368.534700f, 0.8387445f, out_cloud, left_image);
  }

  void
  processCloud(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    // Compute the normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*normal_cloud);

    // Set up the groundplane comparator
    road_comparator->setInputCloud(cloud);
    road_comparator->setInputNormals(normal_cloud);

    // Run segmentation
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> region_indices;
    road_segmentation.setInputCloud(cloud);
    road_segmentation.segment(labels, region_indices);

    // Draw the segmentation result
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_image(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    *ground_image = *cloud;
    *label_image = *cloud;

    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
    Eigen::Vector4f vp = Eigen::Vector4f::Zero();
    Eigen::Matrix3f clust_cov;
    pcl::ModelCoefficients model;
    model.values.resize(4);

    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> covariances;
    std::vector<pcl::PointIndices> inlier_indices;

    for (const auto& region_index : region_indices) {
      if (region_index.indices.size() > 1000) {

        for (std::size_t j = 0; j < region_index.indices.size(); j++) {
          pcl::PointXYZ ground_pt((*cloud)[region_index.indices[j]].x,
                                  (*cloud)[region_index.indices[j]].y,
                                  (*cloud)[region_index.indices[j]].z);
          ground_cloud->points.push_back(ground_pt);
          (*ground_image)[region_index.indices[j]].g = static_cast<std::uint8_t>(
              ((*cloud)[region_index.indices[j]].g + 255) / 2);
          (*label_image)[region_index.indices[j]].r = 0;
          (*label_image)[region_index.indices[j]].g = 255;
          (*label_image)[region_index.indices[j]].b = 0;
        }

        // Compute plane info
        pcl::computeMeanAndCovarianceMatrix(
            *cloud, region_index.indices, clust_cov, clust_centroid);
        Eigen::Vector4f plane_params;

        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33(clust_cov, eigen_value, eigen_vector);
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot(clust_centroid);

        vp -= clust_centroid;
        float cos_theta = vp.dot(plane_params);
        if (cos_theta < 0) {
          plane_params *= -1;
          plane_params[3] = 0;
          plane_params[3] = -1 * plane_params.dot(clust_centroid);
        }

        model.values[0] = plane_params[0];
        model.values[1] = plane_params[1];
        model.values[2] = plane_params[2];
        model.values[3] = plane_params[3];
        model_coefficients.push_back(model);
        inlier_indices.push_back(region_index);
        centroids.push_back(clust_centroid);
        covariances.push_back(clust_cov);
      }
    }

    // Refinement
    std::vector<bool> grow_labels;
    std::vector<int> label_to_model;
    grow_labels.resize(region_indices.size(), false);
    label_to_model.resize(region_indices.size(), 0);

    for (std::size_t i = 0; i < model_coefficients.size(); i++) {
      int model_label = (labels)[inlier_indices[i].indices[0]].label;
      label_to_model[model_label] = static_cast<int>(i);
      grow_labels[model_label] = true;
    }

    pcl::PointCloud<pcl::Label>::Ptr labels_ptr(new pcl::PointCloud<pcl::Label>);
    *labels_ptr = labels;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr
        refinement_compare(
            new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
    refinement_compare->setInputCloud(cloud);
    refinement_compare->setDistanceThreshold(0.15f);
    refinement_compare->setLabels(labels_ptr);
    refinement_compare->setModelCoefficients(model_coefficients);
    refinement_compare->setRefineLabels(grow_labels);
    refinement_compare->setLabelToModel(label_to_model);
    mps.setRefinementComparator(refinement_compare);
    mps.setMinInliers(500);
    mps.setAngularThreshold(pcl::deg2rad(3.0));
    mps.setDistanceThreshold(0.02);
    mps.setInputCloud(cloud);
    mps.setInputNormals(normal_cloud);
    mps.refine(model_coefficients, inlier_indices, labels_ptr, region_indices);

    // Note the regions that have been extended
    pcl::PointCloud<PointT> extended_ground_cloud;
    for (const auto& region_index : region_indices) {
      if (region_index.indices.size() > 1000) {
        for (std::size_t j = 0; j < region_index.indices.size(); j++) {
          // Check to see if it has already been labeled
          if ((*ground_image)[region_index.indices[j]].g ==
              (*ground_image)[region_index.indices[j]].b) {
            pcl::PointXYZ ground_pt((*cloud)[region_index.indices[j]].x,
                                    (*cloud)[region_index.indices[j]].y,
                                    (*cloud)[region_index.indices[j]].z);
            ground_cloud->points.push_back(ground_pt);
            (*ground_image)[region_index.indices[j]].r = static_cast<std::uint8_t>(
                ((*cloud)[region_index.indices[j]].r + 255) / 2);
            (*ground_image)[region_index.indices[j]].g = static_cast<std::uint8_t>(
                ((*cloud)[region_index.indices[j]].g + 255) / 2);
            (*label_image)[region_index.indices[j]].r = 128;
            (*label_image)[region_index.indices[j]].g = 128;
            (*label_image)[region_index.indices[j]].b = 0;
          }
        }
      }
    }

    // Segment Obstacles (Disabled by default)
    Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);

    if (!ground_cloud->points.empty()) {
      ground_centroid = centroids[0];
      ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0],
                                            model_coefficients[0].values[1],
                                            model_coefficients[0].values[2],
                                            model_coefficients[0].values[3]);
    }

    if (detect_obstacles) {
      pcl::PointCloud<PointT>::CloudVectorType clusters;
      if (!ground_cloud->points.empty()) {
        pcl::EuclideanClusterComparator<PointT, pcl::Label>::ExcludeLabelSetPtr
            plane_labels(
                new pcl::EuclideanClusterComparator<PointT,
                                                    pcl::Label>::ExcludeLabelSet);
        for (std::size_t i = 0; i < region_indices.size(); ++i)
          if ((region_indices[i].indices.size() > mps.getMinInliers()))
            plane_labels->insert(i);

        pcl::EuclideanClusterComparator<PointT, pcl::Label>::Ptr
            euclidean_cluster_comparator_(
                new pcl::EuclideanClusterComparator<PointT, pcl::Label>());
        euclidean_cluster_comparator_->setInputCloud(cloud);
        euclidean_cluster_comparator_->setLabels(labels_ptr);
        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);

        pcl::PointCloud<pcl::Label> euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
            euclidean_segmentation(euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud(cloud);
        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

        for (const auto& euclidean_label_index : euclidean_label_indices) {
          if ((euclidean_label_index.indices.size() > 200)) {
            pcl::PointCloud<PointT> cluster;
            pcl::copyPointCloud(*cloud, euclidean_label_index.indices, cluster);
            clusters.push_back(cluster);

            Eigen::Vector4f cluster_centroid;
            Eigen::Matrix3f cluster_cov;
            pcl::computeMeanAndCovarianceMatrix(
                *cloud, euclidean_label_index.indices, cluster_cov, cluster_centroid);

            pcl::PointXYZ centroid_pt(
                cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
            double ptp_dist = pcl::pointToPlaneDistanceSigned(centroid_pt,
                                                              ground_plane_params[0],
                                                              ground_plane_params[1],
                                                              ground_plane_params[2],
                                                              ground_plane_params[3]);

            if ((ptp_dist > 0.5) && (ptp_dist < 3.0)) {

              for (std::size_t j = 0; j < euclidean_label_index.indices.size(); j++) {
                (*ground_image)[euclidean_label_index.indices[j]].r = 255;
                (*label_image)[euclidean_label_index.indices[j]].r = 255;
                (*label_image)[euclidean_label_index.indices[j]].g = 0;
                (*label_image)[euclidean_label_index.indices[j]].b = 0;
              }
            }
          }
        }
      }
    }

    // note the NAN points in the image as well
    for (std::size_t i = 0; i < cloud->size(); i++) {
      if (!pcl::isFinite((*cloud)[i])) {
        (*ground_image)[i].b = static_cast<std::uint8_t>(((*cloud)[i].b + 255) / 2);
        (*label_image)[i].r = 0;
        (*label_image)[i].g = 0;
        (*label_image)[i].b = 255;
      }
    }

    // Update info for the visualization thread
    {
      cloud_mutex.lock();
      prev_cloud = cloud;
      prev_normal_cloud = normal_cloud;
      prev_ground_cloud = ground_cloud;
      prev_ground_image = ground_image;
      prev_label_image = label_image;
      prev_ground_normal = ground_plane_params;
      prev_ground_centroid = ground_centroid;
      cloud_mutex.unlock();
    }
  }

  void
  run()
  {
    while (!viewer->wasStopped()) {
      // Process a new image
      if (trigger || continuous) {
        pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PCDReader pcd;
        pcd.read(left_images[images_idx], *left_cloud);
        pcd.read(right_images[images_idx], *right_cloud);
        processStereoPair(left_cloud, right_cloud, out_cloud);
        processCloud(out_cloud);
        images_idx++;

        trigger = false;
      }

      // Draw visualizations
      if (cloud_mutex.try_lock()) {
        if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
          viewer->addPointCloud(prev_ground_image, "cloud");

        if (prev_normal_cloud->size() > 1000 && display_normals) {
          viewer->removePointCloud("normals");
          viewer->addPointCloudNormals<PointT, pcl::Normal>(
              prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
        }

        if (prev_cloud->size() > 1000) {
          image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
        }

        // Show the groundplane normal
        Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
        // Adjust for camera tilt
        Eigen::Vector3f tilt_road_normal =
            Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) *
            nominal_road_normal;

        // Show the groundplane normal
        pcl::PointXYZ np1(
            prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
        pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                          prev_ground_centroid[1] + prev_ground_normal[1],
                          prev_ground_centroid[2] + prev_ground_normal[2]);
        pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                          prev_ground_centroid[1] + tilt_road_normal[1],
                          prev_ground_centroid[2] + tilt_road_normal[2]);

        viewer->removeShape("ground_norm");
        viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
        viewer->removeShape("expected_ground_norm");
        viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

        cloud_mutex.unlock();
      }
      viewer->spinOnce(100);
      image_viewer->spinOnce(100);
    }
  }
};

int
main(int argc, char** argv)
{
  if (argc < 3) {
    PCL_INFO("usage: pcl_stereo_ground_segmentation left_image_directory "
             "right_image_directory\n");
    PCL_INFO("note: images must be in PCD format.  See pcl_png2pcd\n");
  }

  // Get list of stereo files
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr) {
    left_images.push_back(itr->path().string());
  }
  sort(left_images.begin(), left_images.end());
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv[2]); itr != end_itr; ++itr) {
    right_images.push_back(itr->path().string());
  }
  sort(right_images.begin(), right_images.end());

  PCL_INFO(
      "Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  // Process and display
  HRCSSegmentation hrcs(left_images, right_images);
  hrcs.run();

  return 0;
}
