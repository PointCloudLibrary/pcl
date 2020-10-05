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

#pragma once

#include <pcl/apps/nn_classification.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <fstream>

namespace pcl {

/**
 * \brief Helper function to extract the VFH feature describing the given point cloud.
 * \param points point cloud for feature extraction
 * \param radius search radius for normal estimation
 * \return point cloud containing the extracted feature
 */
template <typename PointT>
pcl::PointCloud<pcl::VFHSignature308>::Ptr
computeVFH(typename PointCloud<PointT>::ConstPtr cloud, double radius)
{
  using namespace pcl;

  // Create an empty kdtree representation, and pass it to the objects.
  // Its content will be filled inside the object, based on the given input dataset
  // (as no other search surface is given).
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  // Create the normal estimation class, and pass the input dataset to it
  NormalEstimation<PointT, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);

  // Use all neighbors in a sphere of given radius to compute the normals
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
  VFHEstimation<PointT, Normal, VFHSignature308> vfh;
  vfh.setInputCloud(cloud);
  vfh.setInputNormals(normals);
  vfh.setSearchMethod(tree);

  // Output datasets
  PointCloud<VFHSignature308>::Ptr vfhs(new PointCloud<VFHSignature308>);

  // Compute the features and return
  vfh.compute(*vfhs);
  return vfhs;
}

/**
 * \brief Utility class for nearest neighbor search based classification of VFH
 * features.
 * \author Zoltan Csaba Marton
 */
class VFHClassifierNN {
public:
  using FeatureCloud = pcl::PointCloud<pcl::VFHSignature308>;
  using FeatureCloudPtr = pcl::PointCloud<pcl::VFHSignature308>::Ptr;
  using FeatureCloudConstPtr = pcl::PointCloud<pcl::VFHSignature308>::ConstPtr;
  using Result = NNClassification<pcl::VFHSignature308>::Result;
  using ResultPtr = NNClassification<pcl::VFHSignature308>::ResultPtr;

private:
  /** \brief Point cloud containing the training VFH features */
  FeatureCloudPtr training_features_;
  /** \brief Class label for each training example */
  std::vector<std::string> labels_;
  /** \brief Nearest neighbor classifier instantiated for VFH features */
  NNClassification<pcl::VFHSignature308> classifier_;

public:
  VFHClassifierNN() { reset(); }

  void
  reset()
  {
    training_features_.reset(new FeatureCloud);
    labels_.clear();
    classifier_ = NNClassification<pcl::VFHSignature308>();
  }

  /** \brief Set up the classifier with the current training features and labels */
  void
  finalizeTraining()
  {
    finalizeTree();
    finalizeLabels();
  }

  /** \brief Set up the classifier with the current training features */
  void
  finalizeTree()
  {
    classifier_.setTrainingFeatures(training_features_);
  }

  /** \brief Set up the classifier with the current training example labels */
  void
  finalizeLabels()
  {
    classifier_.setTrainingLabels(labels_);
  }

  /**
   * \brief Save the list of training examples and corresponding labels.
   * \param file_name file name for writing the training features
   * \param labels_file_name file name for writing the class label for each
   * training example
   * \return true on success, false on failure (write error or number of entries
   * don't match)
   */
  bool
  saveTrainingFeatures(const std::string& file_name,
                       const std::string& labels_file_name)
  {
    if (labels_.size() == training_features_->size()) {
      if (pcl::io::savePCDFile(file_name, *training_features_) != 0)
        return false;
      std::ofstream f(labels_file_name.c_str());
      for (const auto& s : labels_) {
        f << s << "\n";
      }
      return true;
    }
    return false;
  }

  /**
   * \brief Fill the list of training examples and corresponding labels.
   * \note this function has a cumulative effect.
   * \param training_features the training features
   * \param labels the class label for each training example
   * \return true on success, false on failure (number of entries don't match)
   */
  bool
  addTrainingFeatures(const FeatureCloudPtr& training_features,
                      const std::vector<std::string>& labels)
  {
    if (labels.size() == training_features->size()) {
      labels_.insert(labels_.end(), labels.begin(), labels.end());
      training_features_->points.insert(training_features_->points.end(),
                                        training_features->points.begin(),
                                        training_features->points.end());
      training_features_->header = training_features->header;
      training_features_->height = 1;
      training_features_->width = training_features_->size();
      training_features_->is_dense &= training_features->is_dense;
      training_features_->sensor_origin_ = training_features->sensor_origin_;
      training_features_->sensor_orientation_ = training_features->sensor_orientation_;
      return true;
    }
    return false;
  }

  /**
   * \brief Fill the list of training examples and corresponding labels.
   * \note this function has a cumulative effect.
   * \param file_name PCD file containing the training features
   * \param labels_file_name the class label for each training example
   * \return true on success, false on failure (read error or number of entries don't
   * match)
   */
  bool
  loadTrainingFeatures(const std::string& file_name,
                       const std::string& labels_file_name)
  {
    FeatureCloudPtr cloud(new FeatureCloud);
    if (pcl::io::loadPCDFile(file_name, *cloud) != 0)
      return false;
    std::vector<std::string> labels;
    std::ifstream f(labels_file_name.c_str());
    std::string label;
    while (getline(f, label))
      if (!label.empty())
        labels.push_back(label);
    return addTrainingFeatures(cloud, labels);
  }

  /**
   * \brief Add the feature extracted from the cloud at the specified
   * location as a training example with the given labels.
   * \note this function has a cumulative effect.
   * \param file_name PCD file containing the training data
   * \param label the class label for the training example
   * \return true on success, false on failure (read error or number of entries don't
   * match)
   */
  bool
  loadTrainingData(const std::string& file_name, std::string label)
  {
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile(file_name, cloud_blob) != 0)
      return false;
    return addTrainingData(cloud_blob, label);
  }

  /**
   * \brief Add the feature extracted from the cloud as a training example with the
   * given labels.
   * \note this function has a cumulative effect.
   * \param training_data point cloud for training feature extraction
   * \param label the class label for the training example
   * \return true on success, false on failure (read error or number of entries
   * don't match)
   */
  bool
  addTrainingData(const pcl::PCLPointCloud2& training_data, std::string& label)
  {
    // Create label list containing the single label
    std::vector<std::string> labels;
    labels.push_back(label);

    // Compute the feature from the cloud and add it as a training example
    FeatureCloudPtr vfhs = computeFeature(training_data);
    return addTrainingFeatures(vfhs, labels);
  }

  /**
   * \brief Utility function for the default classification process.
   * \param testing_data the point clouds to be classified
   * \param radius the maximum search radius in feature space -- 300 by default
   * \param minimum_score the score to be given to matches at maximum distance
   * (>0) -- 0.002 by default
   * \return pair of label and score for each relevant training class
   */
  ResultPtr
  classify(const pcl::PCLPointCloud2& testing_data,
           double radius = 300,
           double min_score = 0.002)
  {
    // compute the VFH feature for this point cloud
    FeatureCloudPtr vfhs = computeFeature(testing_data);
    // compute gaussian parameter producing the desired minimum score
    // (around 50 for the default values)
    float gaussian_param = -static_cast<float>(radius / std::log(min_score));
    // TODO accept result to be filled in by reference
    return classifier_.classify(vfhs->points.at(0), radius, gaussian_param);
  }

  /**
   * \brief Extract the VFH feature describing the given point cloud.
   * \param points point cloud for feature extraction
   * \param radius search radius for normal estimation -- 0.03 m by default
   * \return point cloud containing the extracted feature
   */
  FeatureCloudPtr
  computeFeature(const pcl::PCLPointCloud2& points, double radius = 0.03)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(points, *cloud);
    return pcl::computeVFH<pcl::PointXYZ>(cloud, radius);
  }
};

} // namespace pcl
