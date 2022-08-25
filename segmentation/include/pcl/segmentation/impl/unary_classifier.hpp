/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_UNARY_CLASSIFIER_HPP_
#define PCL_UNARY_CLASSIFIER_HPP_

#include <Eigen/Core>
#include <flann/flann.hpp>                  // for flann::Index
#include <flann/algorithms/dist.h>          // for flann::ChiSquareDistance
#include <flann/algorithms/linear_index.h>  // for flann::LinearIndexParams
#include <flann/util/matrix.h>              // for flann::Matrix

#include <pcl/features/normal_3d.h> // for NormalEstimation
#include <pcl/segmentation/unary_classifier.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::UnaryClassifier<PointT>::UnaryClassifier () :
  input_cloud_ (new pcl::PointCloud<PointT>),
  label_field_ (false),
  normal_radius_search_ (0.01f),
  fpfh_radius_search_ (0.05f),
  feature_threshold_ (5.0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::UnaryClassifier<PointT>::~UnaryClassifier () = default;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::setInputCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud)
{
  input_cloud_ = input_cloud;

  pcl::PointCloud <PointT> point;
  std::vector<pcl::PCLPointField> fields;

  int label_index = -1;
  label_index = pcl::getFieldIndex<PointT> ("label", fields);
  
  if (label_index != -1)
    label_field_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::convertCloud (typename pcl::PointCloud<PointT>::Ptr in,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
  // resize points of output cloud
  out->points.resize (in->size ());
  out->width = out->size ();
  out->height = 1;
  out->is_dense = false;

  for (std::size_t i = 0; i < in->size (); i++)
  {
    pcl::PointXYZ point;
    // fill X Y Z
    point.x = (*in)[i].x;
    point.y = (*in)[i].y;
    point.z = (*in)[i].z;
    (*out)[i] = point;
  }
}

template <typename PointT> void
pcl::UnaryClassifier<PointT>::convertCloud (typename pcl::PointCloud<PointT>::Ptr in,
                                            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr out)
{
  // TODO:: check if input cloud has RGBA information and insert into the cloud

  // resize points of output cloud
  out->points.resize (in->size ());
  out->width = out->size ();
  out->height = 1;
  out->is_dense = false;

  for (std::size_t i = 0; i < in->size (); i++)
  {
    pcl::PointXYZRGBL point;
    // X Y Z R G B L
    point.x = (*in)[i].x;
    point.y = (*in)[i].y;
    point.z = (*in)[i].z;
    //point.rgba = (*in)[i].rgba;
    point.label = 1;
    (*out)[i] = point;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::findClusters (typename pcl::PointCloud<PointT>::Ptr in,
                                            std::vector<int> &cluster_numbers)
{
  // find the 'label' field index
  std::vector <pcl::PCLPointField> fields;
  const int label_idx = pcl::getFieldIndex<PointT> ("label", fields);

  if (label_idx != -1)
  {
    for (const auto& point: *in)
    {
      // get the 'label' field                                                                       
      std::uint32_t label;
      memcpy (&label, reinterpret_cast<const char*> (&point) + fields[label_idx].offset, sizeof(std::uint32_t));

      // check if label exist
      bool exist = false;
      for (const int &cluster_number : cluster_numbers)
      {
        if (static_cast<std::uint32_t> (cluster_number) == label)
        {
          exist = true;
          break;
        }
      }
      if (!exist)
        cluster_numbers.push_back (label);
    }    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::getCloudWithLabel (typename pcl::PointCloud<PointT>::Ptr in,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr out,
                                                 int label_num)
{
  // find the 'label' field index
  std::vector <pcl::PCLPointField> fields;
  int label_idx = -1;
  label_idx = pcl::getFieldIndex<PointT> ("label", fields);

  if (label_idx != -1)
  {
    for (const auto& point : (*in))
    {
      // get the 'label' field                                                                       
      std::uint32_t label;
      memcpy (&label, reinterpret_cast<const char*> (&point) + fields[label_idx].offset, sizeof(std::uint32_t));

      if (static_cast<int> (label) == label_num)
      {
        pcl::PointXYZ tmp;
        // X Y Z
        tmp.x = point.x;
        tmp.y = point.y;
        tmp.z = point.z;
        out->push_back (tmp);
      }
    }
    out->width = out->size ();
    out->height = 1;
    out->is_dense = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::computeFPFH (pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr out,
                                           float normal_radius_search,
                                           float fpfh_radius_search)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr normals_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;

  n3d.setRadiusSearch (normal_radius_search);
  n3d.setSearchMethod (normals_tree);
  // ---[ Estimate the point normals
  n3d.setInputCloud (in);
  n3d.compute (*normals);

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (in);
  fpfh.setInputNormals (normals);
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh.setSearchMethod (tree);
  fpfh.setRadiusSearch (fpfh_radius_search);
  // Compute the features
  fpfh.compute (*out);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::kmeansClustering (pcl::PointCloud<pcl::FPFHSignature33>::Ptr in,
                                                pcl::PointCloud<pcl::FPFHSignature33>::Ptr out,
                                                int k)
{
  pcl::Kmeans kmeans (static_cast<int> (in->size ()), 33);
  kmeans.setClusterSize (k);

  // add points to the clustering
  for (const auto &point : in->points)
  {
    std::vector<float> data (33);
    for (int idx = 0; idx < 33; idx++)
      data[idx] = point.histogram[idx];
    kmeans.addDataPoint (data);
  }

  // k-means clustering
  kmeans.kMeans ();

  // get the cluster centroids
  pcl::Kmeans::Centroids centroids = kmeans.get_centroids ();

  // initialize output cloud
  out->width = centroids.size ();
  out->height = 1;
  out->is_dense = false;
  out->points.resize (static_cast<int> (centroids.size ()));
  // copy cluster centroids into feature cloud 
  for (std::size_t i = 0; i < centroids.size (); i++)
  {
    pcl::FPFHSignature33 point;
    for (int idx = 0; idx < 33; idx++)
      point.histogram[idx] = centroids[i][idx];
    (*out)[i] = point;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::queryFeatureDistances (std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &trained_features,
                                                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr query_features,
                                                     pcl::Indices &indi,
                                                     std::vector<float> &dist)
{
  // estimate the total number of row's needed
  int n_row = 0;
  for (const auto &trained_feature : trained_features)
    n_row += static_cast<int> (trained_feature->size ());

  // Convert data into FLANN format
  int n_col = 33;
  flann::Matrix<float> data (new float[n_row * n_col], n_row, n_col);
  for (std::size_t k = 0; k < trained_features.size (); k++)
  {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr hist = trained_features[k];
    const auto c = hist->size ();
    for (std::size_t i = 0; i < c; ++i)
      for (std::size_t j = 0; j < data.cols; ++j)
        data[(k * c) + i][j] = (*hist)[i].histogram[j];
  }

  // build kd-tree given the training features
  flann::Index<flann::ChiSquareDistance<float> > *index;
  index = new flann::Index<flann::ChiSquareDistance<float> > (data, flann::LinearIndexParams ());
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KMeansIndexParams (5, -1));  
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index->buildIndex ();

  int k = 1;
  indi.resize (query_features->size ());
  dist.resize (query_features->size ());
  // Query all points
  for (std::size_t i = 0; i < query_features->size (); i++)
  {
    // Query point  
    flann::Matrix<float> p = flann::Matrix<float>(new float[n_col], 1, n_col);
    std::copy((*query_features)[i].histogram, (*query_features)[i].histogram + n_col, p.ptr());

    flann::Matrix<int> indices (new int[k], 1, k);
    flann::Matrix<float> distances (new float[k], 1, k);  
    index->knnSearch (p, indices, distances, k, flann::SearchParams (512));

    indi[i] = indices[0][0];
    dist[i] = distances[0][0];

    delete[] p.ptr ();
  }

  //std::cout << "kdtree size: " << index->size () << std::endl;

  delete[] data.ptr ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::assignLabels (pcl::Indices &indi,
                                            std::vector<float> &dist,
                                            int n_feature_means,
                                            float feature_threshold,
                                            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr out)
                              
{
  float nfm = static_cast<float> (n_feature_means);
  for (std::size_t i = 0; i < out->size (); i++)
  {
    if (dist[i] < feature_threshold)
    {
      float l = static_cast<float> (indi[i]) / nfm;
      float intpart;
      //float fractpart = std::modf (l , &intpart);
      std::modf (l , &intpart);
      int label = static_cast<int> (intpart);
      
      (*out)[i].label = label+2;
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::train (pcl::PointCloud<pcl::FPFHSignature33>::Ptr &output)
{  
  // convert cloud into cloud with XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  convertCloud (input_cloud_, tmp_cloud);

  // compute FPFH feature histograms for all point of the input point cloud
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature (new pcl::PointCloud<pcl::FPFHSignature33>);
  computeFPFH (tmp_cloud, feature, normal_radius_search_, fpfh_radius_search_);

  //PCL_INFO ("Number of input cloud features: %d\n", static_cast<int> (feature->size ()));

  // use k-means to cluster the features
  kmeansClustering (feature, output, cluster_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::trainWithLabel (
    std::vector<pcl::PointCloud<pcl::FPFHSignature33>, Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33> > > &output)
{
  // find clusters
  std::vector<int> cluster_numbers;
  findClusters (input_cloud_, cluster_numbers);
  std::cout << "cluster numbers: ";
  for (const int &cluster_number : cluster_numbers)
    std::cout << cluster_number << " ";
  std::cout << std::endl;

  for (const int &cluster_number : cluster_numbers)
  {    
    // extract all points with the same label number
    pcl::PointCloud<pcl::PointXYZ>::Ptr label_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    getCloudWithLabel (input_cloud_, label_cloud, cluster_number);

    // compute FPFH feature histograms for all point of the input point cloud
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature (new pcl::PointCloud<pcl::FPFHSignature33>);
    computeFPFH (label_cloud, feature, normal_radius_search_, fpfh_radius_search_);

    // use k-means to cluster the features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr kmeans_feature (new pcl::PointCloud<pcl::FPFHSignature33>);
    kmeansClustering (feature, kmeans_feature, cluster_size_);

    output.push_back (*kmeans_feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UnaryClassifier<PointT>::segment (pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &out)
{
  if (!trained_features_.empty ())
  {
    // convert cloud into cloud with XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    convertCloud (input_cloud_, tmp_cloud);

    // compute FPFH feature histograms for all point of the input point cloud
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_cloud_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    computeFPFH (tmp_cloud, input_cloud_features, normal_radius_search_, fpfh_radius_search_);

    // query the distances from the input data features to all trained features
    Indices indices;
    std::vector<float> distance;
    queryFeatureDistances (trained_features_, input_cloud_features, indices, distance);

    // assign a label to each point of the input point cloud
    const auto n_feature_means = trained_features_[0]->size ();
    convertCloud (input_cloud_, out);
    assignLabels (indices, distance, n_feature_means, feature_threshold_, out);
    //std::cout << "Assign labels - DONE" << std::endl;
  }
  else
    PCL_ERROR ("no training features set \n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PCL_INSTANTIATE_UnaryClassifier(T) template class pcl::UnaryClassifier<T>;

#endif    // PCL_UNARY_CLASSIFIER_HPP_
