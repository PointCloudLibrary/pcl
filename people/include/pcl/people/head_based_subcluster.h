/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * head_based_subcluster.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#pragma once

#include <pcl/people/person_cluster.h>
#include <pcl/people/height_map_2d.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace people
  {
    /** \brief @b HeadBasedSubclustering represents a class for searching for people inside a HeightMap2D based on a 3D head detection algorithm
      * \author Matteo Munaro
      * \ingroup people
    */
    template <typename PointT> class HeadBasedSubclustering;

    template <typename PointT>
    class HeadBasedSubclustering
    {
    public:

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Constructor. */
      HeadBasedSubclustering ();

      /** \brief Destructor. */
      virtual ~HeadBasedSubclustering ();

      /**
       * \brief Compute subclusters and return them into a vector of PersonCluster.
       * 
       * \param[in] clusters Vector of PersonCluster.
       */
      void
      subcluster (std::vector<pcl::people::PersonCluster<PointT> >& clusters);

      /**
       * \brief Merge clusters close in floor coordinates.
       * 
       * \param[in] input_clusters Input vector of PersonCluster.
       * \param[in] output_clusters Output vector of PersonCluster (after merging).
       */
      void
      mergeClustersCloseInFloorCoordinates (std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
          std::vector<pcl::people::PersonCluster<PointT> >& output_clusters);

      /**
       * \brief Create subclusters centered on the heads position from the current cluster.
       * 
       * \param[in] cluster A PersonCluster.
       * \param[in] maxima_number_after_filtering Number of local maxima to use as centers of the new cluster.
       * \param[in] maxima_cloud_indices_filtered Cloud indices of local maxima to use as centers of the new cluster.
       * \param[out] subclusters Output vector of PersonCluster objects derived from the input cluster.
       */
      void
      createSubClusters (pcl::people::PersonCluster<PointT>& cluster, int maxima_number_after_filtering,  std::vector<int>& maxima_cloud_indices_filtered,
          std::vector<pcl::people::PersonCluster<PointT> >& subclusters);

      /**
       * \brief Set input cloud.
       * 
       * \param[in] cloud A pointer to the input point cloud.
       */
      void
      setInputCloud (PointCloudPtr& cloud);

      /**
       * \brief Set the ground coefficients.
       * 
       * \param[in] ground_coeffs The ground plane coefficients.
       */
      void
      setGround (Eigen::VectorXf& ground_coeffs);

      /**
       * \brief Set sensor orientation to landscape mode (false) or portrait mode (true).
       * 
       * \param[in] vertical Landscape (false) or portrait (true) mode (default = false).
       */
      void
      setSensorPortraitOrientation (bool vertical);

      /**
       * \brief Set head_centroid_ to true (person centroid is in the head) or false (person centroid is the whole body centroid).
       *
       * \param[in] head_centroid Set the location of the person centroid (head or body center) (default = true).
       */
      void
      setHeadCentroid (bool head_centroid);

      /**
       * \brief Set initial cluster indices.
       * 
       * \param[in] cluster_indices Point cloud indices corresponding to the initial clusters (before subclustering).
       */
      void
      setInitialClusters (std::vector<pcl::PointIndices>& cluster_indices);

      /**
       * \brief Set minimum and maximum allowed height for a person cluster.
       *
       * \param[in] min_height Minimum allowed height for a person cluster (default = 1.3).
       * \param[in] max_height Maximum allowed height for a person cluster (default = 2.3).
       */
      void
      setHeightLimits (float min_height, float max_height);

      /**
       * \brief Set minimum and maximum allowed number of points for a person cluster.
       *
       * \param[in] min_points Minimum allowed number of points for a person cluster.
       * \param[in] max_points Maximum allowed number of points for a person cluster.
       */
      void
      setDimensionLimits (int min_points, int max_points);

      /**
       * \brief Set minimum distance between persons' heads.
       *
       * \param[in] heads_minimum_distance Minimum allowed distance between persons' heads (default = 0.3).
       */
      void
      setMinimumDistanceBetweenHeads (float heads_minimum_distance);

      /**
       * \brief Get minimum and maximum allowed height for a person cluster.
       *
       * \param[out] min_height Minimum allowed height for a person cluster.
       * \param[out] max_height Maximum allowed height for a person cluster.
       */
      void
      getHeightLimits (float& min_height, float& max_height);

      /**
       * \brief Get minimum and maximum allowed number of points for a person cluster.
       *
       * \param[out] min_points Minimum allowed number of points for a person cluster.
       * \param[out] max_points Maximum allowed number of points for a person cluster.
       */
      void
      getDimensionLimits (int& min_points, int& max_points);

      /**
       * \brief Get minimum distance between persons' heads.
       */
      float
      getMinimumDistanceBetweenHeads ();

    protected:
      /** \brief ground plane coefficients */
      Eigen::VectorXf ground_coeffs_;            
      
      /** \brief ground plane normalization factor */
      float sqrt_ground_coeffs_;              
      
      /** \brief initial clusters indices */
      std::vector<pcl::PointIndices> cluster_indices_;   
      
      /** \brief pointer to the input cloud */
      PointCloudPtr cloud_;                
      
      /** \brief person clusters maximum height from the ground plane */
      float max_height_;                  
      
      /** \brief person clusters minimum height from the ground plane */
      float min_height_;                  
      
      /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
      bool vertical_;                   
      
      /** \brief if true, the person centroid is computed as the centroid of the cluster points belonging to the head 
                 if false, the person centroid is computed as the centroid of the whole cluster points (default = true) */
      bool head_centroid_;                                            
      
      /** \brief maximum number of points for a person cluster */
      int max_points_;                  
      
      /** \brief minimum number of points for a person cluster */
      int min_points_;                  
      
      /** \brief minimum distance between persons' heads */
      float heads_minimum_distance_;           
    };
  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/head_based_subcluster.hpp>
