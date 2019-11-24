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
 * height_map_2d.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#pragma once

#include <pcl/people/person_cluster.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace people
  {
    /** \brief @b HeightMap2D represents a class for creating a 2D height map from a point cloud and searching for its local maxima
      * \author Matteo Munaro
      * \ingroup people
    */
    template <typename PointT> class HeightMap2D;

    template <typename PointT>
    class HeightMap2D
    {
    public:

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Constructor. */
      HeightMap2D();

      /** \brief Destructor. */
      virtual ~HeightMap2D ();

      /**
       * \brief Compute the height map with the projection of cluster points onto the ground plane.
       * 
       * \param[in] cluster The PersonCluster used to compute the height map.
       */
      void
      compute (pcl::people::PersonCluster<PointT>& cluster);

      /**
       * \brief Compute local maxima of the height map.
       */
      void
      searchLocalMaxima ();

      /**
       * \brief Filter maxima of the height map by imposing a minimum distance between them.
       */
      void
      filterMaxima ();

      /**
       * \brief Set initial cluster indices.
       * 
       * \param[in] cloud A pointer to the input cloud.
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
       * \brief Set bin size for the height map. 
       * 
       * \param[in] bin_size Bin size for the height map (default = 0.06).
       */
      void
      setBinSize (float bin_size);

      /**
       * \brief Set minimum distance between maxima. 
       * 
       * \param[in] minimum_distance_between_maxima Minimum allowed distance between maxima (default = 0.3).
       */
      void
      setMinimumDistanceBetweenMaxima (float minimum_distance_between_maxima);

      /**
       * \brief Set sensor orientation to landscape mode (false) or portrait mode (true).
       * 
       * \param[in] vertical Landscape (false) or portrait (true) mode (default = false).
       */
      void
      setSensorPortraitOrientation (bool vertical);

      /**
       * \brief Get the height map as a vector of int.
       */
      std::vector<int>&
      getHeightMap ();

      /**
       * \brief Get bin size for the height map. 
       */
      float
      getBinSize ();

      /**
       * \brief Get minimum distance between maxima of the height map. 
       */
      float
      getMinimumDistanceBetweenMaxima ();

      /**
       * \brief Return the maxima number after the filterMaxima method.
       */
      int&
      getMaximaNumberAfterFiltering ();

      /**
       * \brief Return the point cloud indices corresponding to the maxima computed after the filterMaxima method.
       */
      std::vector<int>&
      getMaximaCloudIndicesFiltered ();

    protected:
      /** \brief ground plane coefficients */
      Eigen::VectorXf ground_coeffs_;            
      
      /** \brief ground plane normalization factor */
      float sqrt_ground_coeffs_;              
      
      /** \brief pointer to the input cloud */
      PointCloudPtr cloud_;                
      
      /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
      bool vertical_;                    
      
      /** \brief vector with maximum height values for every bin (height map) */
      std::vector<int> buckets_;              
      
      /** \brief indices of the pointcloud points with maximum height for every bin */
      std::vector<int> buckets_cloud_indices_;      
      
      /** \brief bin dimension */
      float bin_size_;                  
      
      /** \brief number of local maxima in the height map */
      int maxima_number_;                  
      
      /** \brief contains the position of the maxima in the buckets vector */
      std::vector<int> maxima_indices_;          
      
      /** \brief contains the point cloud position of the maxima (indices of the point cloud) */
      std::vector<int> maxima_cloud_indices_;        
      
      /** \brief number of local maxima after filtering */
      int maxima_number_after_filtering_;          
      
      /** \brief contains the position of the maxima in the buckets array after filtering */
      std::vector<int> maxima_indices_filtered_;      
      
      /** \brief contains the point cloud position of the maxima after filtering */
      std::vector<int> maxima_cloud_indices_filtered_;  
      
      /** \brief minimum allowed distance between maxima */
      float min_dist_between_maxima_;            
    };

  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/height_map_2d.hpp>
