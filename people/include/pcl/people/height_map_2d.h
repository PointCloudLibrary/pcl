/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_PEOPLE_HEIGHT_MAP_2D_H_
#define PCL_PEOPLE_HEIGHT_MAP_2D_H_

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

			typedef pcl::PointCloud<PointT> PointCloud;
			typedef boost::shared_ptr<PointCloud> PointCloudPtr;
			typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

			HeightMap2D();

			virtual ~HeightMap2D ();

			/**
			 * Compute the height map with the projection of cluster points onto the ground plane.
			 * @compute the height map with the projection of cluster points onto the ground plane.
			 */
			void
			compute (pcl::people::PersonCluster<PointT>& cluster);

			/**
			 * Compute local maxima of the height map.
			 * @compute local maxima of the height map.
			 */
			void
			searchLocalMaxima ();

			/**
			 * Filter maxima of the height map by imposing a minimum distance between them.
			 * @filter local maxima of the height map by imposing a minimum distance between them.
			 */
			void
			filterMaxima ();

			/**
			 * Set initial cluster indices.
			 * @set initial cluster indices.
			 */
			void
			setInputCloud (PointCloudPtr& cloud);

			/**
			 * Set the ground coefficients.
			 * @set the ground coefficients.
			 */
			void
			setGround (Eigen::VectorXf& ground_coeffs);

			/**
			 * Set bin size for the height map. Default = 0.06 meters.
			 * @set bin size for the height map. Default = 0.06 meters.
			 */
			void
			setBinSize (float bin_size);

			/**
			 * Set minimum distance between maxima. Default = 0.3 meters.
			 * @set minimum distance between maxima. Default = 0.3 meters.
			 */
			void
			setMinimumDistanceBetweenMaxima (float minimum_distance_between_maxima);

			/**
			 * Set sensor orientation (vertical = true means portrait mode, vertical = false means landscape mode). Default = false.
			 * @set sensor orientation (vertical = true means portrait mode, vertical = false means landscape mode). Default = false.
			 */
			void
			setSensorPortraitOrientation (bool vertical);

			/**
			 * Get the height map as a vector of int.
			 * @get the height map as a vector of int.
			 */
			std::vector<int>&
			getHeightMap ();

			/**
			 * Get bin size for the height map. Default = 0.06 meters.
			 * @get bin size for the height map. Default = 0.06 meters.
			 */
			float
			getBinSize ();

			/**
			 * Get minimum distance between maxima. Default = 0.3 meters.
			 * @get minimum distance between maxima. Default = 0.3 meters.
			 */
			float
			getMinimumDistanceBetweenMaxima ();

			/**
			 * Return maxima_number_after_filtering_.
			 * @return maxima_number_after_filtering_.
			 */
			int&
			getMaximaNumberAfterFiltering ();

			/**
			 * Return maxima_cloud_indices_filtered_.
			 * @return maxima_cloud_indices_filtered_.
			 */
			std::vector<int>&
			getMaximaCloudIndicesFiltered ();

		protected:
			Eigen::VectorXf ground_coeffs_;						// ground plane coefficients
			float sqrt_ground_coeffs_;							// ground plane normalization factor
			PointCloudPtr cloud_;								// pointer to the input cloud
			bool vertical_;										// if true, the sensor is considered to be vertically placed (portrait mode)
			std::vector<int> buckets_;							// vector with maximum height values for every bin (height map)
			std::vector<int> buckets_cloud_indices_;			// indices of the pointcloud points with maximum height for every bin
			float bin_size_;									// bin dimension
			int maxima_number_;									// number of local maxima in the height map
			std::vector<int> maxima_indices_;					// contains the position of the maxima in the buckets vector
			std::vector<int> maxima_cloud_indices_;				// contains the point cloud position of the maxima (indices of the point cloud)
			int maxima_number_after_filtering_;					// number of local maxima after filtering
			std::vector<int> maxima_indices_filtered_;			// contains the position of the maxima in the buckets array after filtering
			std::vector<int> maxima_cloud_indices_filtered_;	// contains the point cloud position of the maxima after filtering
			float min_dist_between_maxima_;						// minimum allowed distance between maxima
		};

	} /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/height_map_2d.hpp>
#endif /* PCL_PEOPLE_HEIGHT_MAP_2D_H_ */
