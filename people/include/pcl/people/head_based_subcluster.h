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
 * head_based_subcluster.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_H_
#define PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_H_

#include <pcl/people/person_cluster.h>
#include <pcl/people/height_map_2d.h>
#include <pcl/point_types.h>

namespace pcl
{
	namespace people
	{
		/** \brief @b HeadBasedSubclustering represents a class for searching for people inside a HeadBasedSubclustering based on a 3D head detection algorithm
			* \author Matteo Munaro
			* \ingroup people
		*/
		template <typename PointT> class HeadBasedSubclustering;

		template <typename PointT>
		class HeadBasedSubclustering
		{
		public:

			typedef pcl::PointCloud<PointT> PointCloud;
			typedef boost::shared_ptr<PointCloud> PointCloudPtr;
			typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

			HeadBasedSubclustering ();

			virtual ~HeadBasedSubclustering ();

			/**
			 * Compute subclusters and return them into a vector of PersonCluster.
			 * @compute subclusters and return them into a vector of PersonCluster.
			 */
			void
			subcluster (std::vector<pcl::people::PersonCluster<PointT> >& clusters);

			/**
			 * Merge clusters close in floor coordinates.
			 * @merge clusters close in floor coordinates.
			 */
			void
			mergeClustersCloseInFloorCoordinates (std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
					std::vector<pcl::people::PersonCluster<PointT> >& output_clusters);

			/**
			 * Create subclusters centered on the heads position from the current cluster.
			 * @create subclusters centered on the heads position from the current cluster.
			 */
			void
			createSubClusters (pcl::people::PersonCluster<PointT>& cluster, int maxima_number_after_filtering_,	std::vector<int>& maxima_cloud_indices_filtered,
					std::vector<pcl::people::PersonCluster<PointT> >& subclusters);

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
			 * Set sensor orientation (vertical = true means portrait mode, vertical = false means landscape mode). Default = false.
			 * @set sensor orientation (vertical = true means portrait mode, vertical = false means landscape mode). Default = false.
			 */
			void
			setSensorPortraitOrientation (bool vertical);

			/**
			 * Set head_centroid_ to true (person centroid is in the head) or false (person centroid is the whole body centroid). Default = true.
			 * @set head_centroid_ to true (person centroid is in the head) or false (person centroid is the whole body centroid). Default = true.
			 */
			void
			setHeadCentroid (bool head_centroid);

			/**
			 * Set initial cluster indices.
			 * @set initial cluster indices.
			 */
			void
			setInitialClusters (std::vector<pcl::PointIndices>& cluster_indices);

			/**
			 * Set minimum and maximum allowed height for a person cluster.
			 * @set minimum and maximum allowed height for a person cluster.
			 */
			void
			setHeightLimits (float min_height, float max_height);

			/**
			 * Set minimum and maximum allowed number of points for a person cluster.
			 * @set minimum and maximum allowed number of points for a person cluster.
			 */
			void
			setDimensionLimits (int min_points, int max_points);

			/**
			 * Set minimum distance between people head. Default = 0.3 meters.
			 * @set minimum distance between people head. Default = 0.3 meters.
			 */
			void
			setMinimumDistanceBetweenHeads (float heads_minimum_distance);

			/**
			 * Get minimum and maximum allowed height for a person cluster.
			 * @get minimum and maximum allowed height for a person cluster.
			 */
			void
			getHeightLimits (float& min_height, float& max_height);

			/**
			 * Get minimum and maximum allowed number of points for a person cluster.
			 * @get minimum and maximum allowed number of points for a person cluster.
			 */
			void
			getDimensionLimits (int& min_points, int& max_points);

			/**
			 * Get minimum distance between people head. Default = 0.3 meters.
			 * @get minimum distance between people head. Default = 0.3 meters.
			 */
			float
			getMinimumDistanceBetweenHeads ();

		protected:
			Eigen::VectorXf ground_coeffs_;						// ground plane coefficients
			float sqrt_ground_coeffs_;							// ground plane normalization factor
			std::vector<pcl::PointIndices> cluster_indices_; 	// initial clusters indices
			PointCloudPtr cloud_;								// pointer to the input cloud
			float max_height_;									// person clusters maximum height from the ground plane;
			float min_height_;									// person clusters minimum height from the ground plane;
			bool vertical_;										// if true, the sensor is considered to be vertically placed (portrait mode)
			bool head_centroid_;								// if true, the person centroid is computed as the centroid of the cluster points belonging to the head (default = false)
																// if false, the person centroid is computed as the centroid of the whole cluster points (default = true)
			int max_points_;									// maximum number of points for a person cluster
			int min_points_;									// minimum number of points for a person cluster
			float heads_minimum_distance_;						// minimum distance between persons' head
		};
	} /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/head_based_subcluster.hpp>
#endif /* PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_H_ */
