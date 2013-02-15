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
 * person_cluster.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_PERSON_CLUSTER_H_
#define PCL_PEOPLE_PERSON_CLUSTER_H_

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
	namespace people
	{
		/** \brief @b PersonCluster represents a class for representing information about a cluster containing a person.
			* \author Filippo Basso, Matteo Munaro
			* \ingroup people
		*/
		template <typename PointT> class PersonCluster;
		template <typename PointT> bool operator<(const PersonCluster<PointT>& c1, const PersonCluster<PointT>& c2);

		template <typename PointT>
		class PersonCluster
		{
		protected:

			bool head_centroid_;

			float min_x_;
			float min_y_;
			float min_z_;

			float max_x_;
			float max_y_;
			float max_z_;

			float sum_x_;
			float sum_y_;
			float sum_z_;

			int n_;

			float c_x_;
			float c_y_;
			float c_z_;

			float height_;

			float distance_;
			float angle_;

			float angle_max_;
			float angle_min_;

			Eigen::Vector3f ttop_;
			Eigen::Vector3f tbottom_;
			Eigen::Vector3f tcenter_;

			Eigen::Vector3f top_;
			Eigen::Vector3f bottom_;
			Eigen::Vector3f center_;

			Eigen::Vector3f min_;
			Eigen::Vector3f max_;

			pcl::PointIndices points_indices_;

			bool vertical_;
			float person_confidence_;

		public:

			typedef pcl::PointCloud<PointT> PointCloud;
			typedef boost::shared_ptr<PointCloud> PointCloudPtr;
			typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

			PersonCluster (
					const PointCloudPtr& input_cloud,
					const pcl::PointIndices& indices,
					const Eigen::VectorXf& ground_coeffs,
					float sqrt_ground_coeffs,
					bool head_centroid,
					bool vertical = false);

			virtual ~PersonCluster ();

			/**
			 * Returns the height of the cluster.
			 * @return the height of the cluster.
			 */
			float
			getHeight ();

			/**
			 * Update the height of the cluster.
			 * @param ground_coeffs the coefficients of the ground plane.
			 * @return the height of the cluster.
			 */
			float
			updateHeight (const Eigen::VectorXf& ground_coeffs);

			/**
			 * Update the height of the cluster.
			 * @param ground_coeffs the coefficients of the ground plane.
			 * @param sqrt_ground_coeffs the norm of the vector (a, b, c) where a, b and c are the first
			 * three coefficients of the ground plane (ax + by + cz + d = 0).
			 * @return the height of the cluster.
			 */
			float
			updateHeight (const Eigen::VectorXf& ground_coeffs, float sqrt_ground_coeffs);

			/**
			 * Returns the distance of the cluster from the sensor.
			 * @return the distance of the cluster (its centroid) from the sensor without considering the
			 * y dimension.
			 */
			float
			getDistance ();

			/**
			 * Returns the angle formed by the cluster's centroid with respect to the sensor (in radians).
			 * @return the angle formed by the cluster's centroid with respect to the sensor (in radians).
			 */
			float
			getAngle ();

			/**
			 * Returns the minimum angle formed by the cluster with respect to the sensor (in radians).
			 * @return the minimum angle formed by the cluster with respect to the sensor (in radians).
			 */
			float
			getAngleMin ();

			/**
			 * Returns the maximum angle formed by the cluster with respect to the sensor (in radians).
			 * @return the maximum angle formed by the cluster with respect to the sensor (in radians).
			 */
			float
			getAngleMax ();

			/**
			 * Returns the indices of the point cloud points corresponding to the cluster.
			 * @return the indices of the point cloud points corresponding to the cluster.
			 */
			pcl::PointIndices&
			getIndices ();

			/**
			 * Returns the theoretical top point.
			 * @return the theoretical top point.
			 */
			Eigen::Vector3f&
			getTTop ();

			/**
			 * Returns the theoretical bottom point.
			 * @return the theoretical bottom point.
			 */
			Eigen::Vector3f&
			getTBottom ();

			/**
			 * Returns the theoretical centroid (at half height).
			 * @return the theoretical centroid (at half height).
			 */
			Eigen::Vector3f&
			getTCenter ();

			/**
			 * Returns the top point.
			 * @return the top point.
			 */
			Eigen::Vector3f&
			getTop ();

			/**
			 * Returns the bottom point.
			 * @return the bottom point.
			 */
			Eigen::Vector3f&
			getBottom ();

			/**
			 * Returns the centroid.
			 * @return the centroid.
			 */
			Eigen::Vector3f&
			getCenter ();	//TODO: change in getHeadCenter?

			//Eigen::Vector3f& getTMax();

			/**
			 * Returns the point formed by min x, min y and min z.
			 * @return the point formed by min x, min y and min z.
			 */
			Eigen::Vector3f&
			getMin ();

			/**
			 * Returns the point formed by max x, max y and max z.
			 * @return the point formed by max x, max y and max z.
			 */
			Eigen::Vector3f&
			getMax ();

			/**
			 * Returns the HOG confidence.
			 * @return the HOG confidence.
			 */
			float
			getPersonConfidence ();

			/**
			 * Returns the number of points of the cluster.
			 * @return the number of points of the cluster.
			 */
			int
			getNumberPoints ();

			/**
			 * Sets the cluster height.
			 * @set the cluster height.
			 */
			void
			setHeight (float height);

			/**
			 * Sets the HOG confidence.
			 * @set the HOG confidence.
			 */
			void
			setPersonConfidence (float confidence);

			/**
			 * Draws the theoretical 3D bounding box of the cluster in the PCL visualizer.
			 * @param viewer: PCL visualizer.
			 * @param person_number: progressive number representing the person.
			 */
			void
			drawTBoundingBox (pcl::visualization::PCLVisualizer& viewer, int person_number);

			/**
			 * For sorting purpose: sort by distance.
			 */
			friend bool operator< <>(const PersonCluster<PointT>& c1, const PersonCluster<PointT>& c2);

		protected:

			void init(
					const PointCloudPtr& input_cloud,
					const pcl::PointIndices& indices,
					const Eigen::VectorXf& ground_coeffs,
					float sqrt_ground_coeffs,
					bool head_centroid,
					bool vertical);

		};
	} /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/person_cluster.hpp>
#endif /* PCL_PEOPLE_PERSON_CLUSTER_H_ */
