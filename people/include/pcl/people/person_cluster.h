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
 * person_cluster.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#pragma once

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

      /** \brief Minimum x coordinate of the cluster points. */
      float min_x_;
      /** \brief Minimum y coordinate of the cluster points. */
      float min_y_;
      /** \brief Minimum z coordinate of the cluster points. */
      float min_z_;

      /** \brief Maximum x coordinate of the cluster points. */
      float max_x_;
      /** \brief Maximum y coordinate of the cluster points. */
      float max_y_;
      /** \brief Maximum z coordinate of the cluster points. */
      float max_z_;

      /** \brief Sum of x coordinates of the cluster points. */
      float sum_x_;
      /** \brief Sum of y coordinates of the cluster points. */
      float sum_y_;
      /** \brief Sum of z coordinates of the cluster points. */
      float sum_z_;

      /** \brief Number of cluster points. */
      int n_;

      /** \brief x coordinate of the cluster centroid. */
      float c_x_;
      /** \brief y coordinate of the cluster centroid. */
      float c_y_;
      /** \brief z coordinate of the cluster centroid. */
      float c_z_;

      /** \brief Cluster height from the ground plane. */
      float height_;

      /** \brief Cluster distance from the sensor. */
      float distance_;
      /** \brief Cluster centroid horizontal angle with respect to z axis. */
      float angle_;

      /** \brief Maximum angle of the cluster points. */
      float angle_max_;
      /** \brief Minimum angle of the cluster points. */
      float angle_min_;
      
      /** \brief Cluster top point. */
      Eigen::Vector3f top_;
      /** \brief Cluster bottom point. */
      Eigen::Vector3f bottom_;
      /** \brief Cluster centroid. */
      Eigen::Vector3f center_;
      
      /** \brief Theoretical cluster top. */
      Eigen::Vector3f ttop_;
      /** \brief Theoretical cluster bottom (lying on the ground plane). */
      Eigen::Vector3f tbottom_;
      /** \brief Theoretical cluster center (between ttop_ and tbottom_). */
      Eigen::Vector3f tcenter_;

      /** \brief Vector containing the minimum coordinates of the cluster. */
      Eigen::Vector3f min_;
      /** \brief Vector containing the maximum coordinates of the cluster. */
      Eigen::Vector3f max_;

      /** \brief Point cloud indices of the cluster points. */
      pcl::PointIndices points_indices_;

      /** \brief If true, the sensor is considered to be vertically placed (portrait mode). */
      bool vertical_;
      /** \brief PersonCluster HOG confidence. */
      float person_confidence_;

    public:

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Constructor. */
      PersonCluster (
          const PointCloudPtr& input_cloud,
          const pcl::PointIndices& indices,
          const Eigen::VectorXf& ground_coeffs,
          float sqrt_ground_coeffs,
          bool head_centroid,
          bool vertical = false);

      /** \brief Destructor. */
      virtual ~PersonCluster ();

      /**
       * \brief Returns the height of the cluster.
       * \return the height of the cluster.
       */
      float
      getHeight () const;

      /**
       * \brief Update the height of the cluster.
       * \param[in] ground_coeffs The coefficients of the ground plane.
       * \return the height of the cluster.
       */
      float
      updateHeight (const Eigen::VectorXf& ground_coeffs);

      /**
       * \brief Update the height of the cluster.
       * \param[in] ground_coeffs The coefficients of the ground plane.
       * \param[in] sqrt_ground_coeffs The norm of the vector [a, b, c] where a, b and c are the first
       * three coefficients of the ground plane (ax + by + cz + d = 0).
       * \return the height of the cluster.
       */
      float
      updateHeight (const Eigen::VectorXf& ground_coeffs, float sqrt_ground_coeffs);

      /**
       * \brief Returns the distance of the cluster from the sensor.
       * \return the distance of the cluster (its centroid) from the sensor without considering the
       * y dimension.
       */
      float
      getDistance () const;

      /**
       * \brief Returns the angle formed by the cluster's centroid with respect to the sensor (in radians).
       * \return the angle formed by the cluster's centroid with respect to the sensor (in radians).
       */
      float
      getAngle () const;

      /**
       * \brief Returns the minimum angle formed by the cluster with respect to the sensor (in radians).
       * \return the minimum angle formed by the cluster with respect to the sensor (in radians).
       */
      float
      getAngleMin () const;

      /**
       * \brief Returns the maximum angle formed by the cluster with respect to the sensor (in radians).
       * \return the maximum angle formed by the cluster with respect to the sensor (in radians).
       */
      float
      getAngleMax () const;

      /**
       * \brief Returns the indices of the point cloud points corresponding to the cluster.
       * \return the indices of the point cloud points corresponding to the cluster.
       */
      pcl::PointIndices&
      getIndices ();

      /**
       * \brief Returns the theoretical top point.
       * \return the theoretical top point.
       */
      Eigen::Vector3f&
      getTTop ();

      /**
       * \brief Returns the theoretical bottom point.
       * \return the theoretical bottom point.
       */
      Eigen::Vector3f&
      getTBottom ();

      /**
       * \brief Returns the theoretical centroid (at half height).
       * \return the theoretical centroid (at half height).
       */
      Eigen::Vector3f&
      getTCenter ();

      /**
       * \brief Returns the top point.
       * \return the top point.
       */
      Eigen::Vector3f&
      getTop ();

      /**
       * \brief Returns the bottom point.
       * \return the bottom point.
       */
      Eigen::Vector3f&
      getBottom ();

      /**
       * \brief Returns the centroid.
       * \return the centroid.
       */
      Eigen::Vector3f&
      getCenter ();  

      //Eigen::Vector3f& getTMax();

      /**
       * \brief Returns the point formed by min x, min y and min z.
       * \return the point formed by min x, min y and min z.
       */
      Eigen::Vector3f&
      getMin ();

      /**
       * \brief Returns the point formed by max x, max y and max z.
       * \return the point formed by max x, max y and max z.
       */
      Eigen::Vector3f&
      getMax ();

      /**
       * \brief Returns the HOG confidence.
       * \return the HOG confidence.
       */
      float
      getPersonConfidence () const;

      /**
       * \brief Returns the number of points of the cluster.
       * \return the number of points of the cluster.
       */
      int
      getNumberPoints () const;

      /**
       * \brief Sets the cluster height.
       * \param[in] height
       */
      void
      setHeight (float height);

      /**
       * \brief Sets the HOG confidence.
       * \param[in] confidence
       */
      void
      setPersonConfidence (float confidence);

      /**
       * \brief Draws the theoretical 3D bounding box of the cluster in the PCL visualizer.
       * \param[in] viewer PCL visualizer.
       * \param[in] person_number progressive number representing the person.
       */
      void
      drawTBoundingBox (pcl::visualization::PCLVisualizer& viewer, int person_number);

      /**
       * \brief For sorting purpose: sort by distance.
       */
      friend bool operator< <>(const PersonCluster<PointT>& c1, const PersonCluster<PointT>& c2);

    protected:

      /**
       * \brief PersonCluster initialization.
       */
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
