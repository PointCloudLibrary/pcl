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
 * ground_based_people_detection_app.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_H_
#define PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_H_

#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/people/person_cluster.h>
#include <pcl/people/head_based_subcluster.h>
#include <pcl/people/person_classifier.h>
#include <pcl/common/transforms.h>

namespace pcl
{
  namespace people
  {
    /** \brief GroundBasedPeopleDetectionApp performs people detection on RGB-D data having as input the ground plane coefficients.
     * It implements the people detection algorithm described here:
     * M. Munaro, F. Basso and E. Menegatti,
     * Tracking people within groups with RGB-D data,
     * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
     *
     * \author Matteo Munaro
     * \ingroup people
     */
    template <typename PointT> class GroundBasedPeopleDetectionApp;

    template <typename PointT>
    class GroundBasedPeopleDetectionApp
    {
    public:

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      /** \brief Constructor. */
      GroundBasedPeopleDetectionApp ();

      /** \brief Destructor. */
      virtual ~GroundBasedPeopleDetectionApp ();

      /**
       * \brief Set the pointer to the input cloud.
       *
       * \param[in] cloud A pointer to the input cloud.
       */
      void
      setInputCloud (PointCloudPtr& cloud);

      /**
       * \brief Set the ground coefficients.
       *
       * \param[in] ground_coeffs Vector containing the four plane coefficients.
       */
      void
      setGround (Eigen::VectorXf& ground_coeffs);

      /**
       * \brief Set the transformation matrix, which is used in order to transform the given point cloud, the ground plane and the intrinsics matrix to the internal coordinate frame.
       * \param[in] transformation
       */
      void
      setTransformation (const Eigen::Matrix3f& transformation);

      /**
       * \brief Set sampling factor. 
       *
       * \param[in] sampling_factor Value of the downsampling factor (in each dimension) which is applied to the raw point cloud (default = 1.).
       */
      void
      setSamplingFactor (int sampling_factor);
      
      /**
       * \brief Set voxel size. 
       *
       * \param[in] voxel_size Value of the voxel dimension (default = 0.06m.).
       */
      void
      setVoxelSize (float voxel_size);

      /**
       * \brief Set intrinsic parameters of the RGB camera.
       *
       * \param[in] intrinsics_matrix RGB camera intrinsic parameters matrix.
       */
      void
      setIntrinsics (Eigen::Matrix3f intrinsics_matrix);

      /**
       * \brief Set SVM-based person classifier.
       *
       * \param[in] person_classifier Needed for people detection on RGB data.
       */
      void
      setClassifier (pcl::people::PersonClassifier<pcl::RGB> person_classifier);

      /**
       * \brief Set the field of view of the point cloud in z direction.
       *
       * \param[in] min The beginning of the field of view in z-direction, should be usually set to zero.
       * \param[in] max The end of the field of view in z-direction.
       */
      void
      setFOV (float min, float max);

      /**
       * \brief Set sensor orientation (vertical = true means portrait mode, vertical = false means landscape mode).
       *
       * \param[in] vertical Set landscape/portait camera orientation (default = false).
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
       * \brief Set minimum and maximum allowed height and width for a person cluster.
       *
       * \param[in] min_height Minimum allowed height for a person cluster (default = 1.3).
       * \param[in] max_height Maximum allowed height for a person cluster (default = 2.3).
       * \param[in] min_width Minimum width for a person cluster (default = 0.1).
       * \param[in] max_width Maximum width for a person cluster (default = 8.0).
       */
      void
      setPersonClusterLimits (float min_height, float max_height, float min_width, float max_width);

      /**
       * \brief Set minimum distance between persons' heads.
       *
       * \param[in] heads_minimum_distance Minimum allowed distance between persons' heads (default = 0.3).
       */
      void
      setMinimumDistanceBetweenHeads (float heads_minimum_distance);

      /**
       * \brief Get the minimum and maximum allowed height and width for a person cluster.
       *
       * \param[out] min_height Minimum allowed height for a person cluster.
       * \param[out] max_height Maximum allowed height for a person cluster.
       * \param[out] min_width Minimum width for a person cluster.
       * \param[out] max_width Maximum width for a person cluster.
       */
      void
      getPersonClusterLimits (float& min_height, float& max_height, float& min_width, float& max_width);

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

      /**
       * \brief Get floor coefficients.
       */
      Eigen::VectorXf
      getGround ();

      /**
       * \brief Get the filtered point cloud.
       */
      PointCloudPtr
      getFilteredCloud ();

      /**
       * \brief Get pointcloud after voxel grid filtering and ground removal.
       */
      PointCloudPtr
      getNoGroundCloud ();

      /**
       * \brief Extract RGB information from a point cloud and output the corresponding RGB point cloud.
       *
       * \param[in] input_cloud A pointer to a point cloud containing also RGB information.
       * \param[out] output_cloud A pointer to a RGB point cloud.
       */
      void
      extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud);

      /**
       * \brief Swap rows/cols dimensions of a RGB point cloud (90 degrees counterclockwise rotation).
       *
       * \param[in,out] cloud A pointer to a RGB point cloud.
       */
      void
      swapDimensions (pcl::PointCloud<pcl::RGB>::Ptr& cloud);

     /**
       * \brief Estimates min_points_ and max_points_ based on the minimal and maximal cluster size and the voxel size.
       */
      void
      updateMinMaxPoints ();

      /**
       * \brief Applies the transformation to the input point cloud.
       */
      void
      applyTransformationPointCloud ();

      /**
       * \brief Applies the transformation to the ground plane.
       */
      void
      applyTransformationGround ();

      /**
       * \brief Applies the transformation to the intrinsics matrix.
       */
      void
      applyTransformationIntrinsics ();

      /**
       * \brief Reduces the input cloud to one point per voxel and limits the field of view.
       */
      void
      filter ();

      /**
       * \brief Perform people detection on the input data and return people clusters information.
       * 
       * \param[out] clusters Vector of PersonCluster.
       * 
       * \return true if the compute operation is successful, false otherwise.
       */
      bool
      compute (std::vector<pcl::people::PersonCluster<PointT> >& clusters);

    protected:
      /** \brief sampling factor used to downsample the point cloud */
      int sampling_factor_; 
      
      /** \brief voxel size */
      float voxel_size_;                  
      
      /** \brief ground plane coefficients */
      Eigen::VectorXf ground_coeffs_;

      /** \brief flag stating whether the ground coefficients have been set or not */
      bool ground_coeffs_set_;

      /** \brief the transformed ground coefficients */
      Eigen::VectorXf ground_coeffs_transformed_;

      /** \brief ground plane normalization factor */
      float sqrt_ground_coeffs_;

      /** \brief rotation matrix which transforms input point cloud to internal people tracker coordinate frame */
      Eigen::Matrix3f transformation_;

      /** \brief flag stating whether the transformation matrix has been set or not */
      bool transformation_set_;

      /** \brief pointer to the input cloud */
      PointCloudPtr cloud_;

      /** \brief pointer to the filtered cloud */
      PointCloudPtr cloud_filtered_;

      /** \brief pointer to the cloud after voxel grid filtering and ground removal */
      PointCloudPtr no_ground_cloud_;              
      
      /** \brief pointer to a RGB cloud corresponding to cloud_ */
      pcl::PointCloud<pcl::RGB>::Ptr rgb_image_;      
      
      /** \brief person clusters maximum height from the ground plane */
      float max_height_;                  
      
      /** \brief person clusters minimum height from the ground plane */
      float min_height_;

      /** \brief person clusters maximum width, used to estimate how many points maximally represent a person cluster */
      float max_width_;

      /** \brief person clusters minimum width, used to estimate how many points minimally represent a person cluster */
      float min_width_;

      /** \brief the beginning of the field of view in z-direction, should be usually set to zero */
      float min_fov_;

      /** \brief the end of the field of view in z-direction */
      float max_fov_;

      /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
      bool vertical_;                    
      
      /** \brief if true, the person centroid is computed as the centroid of the cluster points belonging to the head;  
       * if false, the person centroid is computed as the centroid of the whole cluster points (default = true) */
      bool head_centroid_;    // if true, the person centroid is computed as the centroid of the cluster points belonging to the head (default = true)
                              // if false, the person centroid is computed as the centroid of the whole cluster points 
      /** \brief maximum number of points for a person cluster */
      int max_points_;                  
      
      /** \brief minimum number of points for a person cluster */
      int min_points_;                  
      
      /** \brief minimum distance between persons' heads */
      float heads_minimum_distance_;            
      
      /** \brief intrinsic parameters matrix of the RGB camera */
      Eigen::Matrix3f intrinsics_matrix_;

      /** \brief flag stating whether the intrinsics matrix has been set or not */
      bool intrinsics_matrix_set_;

      /** \brief the transformed intrinsics matrix */
      Eigen::Matrix3f intrinsics_matrix_transformed_;

      /** \brief SVM-based person classifier */
      pcl::people::PersonClassifier<pcl::RGB> person_classifier_;  
      
      /** \brief flag stating if the classifier has been set or not */
      bool person_classifier_set_flag_;
    };
  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/ground_based_people_detection_app.hpp>
#endif /* PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_H_ */
