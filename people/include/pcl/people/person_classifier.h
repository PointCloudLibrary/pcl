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
 * person_classifier.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */
 
#pragma once

#include <pcl/people/person_cluster.h>
#include <pcl/people/hog.h>

namespace pcl
{
  namespace people
  {
    template <typename PointT> class PersonClassifier;

    template <typename PointT>
    class PersonClassifier
    {
    protected:

      /** \brief Height of the image patch to classify. */
      int window_height_;          
      
      /** \brief Width of the image patch to classify. */
      int window_width_;          
      
      /** \brief SVM offset. */
      float SVM_offset_;          
      
      /** \brief SVM weights vector. */
      std::vector<float> SVM_weights_;  

    public:

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;

      /** \brief Constructor. */
      PersonClassifier ();

      /** \brief Destructor. */
      virtual ~PersonClassifier ();

      /** \brief Load SVM parameters from a text file. 
       *
       * \param[in] svm_filename Filename containing SVM parameters.
       * 
       * \return true if SVM has been correctly set, false otherwise.
       */
      bool
      loadSVMFromFile (const std::string& svm_filename);

      /**
       * \brief Set trained SVM for person confidence estimation.
       * 
       * \param[in] window_height Detection window height.
       * \param[in] window_width Detection window width.
       * \param[in] SVM_weights SVM weights vector.
       * \param[in] SVM_offset SVM offset.
       */
      void
      setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset);

      /**
       * \brief Get trained SVM for person confidence estimation.
       * 
       * \param[out] window_height Detection window height.
       * \param[out] window_width Detection window width.
       * \param[out] SVM_weights SVM weights vector.
       * \param[out] SVM_offset SVM offset.
       */
      void
      getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset);

      /**
       * \brief Resize an image represented by a pointcloud containing RGB information.
       * 
       * \param[in] input_image A pointer to a pointcloud containing RGB information.
       * \param[out] output_image A pointer to the output pointcloud.
       * \param[in] width Output width.
       * \param[in] height Output height.
       */
      void
      resize (PointCloudPtr& input_image, PointCloudPtr& output_image,
              int width, int height);

      /**
       * \brief Copies an image and makes a black border around it, where the source image is not present.
       * 
       * \param[in] input_image A pointer to a pointcloud containing RGB information.
       * \param[out] output_image A pointer to the output pointcloud.
       * \param[in] xmin x coordinate of the top-left point of the bbox to copy from the input image.
       * \param[in] ymin y coordinate of the top-left point of the bbox to copy from the input image.
       * \param[in] width Output width.
       * \param[in] height Output height.
       */
      void
      copyMakeBorder (PointCloudPtr& input_image, PointCloudPtr& output_image,
          int xmin, int ymin, int width, int height);

      /**
       * \brief Classify the given portion of image.
       * 
       * \param[in] height The height of the image patch to classify, in pixels.
       * \param[in] xc The x-coordinate of the center of the image patch to classify, in pixels.
       * \param[in] yc The y-coordinate of the center of the image patch to classify, in pixels.
       * \param[in] image The whole image (pointer to a point cloud containing RGB information) containing the object to classify.
       * \return The classification score given by the SVM.
       */
      double
      evaluate (float height, float xc, float yc, PointCloudPtr& image);

      /**
       * \brief Compute person confidence for a given PersonCluster.
       * 
       * \param[in] image The input image (pointer to a point cloud containing RGB information).
       * \param[in] bottom Theoretical bottom point of the cluster projected to the image.
       * \param[in] top Theoretical top point of the cluster projected to the image.
       * \param[in] centroid Theoretical centroid point of the cluster projected to the image.
       * \param[in] vertical If true, the sensor is considered to be vertically placed (portrait mode).
       * \return The person confidence.
       */
      double
      evaluate (PointCloudPtr& image, Eigen::Vector3f& bottom, Eigen::Vector3f& top, Eigen::Vector3f& centroid,
         bool vertical);
    };
  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/person_classifier.hpp>
