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
 * person_classifier.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_H_
#define PCL_PEOPLE_PERSON_CLASSIFIER_H_

#include <pcl/people/person_cluster.h>

namespace pcl
{
	namespace people
	{
		class PersonClassifier
		{
		protected:

			int window_height_;					///Height of the image patch to classify
			int window_width_;					///Width of the image patch to classify
			float SVM_offset_;					///SVM offset
			std::vector<float> SVM_weights_;	///SVM weights vector

		public:

			/**
			 * Default constructor
			 */
			PersonClassifier ();

			/**
			 * Default destructor
			 */
			virtual ~PersonClassifier ();

			/**
			 * Set trained SVM for person confidence estimation
			 */
			void
			setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset);

			/**
			 * Get trained SVM for person confidence estimation
			 */
			void
			getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset);

			/**
			 * Resize an image represented by a pointcloud RGB
			 */
			void
			resize (pcl::PointCloud<pcl::RGB>::Ptr& input_image, pcl::PointCloud<pcl::RGB>::Ptr& output_image,
							int window_width_, int window_height_);

			/**
			 * Copies an image and makes a black border around it, where the source image is not present.
			 */
			void
			copyMakeBorder (pcl::PointCloud<pcl::RGB>::Ptr& input_image, pcl::PointCloud<pcl::RGB>::Ptr& output_image,
					int xmin, int ymin, int width, int height);

			/**
			 * Classify the given portion of image
			 * @param height the height of the object to classify, in pixel
			 * @param xc the x-coordinate of the center of the object to classify, in pixel
			 * @param yc the y-coordinate of the center of the object to classify, in pixel
			 * @param image the whole image from which to extract the object to classify
			 * @return the result given by the SVM used
			 */
			double
			evaluate (float height, float xc, float yc, pcl::PointCloud<pcl::RGB>::Ptr& image);

			/**
			 * Compute person confidence for a given PersonCluster
			 */
			double
			evaluate (pcl::PointCloud<pcl::RGB>::Ptr& image, Eigen::Vector3f& bottom, Eigen::Vector3f& top, Eigen::Vector3f& centroid,
				 Eigen::Matrix3f intrinsics_matrix, bool vertical);
		};
	} /* namespace people */
} /* namespace pcl */
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_H_ */
