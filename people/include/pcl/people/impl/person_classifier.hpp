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
 * person_classifier.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#include <pcl/people/person_classifier.h>

#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_HPP_
#define PCL_PEOPLE_PERSON_CLASSIFIER_HPP_

template <typename PointT>
pcl::people::PersonClassifier<PointT>::PersonClassifier () {}

template <typename PointT>
pcl::people::PersonClassifier<PointT>::~PersonClassifier () {}

template <typename PointT> bool
pcl::people::PersonClassifier<PointT>::loadSVMFromFile (std::string svm_filename)
{
  std::string line;
  std::ifstream SVM_file;
  SVM_file.open(svm_filename.c_str());

  getline (SVM_file,line);      // read window_height line
  std::size_t tok_pos = line.find_first_of(':', 0);  // search for token ":"
  window_height_ = std::atoi(line.substr(tok_pos+1, std::string::npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read window_width line
  tok_pos = line.find_first_of(':', 0);  // search for token ":"
  window_width_ = std::atoi(line.substr(tok_pos+1, std::string::npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_offset line
  tok_pos = line.find_first_of(':', 0);  // search for token ":"
  SVM_offset_ = std::atof(line.substr(tok_pos+1, std::string::npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_weights line
  tok_pos = line.find_first_of('[', 0);  // search for token "["
  std::size_t tok_end_pos = line.find_first_of(']', 0);  // search for token "]" , end of SVM weights
  while (tok_pos < tok_end_pos) // while end of SVM_weights is not reached
  {
    std::size_t prev_tok_pos = tok_pos;
    tok_pos = line.find_first_of(',', prev_tok_pos+1);  // search for token ","
    SVM_weights_.push_back(std::atof(line.substr(prev_tok_pos+1, tok_pos-prev_tok_pos-1).c_str()));
  }
  SVM_file.close();
  
  if (SVM_weights_.empty ())
  {
    PCL_ERROR ("[pcl::people::PersonClassifier::loadSVMFromFile] Invalid SVM file!\n");
    return (false);
  }
  return (true);
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset)
{
  window_height_ = window_height;
  window_width_ = window_width;
  SVM_weights_ = SVM_weights;
  SVM_offset_ = SVM_offset;
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset)
{
  window_height = window_height_;
  window_width = window_width_;
  SVM_weights = SVM_weights_;
  SVM_offset = SVM_offset_;
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::resize (PointCloudPtr& input_image,
              PointCloudPtr& output_image,
              int width,
              int height)
{
  PointT new_point;
  new_point.r = 0;
  new_point.g = 0;
  new_point.b = 0;

  // Allocate the vector of points:
  output_image->points.resize(width*height, new_point);
  output_image->height = height;
  output_image->width = width;

  // Compute scale factor:
  float scale1 = float(height) / float(input_image->height);
  float scale2 = float(width) / float(input_image->width);

  Eigen::Matrix3f T_inv;
  T_inv << 1/scale1, 0, 0,
       0, 1/scale2, 0,
       0,   0,   1;

  Eigen::Vector3f A;
  int c1, c2, f1, f2;
  PointT g1, g2, g3, g4;
  float w1, w2;
  for (int i = 0; i < height; i++)    // for every row
  {
  for (int j = 0; j < width; j++)  // for every column
  {
    A = T_inv * Eigen::Vector3f(i, j, 1);
    c1 = std::ceil(A(0));
    f1 = std::floor(A(0));
    c2 = std::ceil(A(1));
    f2 = std::floor(A(1));

    if ( (f1 < 0) ||
       (c1 < 0) ||
       (f1 >= static_cast<int> (input_image->height)) ||
       (c1 >= static_cast<int> (input_image->height)) ||
       (f2 < 0) ||
       (c2 < 0) ||
       (f2 >= static_cast<int> (input_image->width)) ||
       (c2 >= static_cast<int> (input_image->width)))
    { // if out of range, continue
    continue;
    }

    g1 = (*input_image)(f2, c1);
    g3 = (*input_image)(f2, f1);
    g4 = (*input_image)(c2, f1);
    g2 = (*input_image)(c2, c1);

    w1 = (A(0) - f1);
    w2 = (A(1) - f2);
    new_point.r = int((1 - w1) * ((1 - w2) * g1.r + w2 * g4.r) + w1 * ((1 - w2) * g3.r + w2 * g4.r));
    new_point.g = int((1 - w1) * ((1 - w2) * g1.g + w2 * g4.g) + w1 * ((1 - w2) * g3.g + w2 * g4.g));
    new_point.b = int((1 - w1) * ((1 - w2) * g1.b + w2 * g4.b) + w1 * ((1 - w2) * g3.b + w2 * g4.b));

    // Insert the point in the output image:
    (*output_image)(j,i) = new_point;
  }
  }
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::copyMakeBorder (PointCloudPtr& input_image,
                  PointCloudPtr& output_image,
                  int xmin,
                  int ymin,
                  int width,
                  int height)
{
  PointT black_point;
  black_point.r = 0;
  black_point.g = 0;
  black_point.b = 0;
  output_image->points.resize(height*width, black_point);
  output_image->width = width;
  output_image->height = height;

  int x_start_in = std::max(0, xmin);
  int x_end_in = std::min(int(input_image->width-1), xmin+width-1);
  int y_start_in = std::max(0, ymin);
  int y_end_in = std::min(int(input_image->height-1), ymin+height-1);

  int x_start_out = std::max(0, -xmin);
  //int x_end_out = x_start_out + (x_end_in - x_start_in);
  int y_start_out = std::max(0, -ymin);
  //int y_end_out = y_start_out + (y_end_in - y_start_in);

  for (int i = 0; i < (y_end_in - y_start_in + 1); i++)
  {
    for (int j = 0; j < (x_end_in - x_start_in + 1); j++)
    {
      (*output_image)(x_start_out + j, y_start_out + i) = (*input_image)(x_start_in + j, y_start_in + i);
    }
  }
}

template <typename PointT> double
pcl::people::PersonClassifier<PointT>::evaluate (float height_person,
              float xc,
              float yc,
              PointCloudPtr& image)
{
  if (SVM_weights_.empty ())
  {
  PCL_ERROR ("[pcl::people::PersonClassifier::evaluate] SVM has not been set!\n");
  return (-1000);
  }

  int height = std::floor((height_person * window_height_) / (0.75 * window_height_) + 0.5);  // std::floor(i+0.5) = round(i)
  int width = std::floor((height_person * window_width_) / (0.75 * window_height_) + 0.5);
  int xmin = std::floor(xc - width / 2 + 0.5);
  int ymin = std::floor(yc - height / 2 + 0.5);
  double confidence;

  if (height > 0)
  {
    // If near the border, fill with black:
    PointCloudPtr box(new PointCloud);
    copyMakeBorder(image, box, xmin, ymin, width, height);

    // Make the image match the correct size (used in the training stage):
    PointCloudPtr sample(new PointCloud);
    resize(box, sample, window_width_, window_height_);

    // Convert the image to array of float:
    float* sample_float = new float[sample->width * sample->height * 3]; 
    int delta = sample->height * sample->width;
    for (std::uint32_t row = 0; row < sample->height; row++)
    {
      for (std::uint32_t col = 0; col < sample->width; col++)
      {
        sample_float[row + sample->height * col] = ((float) ((*sample)(col, row).r))/255; //ptr[col * 3 + 2];
        sample_float[row + sample->height * col + delta] = ((float) ((*sample)(col, row).g))/255; //ptr[col * 3 + 1];
        sample_float[row + sample->height * col + delta * 2] = (float) (((*sample)(col, row).b))/255; //ptr[col * 3];
      }
    }

    // Calculate HOG descriptor:
    pcl::people::HOG hog;
    float *descriptor = (float*) calloc(SVM_weights_.size(), sizeof(float));
    hog.compute(sample_float, descriptor);
 
    // Calculate confidence value by dot product:
    confidence = 0.0;
    for(std::size_t i = 0; i < SVM_weights_.size(); i++)
    { 
      confidence += SVM_weights_[i] * descriptor[i];
    }
    // Confidence correction:
    confidence -= SVM_offset_;  

    delete[] descriptor;
    delete[] sample_float;
  }
  else
  {
    confidence = std::numeric_limits<double>::quiet_NaN();
  } 

  return confidence;
}

template <typename PointT> double
pcl::people::PersonClassifier<PointT>::evaluate (PointCloudPtr& image,
              Eigen::Vector3f& bottom,
              Eigen::Vector3f& top,
              Eigen::Vector3f& centroid,
              bool vertical)
{
  float pixel_height;
  float pixel_width;

  if (!vertical)
  {
    pixel_height = bottom(1) - top(1);
    pixel_width = pixel_height / 2.0f;
  }
  else
  {
    pixel_width = top(0) - bottom(0);
    pixel_height = pixel_width / 2.0f;
  }
  float pixel_xc = centroid(0);
  float pixel_yc = centroid(1);

  if (!vertical)
  {
    return (evaluate(pixel_height, pixel_xc, pixel_yc, image));
  }
  return (evaluate(pixel_width, pixel_yc, image->height-pixel_xc+1, image));
}
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_HPP_ */
