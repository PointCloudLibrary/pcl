/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pcl/common/feature_histogram.h>

#include <pcl/console/print.h>

pcl::FeatureHistogram::FeatureHistogram (std::size_t const number_of_bins,
    const float min, const float max) : 
    histogram_ (number_of_bins, 0)
{
  // Initialize thresholds.
  if (min < max)
  {
    threshold_min_ = min;
    threshold_max_ = max;
    step_ = (max - min) / static_cast<float> (number_of_bins);
  }
  else
  {
    threshold_min_ = 0.0f;
    threshold_max_ = static_cast<float> (number_of_bins);
    step_ = 1.0f;
    PCL_WARN ("[FeatureHistogram::setThresholds] Variable \"max\" must be greater then \"min\".\n");
  }

  // Initialize sum.
  number_of_elements_ = 0;

  // Initialize size;
  number_of_bins_ = number_of_bins;
}

pcl::FeatureHistogram::~FeatureHistogram () = default;

float
pcl::FeatureHistogram::getThresholdMin () const
{
  return (threshold_min_);
}

float
pcl::FeatureHistogram::getThresholdMax () const
{
  return (threshold_max_);
}

std::size_t
pcl::FeatureHistogram::getNumberOfElements () const
{
  return (number_of_elements_);
}

std::size_t
pcl::FeatureHistogram::getNumberOfBins () const
{
  return (number_of_bins_);
}

void
pcl::FeatureHistogram::addValue (float value)
{
  // Check, if value in the allowed range.
  if (threshold_min_ < value && value < threshold_max_)
  {
    // Increase the sum.
    ++number_of_elements_;

    // Increase the bin.
    auto bin_number = static_cast<std::size_t> ((value - threshold_min_) / step_);
    ++histogram_[bin_number];
  }
}

float
pcl::FeatureHistogram::getMeanValue ()
{
    // Check, if the histogram is empty.
  if (number_of_elements_ == 0)
  {
    return (0.0f);
  }
  // Smoothe the histogram and find a bin with a max smoothed value.
  std::size_t max_idx = 0;
  float max = 0.50f * histogram_[0] + 
              0.25f * histogram_[1] * 2.0f;
  for (std::size_t bin = 1; bin < histogram_.size () - 1; ++bin)
  {
    float smothed_value = 0.25f * histogram_[bin - 1] + 
                          0.50f * histogram_[bin] + 
                          0.25f * histogram_[bin + 1];
    if (smothed_value > max)
    {
      max = smothed_value;
      max_idx = bin;
    }
  }
  // Check last bin.
  float last_value = 0.50f * histogram_[histogram_.size () - 1] + 
                     0.25f * histogram_[histogram_.size () - 2] * 2.0f;
  if (last_value > max)
  {
    max_idx = histogram_.size () - 1;
  }

  // Compute mean value.
  float mean = step_ * (static_cast<float> (max_idx) + 0.5f) + threshold_min_;

  return (mean);
}

float
pcl::FeatureHistogram::getVariance (float mean)
{
  // Check, if the histogram is empty.
  if (number_of_elements_ == 0)
  {
    return (0.0f);
  }
  // The histogram is not empty.
  // Variable to accumulate the terms of variance.
  float variances_sum = 0;

  for (std::size_t bin = 0; bin < number_of_bins_; ++bin)
  {
    if (histogram_[bin] > 0)
    {
      // Value corresponding to the bin.
      float value = step_ * (static_cast<float> (bin) + 0.5f) + threshold_min_;
      float dif = value - mean;
      variances_sum += static_cast<float> (histogram_[bin]) * dif * dif;
    }
  }

  // Compute variance and return it.
  return (variances_sum / static_cast<float> (number_of_elements_));
}
