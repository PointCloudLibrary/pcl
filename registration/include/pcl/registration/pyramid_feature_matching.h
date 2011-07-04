/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * $Id$
 */

#ifndef PCL_PYRAMID_FEATURE_MATCHING_H_
#define PCL_PYRAMID_FEATURE_MATCHING_H_

#include <pcl/pcl_base.h>

namespace pcl
{
  struct PyramidHistogram
  {
    typedef boost::shared_ptr<PyramidHistogram> Ptr;

    PyramidHistogram (std::vector<std::pair<float, float> > &a_dimension_range,
                      size_t &a_nr_levels)
      : dimension_range (a_dimension_range),
        nr_levels (a_nr_levels)
    {
      initializeHistogram ();
    }

    void
    initializeHistogram ();

    void
    addFeature (std::vector<float> &feature);

    unsigned int&
    at (std::vector<size_t> &access,
        size_t &level);

    unsigned int&
    at (std::vector<float> &feature,
        size_t &level);

    size_t dimensions, nr_levels;
    std::vector<std::pair<float, float> > dimension_range;

    struct PyramidHistogramLevel
    {
      PyramidHistogramLevel (std::vector<size_t> &a_bins_per_dimension,
                             std::vector<float> &a_bin_step)
        : bins_per_dimension (a_bins_per_dimension),
          bin_step (a_bin_step)
      {
        initializeHistogramLevel ();
      }

      void
      initializeHistogramLevel ();

      std::vector<unsigned int> hist;
      std::vector<size_t> bins_per_dimension;
      std::vector<float> bin_step;
    };

    std::vector<PyramidHistogramLevel> hist_levels;
  };


  template <typename PointFeature>
  class PyramidFeatureMatching : public PCLBase<PointFeature>
  {
    public:
      typedef pcl::PointCloud<PointFeature> FeatureCloud;
      typedef typename pcl::PointCloud<PointFeature>::ConstPtr FeatureCloudConstPtr;

      using PCLBase<PointFeature>::input_;

      PyramidFeatureMatching (size_t a_dimensions,
                              std::vector<std::pair<float, float> > a_dimension_range)
        : dimensions (a_dimensions),
          dimension_range (a_dimension_range)
      {
        initialize ();
      };

      void
      initialize ();

      void
      computePyramidHistogram (const FeatureCloudConstPtr &feature_cloud,
                               PyramidHistogram::Ptr &pyramid);

      virtual void
      convertFeatureToVector (const PointFeature& feature,
                              std::vector<float>& feature_vector) = 0;

      float
      comparePyramidHistograms (const pcl::PyramidHistogram::Ptr &pyramid_a,
                                const pcl::PyramidHistogram::Ptr &pyramid_b);

    protected:
      size_t dimensions, levels;
      std::vector<std::pair<float, float> > dimension_range;
  };
}

#endif /* PCL_PYRAMID_FEATURE_MATCHING_H_ */
