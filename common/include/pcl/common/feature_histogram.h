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

#ifndef PCL_FEATURE_HISTOGRAM_H_
#define PCL_FEATURE_HISTOGRAM_H_

#include <vector>

#include <pcl/pcl_macros.h>

namespace pcl
{ 
  /** \brief Type for histograms for computing mean and variance of some floats.
    *
    * \author Timur Ibadov (ibadov.timur@gmail.com)
    * \ingroup common
    */
  class PCL_EXPORTS FeatureHistogram
  {
    public:
      /** \brief Public constructor.
        * \param[in] number_of_bins number of bins in the histogram.
        * \param[in] min lower threshold.
        * \param[in] max upper threshold.
        */
      FeatureHistogram (const size_t number_of_bins, const float min,
          const float max);

      /** \brief Public destructor. */
      virtual ~FeatureHistogram ();

      /** \brief Get the lower threshold.
        * \return lower threshold.
        */
      float
      getThresholdMin () const;

      /** \brief Get the upper threshold.
        * \return upper threshold.
        */
      float
      getThresholdMax () const;

      /** \brief Get the number of elements was added to the histogram.
        * \return number of elements in the histogram.
        */
      size_t
      getNumberOfElements () const;

      /** \brief Get number of bins in the histogram.
        * \return number of bins in the histogram.
        */
      size_t
      getNumberOfBins () const;

      /** \brief Increase a bin, that corresponds the value.
        * \param[in] value new value.
        */
      void
      addValue (float value);

      /** \brief Get value, corresponds to the greatest bin.
        * \return mean value of the greatest bin.
        */
      float
      getMeanValue ();

      /** \brief Get variance of the value.
        * \return variance of the greatest bin.
        */
      float
      getVariance (float mean);

    protected:
      /** \brief Vector, that contain the histogram. */
      std::vector <unsigned> histogram_;

      /** \brief Min threshold. */
      float threshold_min_;
      /** \brief Max threshold. */
      float threshold_max_;
      /** \brief "Width" of a bin. */
      float step_;

      /** \brief Number of values was added to the histogram. */
      size_t number_of_elements_;

      /** \brief Number of bins. */
      size_t number_of_bins_;
  };
}
#endif // PCL_FEATURE_HISTOGRAM_H_