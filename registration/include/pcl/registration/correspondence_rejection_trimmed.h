/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/registration/correspondence_rejection.h>

namespace pcl {
namespace registration {
/** \brief CorrespondenceRejectorTrimmed implements a correspondence
 * rejection for ICP-like registration algorithms that uses only the best
 * 'k' correspondences where 'k' is some estimate of the overlap between
 * the two point clouds being registered.
 *
 * Reference:
 * 'The Trimmed Iterative Closest Point Algorithm' by
 * D. Chetverikov, D. Svirko, D. Stepanov, and Pavel Krsek.
 * In Proceedings of the 16th International Conference on Pattern
 * Recognition (ICPR 2002).
 *
 * \author Dirk Holz
 * \ingroup registration
 */
class PCL_EXPORTS CorrespondenceRejectorTrimmed : public CorrespondenceRejector {
  using CorrespondenceRejector::getClassName;
  using CorrespondenceRejector::input_correspondences_;
  using CorrespondenceRejector::rejection_name_;

public:
  using Ptr = shared_ptr<CorrespondenceRejectorTrimmed>;
  using ConstPtr = shared_ptr<const CorrespondenceRejectorTrimmed>;

  /** \brief Empty constructor. */
  CorrespondenceRejectorTrimmed() : overlap_ratio_(0.5f), nr_min_correspondences_(0)
  {
    rejection_name_ = "CorrespondenceRejectorTrimmed";
  }

  /** \brief Destructor. */
  ~CorrespondenceRejectorTrimmed() {}

  /** \brief Set the expected ratio of overlap between point clouds (in
   * terms of correspondences).
   * \param[in] ratio ratio of overlap between 0 (no overlap, no
   * correspondences) and 1 (full overlap, all correspondences)
   */
  virtual inline void
  setOverlapRatio(float ratio)
  {
    overlap_ratio_ = std::min(1.0f, std::max(0.0f, ratio));
  };

  /** \brief Get the maximum distance used for thresholding in correspondence rejection.
   */
  inline float
  getOverlapRatio() const
  {
    return overlap_ratio_;
  };

  /** \brief Set a minimum number of correspondences. If the specified overlap ratio
   * causes to have less correspondences,  \a CorrespondenceRejectorTrimmed will try to
   * return at least \a nr_min_correspondences_ correspondences (or all correspondences
   * in case \a nr_min_correspondences_ is less than the number of given
   * correspondences). \param[in] min_correspondences the minimum number of
   * correspondences
   */
  inline void
  setMinCorrespondences(unsigned int min_correspondences)
  {
    nr_min_correspondences_ = min_correspondences;
  };

  /** \brief Get the minimum number of correspondences. */
  inline unsigned int
  getMinCorrespondences() const
  {
    return nr_min_correspondences_;
  };

  /** \brief Get a list of valid correspondences after rejection from the original set
   * of correspondences. \param[in] original_correspondences the set of initial
   * correspondences given \param[out] remaining_correspondences the resultant filtered
   * set of remaining correspondences
   */
  void
  getRemainingCorrespondences(const pcl::Correspondences& original_correspondences,
                              pcl::Correspondences& remaining_correspondences) override;

protected:
  /** \brief Apply the rejection algorithm.
   * \param[out] correspondences the set of resultant correspondences.
   */
  inline void
  applyRejection(pcl::Correspondences& correspondences) override
  {
    getRemainingCorrespondences(*input_correspondences_, correspondences);
  }

  /** Overlap Ratio in [0..1] */
  float overlap_ratio_;

  /** Minimum number of correspondences. */
  unsigned int nr_min_correspondences_;
};

} // namespace registration
} // namespace pcl
