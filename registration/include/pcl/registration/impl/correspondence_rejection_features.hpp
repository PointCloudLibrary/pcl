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

#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_

#include <boost/pointer_cast.hpp> // for static_pointer_cast

namespace pcl {

namespace registration {

template <typename FeatureT>
inline void
CorrespondenceRejectorFeatures::setSourceFeature(
    const typename pcl::PointCloud<FeatureT>::ConstPtr& source_feature,
    const std::string& key)
{
  if (features_map_.count(key) == 0)
    features_map_[key].reset(new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
      ->setSourceFeature(source_feature);
}

template <typename FeatureT>
inline typename pcl::PointCloud<FeatureT>::ConstPtr
CorrespondenceRejectorFeatures::getSourceFeature(const std::string& key)
{
  if (features_map_.count(key) == 0)
    return (nullptr);
  return (boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
              ->getSourceFeature());
}

template <typename FeatureT>
inline void
CorrespondenceRejectorFeatures::setTargetFeature(
    const typename pcl::PointCloud<FeatureT>::ConstPtr& target_feature,
    const std::string& key)
{
  if (features_map_.count(key) == 0)
    features_map_[key].reset(new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
      ->setTargetFeature(target_feature);
}

template <typename FeatureT>
inline typename pcl::PointCloud<FeatureT>::ConstPtr
CorrespondenceRejectorFeatures::getTargetFeature(const std::string& key)
{
  if (features_map_.count(key) == 0)
    return (nullptr);
  return (boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
              ->getTargetFeature());
}

template <typename FeatureT>
inline void
CorrespondenceRejectorFeatures::setDistanceThreshold(double thresh,
                                                     const std::string& key)
{
  if (features_map_.count(key) == 0)
    features_map_[key].reset(new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
      ->setDistanceThreshold(thresh);
}

template <typename FeatureT>
inline void
CorrespondenceRejectorFeatures::setFeatureRepresentation(
    const typename pcl::PointRepresentation<FeatureT>::ConstPtr& fr,
    const std::string& key)
{
  if (features_map_.count(key) == 0)
    features_map_[key].reset(new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT>>(features_map_[key])
      ->setFeatureRepresentation(fr);
}

} // namespace registration
} // namespace pcl

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_ */
