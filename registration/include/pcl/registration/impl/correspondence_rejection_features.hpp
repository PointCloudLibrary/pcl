/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectorFeatures::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences, 
    pcl::Correspondences& remaining_correspondences)
{
  unsigned int number_valid_correspondences = 0;
  remaining_correspondences.resize (original_correspondences.size ());
  // For each set of features, go over each correspondence from input_correspondences_
  for (size_t i = 0; i < input_correspondences_->size (); ++i)
  {
    // Go over the map of features
    for (FeaturesMap::const_iterator it = features_map_.begin (); it != features_map_.end (); ++it)
    {
      // Check if the score in feature space is above the given threshold
      // (assume that the number of feature correspondenecs is the same as the number of point correspondences)
      if (!it->second->isCorrespondenceValid (static_cast<int> (i)))
        break;

      remaining_correspondences[number_valid_correspondences] = original_correspondences[i];
      ++number_valid_correspondences;
    }
  }
  remaining_correspondences.resize (number_valid_correspondences);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline void 
pcl::registration::CorrespondenceRejectorFeatures::setSourceFeature (
    const typename pcl::PointCloud<FeatureT>::ConstPtr &source_feature, const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setSourceFeature (source_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
pcl::registration::CorrespondenceRejectorFeatures::getSourceFeature (const std::string &key)
{
  if (features_map_.count (key) == 0)
    return (boost::shared_ptr<pcl::PointCloud<const FeatureT> > ());
  else
    return (boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->getSourceFeature ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline void 
pcl::registration::CorrespondenceRejectorFeatures::setTargetFeature (
    const typename pcl::PointCloud<FeatureT>::ConstPtr &target_feature, const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setTargetFeature (target_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
pcl::registration::CorrespondenceRejectorFeatures::getTargetFeature (const std::string &key)
{
  typedef pcl::PointCloud<FeatureT> FeatureCloud;
  typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

  if (features_map_.count (key) == 0)
    return (boost::shared_ptr<const pcl::PointCloud<FeatureT> > ());
  else
    return (boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->getTargetFeature ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline void 
pcl::registration::CorrespondenceRejectorFeatures::setDistanceThreshold (
    double thresh, const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setDistanceThreshold (thresh);
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline bool
pcl::registration::CorrespondenceRejectorFeatures::hasValidFeatures ()
{
  if (features_map_.empty ())
    return (false);
  FeaturesMap::const_iterator feature_itr;
  for (feature_itr = features_map_.begin (); feature_itr != features_map_.end (); ++feature_itr)
    if (!feature_itr->second->isValid ())
      return (false);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureT> inline void 
pcl::registration::CorrespondenceRejectorFeatures::setFeatureRepresentation (
  const typename pcl::PointRepresentation<FeatureT>::ConstPtr &fr,
  const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setFeatureRepresentation (fr);
}


#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_FEATURES_HPP_ */
