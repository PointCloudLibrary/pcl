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
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_

#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setInputTarget (
    const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }

  // Create a copy of the data so we can set the last element to 1
  target_.reset (new PointCloudTarget);
  *target_ = *cloud;

  // Set all the point.data[3] values to 1 to aid the rigid transformation
  for (size_t i = 0; i < target_->points.size (); ++i)
    target_->points[i].data[3] = 1.0;

  tree_->setInputCloud (target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setSourceFeature (
    const typename pcl::PointCloud<FeatureT>::ConstPtr &source_feature, 
    const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setSourceFeature (source_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::getSourceFeature (
    const std::string &key)
{
  if (features_map_.count (key) == 0)
    return (boost::shared_ptr<pcl::PointCloud<const FeatureT> > ());
  else
    return (boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->getSourceFeature ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setTargetFeature (
    const typename pcl::PointCloud<FeatureT>::ConstPtr &target_feature, 
    const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setTargetFeature (target_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::getTargetFeature (
    const std::string &key)
{
  typedef pcl::PointCloud<FeatureT> FeatureCloud;
  typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

  if (features_map_.count (key) == 0)
    return (boost::shared_ptr<const pcl::PointCloud<FeatureT> > ());
  else
    return (boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->getTargetFeature ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setRadiusSearch (
    const typename pcl::KdTree<FeatureT>::Ptr &tree,
    float r, 
    const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setRadiusSearch (tree, r);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setKSearch (
    const typename pcl::KdTree<FeatureT>::Ptr &tree,                                                       
    int k, 
    const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setKSearch (tree, k);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline bool
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::hasValidFeatures ()
{
  if (features_map_.empty ())
    return (false);
  typename FeaturesMap::const_iterator feature_itr;
  for (feature_itr = features_map_.begin (); feature_itr != features_map_.end (); ++feature_itr)
    if (!feature_itr->second->isValid ())
      return (false);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::determineFeatureCorrespondences (
    std::vector<pcl::Correspondence> &correspondence_indices)
{
  if (features_map_.empty ())
  {
    PCL_ERROR ("[pcl::%s::findFeatureCorrespondences] At least one feature must be set!\n",
               getClassName ().c_str ());
    return;
  }

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  // Find the correspondence indices for the first feature in the features map
/*  typename FeaturesMap::const_iterator feature_itr = features_map_.begin ();
  feature_itr->second->findFeatureCorrespondences ((*indices_)[0], correspondence_indices, nn_dists);
  std::vector<int>::iterator correspondence_indices_end = correspondence_indices.end ();
  std::sort (correspondence_indices.begin (), correspondence_indices_end);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Iterate over the remaining features and continuously narrow down the set of corresponding point
    for (typename FeaturesMap::const_iterator it = features_map_.begin (); it != features_map_.end (); ++it)
    //for (++feature_itr; feature_itr != features_map_.end (); ++feature_itr)
    {
      feature_itr->second->findFeatureCorrespondences ((*indices_)[i], nn_indices, nn_dists);

      std::sort (nn_indices.begin (), nn_indices.end ());
      correspondence_indices_end = std::set_intersection (nn_indices.begin (), nn_indices.end (),
                                                          correspondence_indices.begin (), 
                                                          correspondence_indices_end,
                                                          correspondence_indices.begin ());
    }
  }

  // 'corresponding_indices' now contains the indices of the points that corresponded to 'index' for *all* 
  // features in 'feature_map_'.
  correspondence_indices.resize (correspondence_indices_end - correspondence_indices.begin ());
  */
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::determineCorrespondences (
    std::vector<pcl::Correspondence> &correspondences, float max_distance)
{
  typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;

  if (!initCompute ())
    return;

  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  float max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size ());
  std::vector<int> index (1);
  std::vector<float> distance (1);
  pcl::Correspondence corr;
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Copy the source data to a target PointTarget format so we can search in the tree
    PointTarget pt;
    pcl::for_each_type <FieldListTarget> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
          input_->points[(*indices_)[i]], 
          pt));

    //if (tree_->nearestKSearch (input_->points[(*indices_)[i]], 1, index, distance))
    if (tree_->nearestKSearch (pt, 1, index, distance))
    {
      if (distance[0] <= max_dist_sqr)
      {
        corr.index_query = i;
        corr.index_match = index[0];
        corr.distance = distance[0];
        correspondences[i] = corr;
        continue;
      }
    }
//    correspondences[i] = pcl::Correspondence(i, -1, std::numeric_limits<float>::max());
  }
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::determineReciprocalCorrespondences (
    std::vector<pcl::Correspondence> &correspondences)
{
  typedef typename pcl::traits::fieldList<PointSource>::type FieldListSource;
  typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
  typedef typename pcl::intersect<FieldListSource, FieldListTarget>::type FieldList;
  
  if (!initCompute ())
    return;

  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  // setup tree for reciprocal search
  pcl::KdTreeFLANN<PointSource> tree_reciprocal;
  tree_reciprocal.setInputCloud (input_, indices_);

  correspondences.resize (indices_->size());
  std::vector<int> index (1);
  std::vector<float> distance (1);
  std::vector<int> index_reciprocal (1);
  std::vector<float> distance_reciprocal (1);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Copy the source data to a target PointTarget format so we can search in the tree
    PointTarget pt_src;
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
          input_->points[(*indices_)[i]], 
          pt_src));

    //tree_->nearestKSearch (input_->points[(*indices_)[i]], 1, index, distance);
    tree_->nearestKSearch (pt_src, 1, index, distance);

    // Copy the target data to a target PointSource format so we can search in the tree_reciprocal
    PointSource pt_tgt;
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTarget, PointSource> (
          target_->points[index[0]],
          pt_tgt));
    //tree_reciprocal.nearestKSearch (target_->points[index[0]], 1, index_reciprocal, distance_reciprocal);
    tree_reciprocal.nearestKSearch (pt_tgt, 1, index_reciprocal, distance_reciprocal);

    if ((*indices_)[i] == index_reciprocal[0])
    {
      corr.index_query = (*indices_)[i];
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences] = corr;
      ++nr_valid_correspondences;
    }
  }
  correspondences.resize (nr_valid_correspondences);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureT> inline void 
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setFeatureRepresentation (
  const typename pcl::PointRepresentation<FeatureT>::ConstPtr &fr,
  const std::string &key)
{
  if (features_map_.count (key) == 0)
    features_map_[key].reset (new FeatureContainer<FeatureT>);
  boost::static_pointer_cast<FeatureContainer<FeatureT> > (features_map_[key])->setFeatureRepresentation (fr);
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */
