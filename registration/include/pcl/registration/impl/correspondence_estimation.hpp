/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    ROS_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!", getClassName ().c_str ());
    return;
  }
  PointCloudTarget target = *cloud;
  // Set all the point.data[3] values to 1 to aid the rigid transformation
  for (size_t i = 0; i < target.points.size (); ++i)
    target.points[i].data[3] = 1.0;

  //target_ = cloud;
  target_ = target.makeShared ();
  tree_->setInputCloud (target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setSourceFeature (
    const typename pcl::PointCloud<FeatureType>::ConstPtr &source_feature, std::string key)
{
  if (features_map_.count (key) == 0)
  {
    features_map_[key] = boost::make_shared<FeatureContainer<FeatureType> > ();
  }
  boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->setSourceFeature (source_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline typename pcl::PointCloud<FeatureType>::ConstPtr
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::getSourceFeature (std::string key)
{
  if (features_map_.count (key) == 0)
  {
    return (boost::shared_ptr<pcl::PointCloud<const FeatureType> > ());
  }
  else
  {
    return (boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->getSourceFeature ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setTargetFeature (
    const typename pcl::PointCloud<FeatureType>::ConstPtr &target_feature, std::string key)
{
  if (features_map_.count (key) == 0)
  {
    features_map_[key] = boost::make_shared<FeatureContainer<FeatureType> > ();
  }
  boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->setTargetFeature (target_feature);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline typename pcl::PointCloud<FeatureType>::ConstPtr
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::getTargetFeature (std::string key)
{
  typedef pcl::PointCloud<FeatureType> FeatureCloud;
  typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

  if (features_map_.count (key) == 0)
  {
    return (boost::shared_ptr<const pcl::PointCloud<FeatureType> > ());
  }
  else
  {
    return (boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->getTargetFeature ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setRadiusSearch (const typename pcl::KdTree<FeatureType>::Ptr &tree,
                                                                float r, std::string key)
{
  if (features_map_.count (key) == 0)
  {
    features_map_[key] = boost::make_shared<FeatureContainer<FeatureType> > ();
  }
  boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->setRadiusSearch (tree, r);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template <typename FeatureType> inline void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::setKSearch (const typename pcl::KdTree<FeatureType>::Ptr &tree,
                                                           int k, std::string key)
{
  if (features_map_.count (key) == 0)
  {
    features_map_[key] = boost::make_shared<FeatureContainer<FeatureType> > ();
  }
  boost::static_pointer_cast<FeatureContainer<FeatureType> > (features_map_[key])->setKSearch (tree, k);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline bool
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::hasValidFeatures ()
{
  if (features_map_.empty ())
  {
    return (false);
  }
  typename FeaturesMap::const_iterator feature_itr;
  for (feature_itr = features_map_.begin (); feature_itr != features_map_.end (); ++feature_itr)
  {
    if (!feature_itr->second->isValid ())
    {
      return (false);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::findFeatureCorrespondences (int index,
                                                                           std::vector<int> &correspondence_indices)
{
  if (features_map_.empty ())
  {
    ROS_ERROR ("[pcl::%s::findFeatureCorrespondences] One or more features must be set before finding correspondences!",
               getClassName ().c_str ());
    return;
  }

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  // Find the correspondence indices for the first feature in the features map
  typename FeaturesMap::const_iterator feature_itr = features_map_.begin ();
  feature_itr->second->findFeatureCorrespondences (index, correspondence_indices, nn_dists);
  std::vector<int>::iterator correspondence_indices_end = correspondence_indices.end ();
  std::sort (correspondence_indices.begin (), correspondence_indices_end);

  // Iterate over the remaining features and continuously narrow down the set of corresponding point
  for (++feature_itr; feature_itr != features_map_.end (); ++feature_itr)
  {
    feature_itr->second->findFeatureCorrespondences (index, nn_indices, nn_dists);

    sort (nn_indices.begin (), nn_indices.end ());
    correspondence_indices_end = set_intersection (nn_indices.begin (), nn_indices.end (),
                                                   correspondence_indices.begin (), correspondence_indices_end,
                                                   correspondence_indices.begin ());
  }

  // 'corresponding_indices' now contains the indices of the points that corresponded to 'index' for *all* features
  // in 'feature_map_'.
  correspondence_indices.resize (correspondence_indices_end - correspondence_indices.begin());
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::determineCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences, float max_distance)
{
  if (!initCompute())
    return;

  if (!target_)
  {
    ROS_WARN ("[pcl::%s::compute] No input target dataset was given!", getClassName ().c_str ());
    return;
  }

  float max_dist_sqr = max_distance * max_distance;

  correspondences.resize(indices_->size());
  std::vector<int> index(1);
  std::vector<float> distance(1);
  pcl::registration::Correspondence corr;
  for (unsigned int i = 0; i < indices_->size(); ++i)
  {
    if ( tree_->nearestKSearch(input_->points[(*indices_)[i]], 1, index, distance) )
    {
      if ( distance[0] <= max_dist_sqr )
      {
        corr.indexQuery = i;
        corr.indexMatch = index[0];
        corr.distance = distance[0];
        correspondences[i] = corr;
        continue;
      }
    }
//    correspondences[i] = pcl::registration::Correspondence(i, -1, std::numeric_limits<float>::max());
  }
  deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::determineReciprocalCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences)
{
  if (!initCompute())
    return;

  if (!target_)
  {
    ROS_WARN ("[pcl::%s::compute] No input target dataset was given!", getClassName ().c_str ());
    return;
  }

  // setup tree for reciprocal search
  pcl::KdTreeFLANN<PointTarget> tree_reciprocal;
  tree_reciprocal.setInputCloud(input_, indices_);

  correspondences.resize(indices_->size());
  std::vector<int> index(1);
  std::vector<float> distance(1);
  std::vector<int> index_reciprocal(1);
  std::vector<float> distance_reciprocal(1);
  pcl::registration::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

//  #pragma omp parallel for shared( input_tree, output_tree )
  for (unsigned int i = 0; i < indices_->size(); ++i)
  {
    tree_->nearestKSearch(input_->points[(*indices_)[i]], 1, index, distance);
    tree_reciprocal.nearestKSearch(target_->points[index[0]], 1, index_reciprocal, distance_reciprocal);

//    #pragma omp critical
    if ( (*indices_)[i] == index_reciprocal[0] )
    {
      corr.indexQuery = (*indices_)[i];
      corr.indexMatch = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences] = corr;
      ++nr_valid_correspondences;
    }
  }
  correspondences.resize(nr_valid_correspondences);

  deinitCompute();
}



#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */
