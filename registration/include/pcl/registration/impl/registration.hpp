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
 * $Id: registration.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::estimateRigidTransformationSVD (const pcl::PointCloud<PointSource> &cloud_src, 
                                     const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                     Eigen::Matrix4f &transformation_matrix)
{
  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    ROS_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%zu) differs than target (%zu)!", cloud_src.points.size (), cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, centroid_src);
  compute3DCentroid (cloud_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix3f R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner<3, 3> () = R;
  Eigen::Vector3f Rc = R * centroid_src.head<3> ();
  transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::estimateRigidTransformationSVD (const pcl::PointCloud<PointSource> &cloud_src, 
                                     const std::vector<int> &indices_src, 
                                     const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                     const std::vector<int> &indices_tgt, 
                                     Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () != indices_tgt.size ())
  {
    ROS_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%zu) differs than target (%zu)!", indices_src.size (), indices_tgt.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix3f R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner<3, 3> () = R;
  Eigen::Vector3f Rc = R * centroid_src.head<3> ();
  transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::estimateRigidTransformationSVD (const pcl::PointCloud<PointSource> &cloud_src, 
                                     const std::vector<int> &indices_src, 
                                     const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                     Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    ROS_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%zu) differs than target (%zu)!", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3f &u = svd.matrixU ();
  const Eigen::Matrix3f &v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix3f R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner<3, 3> () = R;
  Eigen::Vector3f Rc = R * centroid_src.head<3> ();
  transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
  pcl::Registration<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
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
pcl::Registration<PointSource, PointTarget>::setSourceFeature (
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
pcl::Registration<PointSource, PointTarget>::getSourceFeature (std::string key)
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
pcl::Registration<PointSource, PointTarget>::setTargetFeature (
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
  pcl::Registration<PointSource, PointTarget>::getTargetFeature (std::string key)
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
pcl::Registration<PointSource, PointTarget>::setRadiusSearch (const typename pcl::KdTree<FeatureType>::Ptr &tree, 
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
pcl::Registration<PointSource, PointTarget>::setKSearch (const typename pcl::KdTree<FeatureType>::Ptr &tree,
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
  pcl::Registration<PointSource, PointTarget>::hasValidFeatures ()
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
pcl::Registration<PointSource, PointTarget>::findFeatureCorrespondences (int index, 
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

    std::sort (nn_indices.begin (), nn_indices.end ());
    correspondence_indices_end = set_intersection (nn_indices.begin (), nn_indices.end (),
                                                   correspondence_indices.begin (), correspondence_indices_end, 
                                                   correspondence_indices.begin ());
  }

  // 'corresponding_indices' now contains the indices of the points that corresponded to 'index' for *all* features
  // in 'feature_map_'.
  correspondence_indices.resize (correspondence_indices_end - correspondence_indices.begin());
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
pcl::Registration<PointSource, PointTarget>::getFitnessScore (double max_range)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  transformPointCloud (*input_, input_transformed, final_transformation_);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input_transformed.points[i].x,
                                          input_transformed.points[i].y,
                                          input_transformed.points[i].z, 0);
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    
    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target_->points[nn_indices[0]].x,
                                          target_->points[nn_indices[0]].y,
                                          target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::align (PointCloudSource &output)
{
  if (!initCompute ()) return;

  if (!target_)
  {
    ROS_WARN ("[pcl::%s::compute] No input target dataset was given!", getClassName ().c_str ());
    return;
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = indices_->size ();
    output.height   = 1;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix4f::Identity ();

  // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid 
  // transformation
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i].data[3] = 1.0;

  computeTransformation (output);

  deinitCompute ();
}

