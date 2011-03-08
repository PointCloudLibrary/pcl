/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: feature.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid)
{
  // Initialize to 0
  centroid.setZero ();
  if (cloud.points.empty ()) 
    return;
  // For each point in the cloud
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      centroid += cloud.points[i].getVector4fMap ();
    centroid[3] = 0;
    centroid /= cloud.points.size ();
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;

      centroid += cloud.points[i].getVector4fMap ();
      cp++;
    }
    centroid[3] = 0;
    centroid /= cp;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                        Eigen::Vector4f &centroid)
{
  // Initialize to 0
  centroid.setZero ();
  if (indices.empty ()) 
    return;
  // For each point in the cloud
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
      centroid += cloud.points[indices[i]].getVector4fMap ();
    centroid[3] = 0;
    centroid /= indices.size ();
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

      centroid += cloud.points[indices[i]].getVector4fMap ();
      cp++;
    }
    centroid[3] = 0;
    centroid /= cp;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                        const pcl::PointIndices &indices, Eigen::Vector4f &centroid)
{
  return (pcl::compute3DCentroid<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::VectorXf &centroid)
{
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (cloud.points.empty ())
    return;
  // Iterate over each point
  int size = cloud.points.size ();
  for (int i = 0; i < size; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[i], centroid));
  }
  centroid /= size;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                        Eigen::VectorXf &centroid)
{
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (indices.empty ()) 
    return;
  // Iterate over each point
  int nr_points = indices.size ();
  for (int i = 0; i < nr_points; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[indices[i]], centroid));
  }
  centroid /= nr_points;
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                        const pcl::PointIndices &indices, Eigen::VectorXf &centroid)
{
  return (pcl::computeNDCentroid<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  // Initialize to 0
  covariance_matrix.setZero ();

  if (cloud.points.empty ())
    return;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // For each point in the cloud
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      Eigen::Vector4f pt = cloud.points[i].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    // For each point in the cloud
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;

      Eigen::Vector4f pt = cloud.points[i].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                        const Eigen::Vector4f &centroid, 
                                        Eigen::Matrix3f &covariance_matrix)
{
  pcl::computeCovarianceMatrix (cloud, centroid, covariance_matrix);
  covariance_matrix /= cloud.points.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                              const std::vector<int> &indices,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  // Initialize to 0
  covariance_matrix.setZero ();

  if (indices.empty ())
    return;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
    {
      Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

      Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                              const pcl::PointIndices &indices,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                        const std::vector<int> &indices,
                                        const Eigen::Vector4f &centroid, 
                                        Eigen::Matrix3f &covariance_matrix)
{
  pcl::computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix);
  covariance_matrix /= indices.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
  pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                          const pcl::PointIndices &indices,
                                          const Eigen::Vector4f &centroid, 
                                          Eigen::Matrix3f &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix (cloud, indices.indices, centroid, covariance_matrix));
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix, 
                           const Eigen::Vector4f &point,
                           Eigen::Vector4f &plane_parameters, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!pcl_isfinite (covariance_matrix (i, j)))
      {
        //ROS_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!");
        plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
        curvature = std::numeric_limits<float>::quiet_NaN ();
        return;
      }

  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  //EIGEN_ALIGN16 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
  //EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  //float norm = 1.0 / eigen_vectors.col (0).norm ();

  //plane_parameters[0] = eigen_vectors (0, 0) * norm;
  //plane_parameters[1] = eigen_vectors (1, 0) * norm;
  //plane_parameters[2] = eigen_vectors (2, 0) * norm;

  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
  plane_parameters[0] = eigen_vectors (0, 0);
  plane_parameters[1] = eigen_vectors (1, 0);
  plane_parameters[2] = eigen_vectors (2, 0);
  plane_parameters[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  plane_parameters[3] = -1 * plane_parameters.dot (point);

  // Compute the curvature surface change
  float eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                           float &nx, float &ny, float &nz, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!pcl_isfinite (covariance_matrix (i, j)))
      {
        //ROS_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!");
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        return;
      }
  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  //EIGEN_ALIGN16 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
  //EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  //float norm = 1.0 / eigen_vectors.col (0).norm ();

  //nx = eigen_vectors (0, 0) * norm;
  //ny = eigen_vectors (1, 0) * norm;
  //nz = eigen_vectors (2, 0) * norm;
  
  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
  nx = eigen_vectors (0, 0);
  ny = eigen_vectors (1, 0);
  nz = eigen_vectors (2, 0);

  // Compute the curvature surface change
  float eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Feature<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  // Copy the header
  output.header = input_->header;

  if (!initCompute ()) 
  {
    ROS_ERROR ("[pcl::%s::compute] Init failed.", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    deinitCompute ();
    return;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    ROS_ERROR ("[pcl::%s::compute] No spatial search method was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    deinitCompute ();
    return;
  }

  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Send the surface dataset to the spatial locator
  tree_->setInputCloud (surface_);

  // Do a fast check to see if the search parameters are well defined
  if (search_radius_ != 0.0)
  {
    if (k_ != 0)
    {
      ROS_ERROR ("[pcl::%s::compute] Both radius (%f) and K (%d) defined! Set one of them to zero first and then re-run compute ().", getClassName ().c_str (), search_radius_, k_);
      output.width = output.height = 0;
      output.points.clear ();

      // Cleanup
      deinitCompute ();
      // Reset the surface
      if (fake_surface_)
      {
        surface_.reset ();
        fake_surface_ = false;
      }
      return;
    }
    else                  // Use the radiusSearch () function
    {
      search_parameter_ = search_radius_;
      if (surface_ == input_)       // if the two surfaces are the same
      {
        // Declare the search locator definition
        int (KdTree::*radiusSearch)(int index, double radius, std::vector<int> &k_indices,
                                    std::vector<float> &k_distances, int max_nn) const = &KdTree::radiusSearch;
        search_method_ = boost::bind (radiusSearch, boost::ref (tree_), _1, _2, _3, _4, INT_MAX);
      }
      else
      {
        // Declare the search locator definition
        int (KdTree::*radiusSearchSurface)(const PointCloudIn &cloud, int index, double radius, std::vector<int> &k_indices,
                                            std::vector<float> &k_distances, int max_nn) const = &KdTree::radiusSearch;
        search_method_surface_ = boost::bind (radiusSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5, INT_MAX);
      }
    }
  }
  else
  {
    if (k_ != 0)         // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      if (surface_ == input_)       // if the two surfaces are the same
      {
        // Declare the search locator definition
        int (KdTree::*nearestKSearch)(int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = &KdTree::nearestKSearch;
        search_method_ = boost::bind (nearestKSearch, boost::ref (tree_), _1, _2, _3, _4);
      }
      else
      {
        // Declare the search locator definition
        int (KdTree::*nearestKSearchSurface)(const PointCloudIn &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = &KdTree::nearestKSearch;
        search_method_surface_ = boost::bind (nearestKSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5);
      }
    }
    else
    {
      ROS_ERROR ("[pcl::%s::compute] Neither radius nor K defined! Set one of them to a positive number first and then re-run compute ().", getClassName ().c_str ());
      output.width = output.height = 0;
      output.points.clear ();

      // Cleanup
      deinitCompute ();
      // Reset the surface
      if (fake_surface_)
      {
        surface_.reset ();
        fake_surface_ = false;
      }
      return;
    }
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
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

  // Perform the actual feature computation
  computeFeature (output);

  deinitCompute ();

  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
}

