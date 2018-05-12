/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                          bool copy_all_fields)
{
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_in.points.size ());
    if (copy_all_fields)
      cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    else
      cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
    }
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          const std::vector<int> &indices, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                          bool copy_all_fields)
{
  size_t npts = indices.size ();
  // In order to transform the data, we need to remove NaNs
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = static_cast<int> (npts);
  cloud_out.height   = 1;
  cloud_out.points.resize (npts);
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_      = cloud_in.sensor_origin_;

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < npts; ++i)
    {
      // Copy fields first, then transform xyz data
      if (copy_all_fields)
        cloud_out.points[i] = cloud_in.points[indices[i]];
      //cloud_out.points[i].getVector3fMap () = transform*cloud_out.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[indices[i]].x, cloud_in[indices[i]].y, cloud_in[indices[i]].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
    }
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (size_t i = 0; i < npts; ++i)
    {
      if (copy_all_fields)
        cloud_out.points[i] = cloud_in.points[indices[i]];
      if (!pcl_isfinite (cloud_in.points[indices[i]].x) || 
          !pcl_isfinite (cloud_in.points[indices[i]].y) || 
          !pcl_isfinite (cloud_in.points[indices[i]].z))
        continue;
      //cloud_out.points[i].getVector3fMap () = transform*cloud_out.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[indices[i]].x, cloud_in[indices[i]].y, cloud_in[indices[i]].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                     bool copy_all_fields)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    if (copy_all_fields)
      cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    else
      cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      //cloud_out.points[i].getVector3fMap() = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

      // Rotate normals (WARNING: transform.rotation () uses SVD internally!)
      //cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * cloud_in.points[i].getNormalVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> nt (cloud_in[i].normal_x, cloud_in[i].normal_y, cloud_in[i].normal_z);
      cloud_out[i].normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
      cloud_out[i].normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
      cloud_out[i].normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;

      //cloud_out.points[i].getVector3fMap() = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

      // Rotate normals
      //cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * cloud_in.points[i].getNormalVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> nt (cloud_in[i].normal_x, cloud_in[i].normal_y, cloud_in[i].normal_z);
      cloud_out[i].normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
      cloud_out[i].normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
      cloud_out[i].normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     const std::vector<int> &indices, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                     bool copy_all_fields)
{
  size_t npts = indices.size ();
  // In order to transform the data, we need to remove NaNs
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = static_cast<int> (npts);
  cloud_out.height   = 1;
  cloud_out.points.resize (npts);
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_      = cloud_in.sensor_origin_;

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      // Copy fields first, then transform
      if (copy_all_fields)
        cloud_out.points[i] = cloud_in.points[indices[i]];
      //cloud_out.points[i].getVector3fMap() = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[indices[i]].x, cloud_in[indices[i]].y, cloud_in[indices[i]].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

      // Rotate normals
      //cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * cloud_in.points[i].getNormalVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> nt (cloud_in[indices[i]].normal_x, cloud_in[indices[i]].normal_y, cloud_in[indices[i]].normal_z);
      cloud_out[i].normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
      cloud_out[i].normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
      cloud_out[i].normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      // Copy fields first, then transform
      if (copy_all_fields)
        cloud_out.points[i] = cloud_in.points[indices[i]];

      if (!pcl_isfinite (cloud_in.points[indices[i]].x) || 
          !pcl_isfinite (cloud_in.points[indices[i]].y) || 
          !pcl_isfinite (cloud_in.points[indices[i]].z))
        continue;

      //cloud_out.points[i].getVector3fMap() = transform * cloud_in.points[i].getVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[indices[i]].x, cloud_in[indices[i]].y, cloud_in[indices[i]].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

      // Rotate normals
      //cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * cloud_in.points[i].getNormalVector3fMap ();
      Eigen::Matrix<Scalar, 3, 1> nt (cloud_in[indices[i]].normal_x, cloud_in[indices[i]].normal_y, cloud_in[indices[i]].normal_z);
      cloud_out[i].normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
      cloud_out[i].normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
      cloud_out[i].normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> inline void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Matrix<Scalar, 3, 1> &offset, 
                          const Eigen::Quaternion<Scalar> &rotation,
                          bool copy_all_fields)
{
  Eigen::Translation<Scalar, 3> translation (offset);
  // Assemble an Eigen Transform
  Eigen::Transform<Scalar, 3, Eigen::Affine> t (translation * rotation);
  transformPointCloud (cloud_in, cloud_out, t, copy_all_fields);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> inline void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Matrix<Scalar, 3, 1> &offset, 
                                     const Eigen::Quaternion<Scalar> &rotation,
                                     bool copy_all_fields)
{
  Eigen::Translation<Scalar, 3> translation (offset);
  // Assemble an Eigen Transform
  Eigen::Transform<Scalar, 3, Eigen::Affine> t (translation * rotation);
  transformPointCloudWithNormals (cloud_in, cloud_out, t, copy_all_fields);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> inline PointT
pcl::transformPoint (const PointT &point, 
                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
{
  PointT ret = point;
  ret.getVector3fMap () = transform * point.getVector3fMap ();

  return (ret);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> inline PointT
pcl::transformPointWithNormal (const PointT &point, 
                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
{
  PointT ret = point;
  ret.getVector3fMap () = transform * point.getVector3fMap ();
  ret.getNormalVector3fMap () = transform.rotation () * point.getNormalVector3fMap ();

  return (ret);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Scalar> double
pcl::getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                                 Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
{
  EIGEN_ALIGN16 Eigen::Matrix<Scalar, 3, 3> covariance_matrix;
  Eigen::Matrix<Scalar, 4, 1> centroid;
  
  pcl::computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid);

  EIGEN_ALIGN16 Eigen::Matrix<Scalar, 3, 3> eigen_vects;
  Eigen::Matrix<Scalar, 3, 1> eigen_vals;
  pcl::eigen33 (covariance_matrix, eigen_vects, eigen_vals);

  double rel1 = eigen_vals.coeff (0) / eigen_vals.coeff (1);
  double rel2 = eigen_vals.coeff (1) / eigen_vals.coeff (2);
  
  transform.translation () = centroid.head (3);
  transform.linear () = eigen_vects;
  
  return (std::min (rel1, rel2));
}

