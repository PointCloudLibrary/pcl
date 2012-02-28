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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Affine3f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
      cloud_out.points[i].getVector3fMap () = transform * 
                                              cloud_in.points[i].getVector3fMap ();
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
      cloud_out.points[i].getVector3fMap () = transform * 
                                              cloud_in.points[i].getVector3fMap ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          const std::vector<int> &indices, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Affine3f &transform)
{
  size_t npts = indices.size ();
  // In order to transform the data, we need to remove NaNs
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = npts;
  cloud_out.height   = 1;
  cloud_out.points.resize (npts);

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < npts; ++i)
      cloud_out.points[i].getVector3fMap () = transform *
                                              cloud_in.points[indices[i]].getVector3fMap ();
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (size_t i = 0; i < npts; ++i)
    {
      if (!pcl_isfinite (cloud_in.points[indices[i]].x) || 
          !pcl_isfinite (cloud_in.points[indices[i]].y) || 
          !pcl_isfinite (cloud_in.points[indices[i]].z))
        continue;
      cloud_out.points[i].getVector3fMap () = transform *
                                              cloud_in.points[indices[i]].getVector3fMap ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Affine3f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      cloud_out.points[i].getVector3fMap() = transform * 
                                             cloud_in.points[i].getVector3fMap ();

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
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
      cloud_out.points[i].getVector3fMap() = transform * 
                                             cloud_in.points[i].getVector3fMap ();

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Matrix4f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
  Eigen::Vector3f trans = transform.block<3, 1> (0, 3);
  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
      cloud_out.points[i].getVector3fMap () = rot * 
                                              cloud_in.points[i].getVector3fMap () + trans;
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
      cloud_out.points[i].getVector3fMap () = rot * 
                                              cloud_in.points[i].getVector3fMap () + trans;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Matrix4f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
  Eigen::Vector3f trans = transform.block<3, 1> (0, 3);

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      cloud_out.points[i].getVector3fMap () = rot * 
                                              cloud_in.points[i].getVector3fMap () + trans;

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = rot * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
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
      cloud_out.points[i].getVector3fMap () = rot * 
                                              cloud_in.points[i].getVector3fMap () + trans;

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = rot * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Vector3f &offset, 
                          const Eigen::Quaternionf &rotation)
{
  Eigen::Translation3f translation (offset);
  // Assemble an Eigen Transform
  Eigen::Affine3f t;
  t = translation * rotation;
  transformPointCloud (cloud_in, cloud_out, t);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Vector3f &offset, 
                                     const Eigen::Quaternionf &rotation)
{
  Eigen::Translation3f translation (offset);
  // Assemble an Eigen Transform
  Eigen::Affine3f t;
  t = translation * rotation;
  transformPointCloudWithNormals (cloud_in, cloud_out, t);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline PointT
pcl::transformPoint (const PointT &point, const Eigen::Affine3f &tranform)
{
  PointT ret = point;
  ret.getVector3fMap () = tranform * point.getVector3fMap ();
  return (ret);
}
