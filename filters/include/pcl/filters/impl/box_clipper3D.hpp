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

#ifndef PCL_FILTERS_IMPL_BOX_CLIPPER3D_HPP
#define PCL_FILTERS_IMPL_BOX_CLIPPER3D_HPP

#include <pcl/filters/box_clipper3D.h>

template<typename PointT>
pcl::BoxClipper3D<PointT>::BoxClipper3D (const Eigen::Affine3f& transformation)
: transformation_ (transformation)
{
  //inverse_transformation_ = transformation_.inverse ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::BoxClipper3D<PointT>::BoxClipper3D (const Eigen::Vector3f& rodrigues, const Eigen::Vector3f& translation, const Eigen::Vector3f& box_size)
{
  setTransformation (rodrigues, translation, box_size);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::BoxClipper3D<PointT>::~BoxClipper3D () throw ()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::BoxClipper3D<PointT>::setTransformation (const Eigen::Affine3f& transformation)
{
  transformation_ = transformation;
  //inverse_transformation_ = transformation_.inverse ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::BoxClipper3D<PointT>::setTransformation (const Eigen::Vector3f& rodrigues, const Eigen::Vector3f& translation, const Eigen::Vector3f& box_size)
{
  transformation_ = Eigen::Translation3f (translation) * Eigen::AngleAxisf(rodrigues.norm (), rodrigues.normalized ()) * Eigen::Scaling (box_size);
  //inverse_transformation_ = transformation_.inverse ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> pcl::Clipper3D<PointT>*
pcl::BoxClipper3D<PointT>::clone () const
{
  return new BoxClipper3D<PointT> (transformation_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::BoxClipper3D<PointT>::transformPoint (const PointT& pointIn, PointT& pointOut) const
{
  const Eigen::Vector4f& point = pointIn.getVector4fMap ();
  pointOut.getVector4fMap () = transformation_ * point;

  // homogeneous value might not be 1
  if (point [3] != 1)
  {
    // homogeneous component might be uninitialized -> invalid
    if (point [3] != 0)
    {
      pointOut.x += (1 - point [3]) * transformation_.data () [ 9];
      pointOut.y += (1 - point [3]) * transformation_.data () [10];
      pointOut.z += (1 - point [3]) * transformation_.data () [11];
    }
    else
    {
      pointOut.x += transformation_.data () [ 9];
      pointOut.y += transformation_.data () [10];
      pointOut.z += transformation_.data () [11];
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ToDo use product on point.getVector3fMap () and transformatio_.col (i) to use the SSE advantages of eigen
template<typename PointT> bool
pcl::BoxClipper3D<PointT>::clipPoint3D (const PointT& point) const
{
  return  (fabs(transformation_.data () [ 0] * point.x +
                transformation_.data () [ 3] * point.y +
                transformation_.data () [ 6] * point.z +
                transformation_.data () [ 9]) <= 1 &&
           fabs(transformation_.data () [ 1] * point.x +
                transformation_.data () [ 4] * point.y +
                transformation_.data () [ 7] * point.z +
                transformation_.data () [10]) <= 1 &&
           fabs(transformation_.data () [ 2] * point.x +
                transformation_.data () [ 5] * point.y +
                transformation_.data () [ 8] * point.z +
                transformation_.data () [11]) <= 1 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT> bool
pcl::BoxClipper3D<PointT>::clipLineSegment3D (PointT& point1, PointT& point2) const
{
  /*
  PointT pt1, pt2;
  transformPoint (point1, pt1);
  transformPoint (point2, pt2);

  //
  bool pt1InBox = (fabs(pt1.x) <= 1.0 && fabs (pt1.y) <= 1.0 && fabs (pt1.z) <= 1.0);
  bool pt2InBox = (fabs(pt2.x) <= 1.0 && fabs (pt2.y) <= 1.0 && fabs (pt2.z) <= 1.0);

  // one is outside the other one inside the box
  //if (pt1InBox ^ pt2InBox)
  if (pt1InBox && !pt2InBox)
  {
    PointT diff;
    PointT lambda;
    diff.getVector3fMap () = pt2.getVector3fMap () - pt1.getVector3fMap ();

    if (diff.x > 0)
      lambda.x = (1.0 - pt1.x) / diff.x;
    else
      lambda.x = (-1.0 - pt1.x) / diff.x;

    if (diff.y > 0)
      lambda.y = (1.0 - pt1.y) / diff.y;
    else
      lambda.y = (-1.0 - pt1.y) / diff.y;

    if (diff.z > 0)
      lambda.z = (1.0 - pt1.z) / diff.z;
    else
      lambda.z = (-1.0 - pt1.z) / diff.z;

    pt2 = pt1 + std::min(std::min(lambda.x, lambda.y), lambda.z) * diff;

    // inverse transformation
    inverseTransformPoint (pt2, point2);
    return true;
  }
  else if (!pt1InBox && pt2InBox)
  {
    return true;
  }
  */
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT> void
pcl::BoxClipper3D<PointT>::clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const
{
  // not implemented -> clip everything
  clipped_polygon.clear ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @attention untested code
 */
template<typename PointT> void
pcl::BoxClipper3D<PointT>::clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon) const
{
  // not implemented -> clip everything
  polygon.clear ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /ToDo: write fast version using eigen map and single matrix vector multiplication, that uses advantages of eigens SSE operations.
template<typename PointT> void
pcl::BoxClipper3D<PointT>::clipPointCloud3D (const pcl::PointCloud<PointT>& cloud_in, std::vector<int>& clipped, const std::vector<int>& indices) const
{
  if (indices.empty ())
  {
    clipped.reserve (cloud_in.size ());
    for (register unsigned pIdx = 0; pIdx < cloud_in.size (); ++pIdx)
      if (clipPoint3D (cloud_in[pIdx]))
        clipped.push_back (pIdx);
  }
  else
  {
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
      if (clipPoint3D (cloud_in[*iIt]))
        clipped.push_back (*iIt);
  }
}
#endif //PCL_FILTERS_IMPL_BOX_CLIPPER3D_HPP