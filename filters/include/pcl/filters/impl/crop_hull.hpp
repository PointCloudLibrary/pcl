 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2011, Willow Garage, Inc.
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
  */

#ifndef PCL_FILTERS_IMPL_CROP_HULL_H_
#define PCL_FILTERS_IMPL_CROP_HULL_H_

#include <pcl/filters/crop_hull.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropHull<PointT>::applyFilter (pcl::Indices &indices)
{
  indices.clear();
  removed_indices_->clear();
  indices.reserve(indices_->size());
  if (extract_removed_indices_) {
    removed_indices_->reserve(indices_->size());
  }

  if (dim_ == 2)
  {
    // choose to squash the XYZ component of the
    // hull-points that has least variation - this will also give reasonable
    // results if the points don't lie exactly in the same plane
    const Eigen::Vector3f range = getHullCloudRange ();
    if (range[0] <= range[1] && range[0] <= range[2])
      applyFilter2D<1,2> (indices);
    else if (range[1] <= range[2] && range[1] <= range[0])
      applyFilter2D<2,0> (indices);
    else
      applyFilter2D<0,1> (indices);
  }
  else
  {
    applyFilter3D (indices);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> Eigen::Vector3f
pcl::CropHull<PointT>::getHullCloudRange ()
{
  Eigen::Vector3f cloud_min (
    std::numeric_limits<float>::max (),
    std::numeric_limits<float>::max (),
    std::numeric_limits<float>::max ()
  );
  Eigen::Vector3f cloud_max (
    -std::numeric_limits<float>::max (),
    -std::numeric_limits<float>::max (),
    -std::numeric_limits<float>::max ()
  );
  for (pcl::Vertices const & poly : hull_polygons_)
  {
    for (std::uint32_t const & idx : poly.vertices)
    {
      Eigen::Vector3f pt = hull_cloud_->points[idx].getVector3fMap ();
      for (int i = 0; i < 3; i++)
      {
        if (pt[i] < cloud_min[i]) cloud_min[i] = pt[i];
        else if (pt[i] > cloud_max[i]) cloud_max[i] = pt[i];
      }
    }
  }

  return (cloud_max - cloud_min);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> template<unsigned PlaneDim1, unsigned PlaneDim2> void
pcl::CropHull<PointT>::applyFilter2D (pcl::Indices &indices)
{
  const bool keepInside = crop_outside_ ^ negative_;

  const Eigen::Vector4f minPt = crop_box_.getMin();
  const Eigen::Vector4f maxPt = crop_box_.getMax();
  std::vector<bool> cropBoxInlierMask(input_->size(), true);
  for (const index_t idx : *indices_)
  {
    Eigen::Vector3f pt = input_->points[idx].getVector3fMap();
    if (pt[PlaneDim1] < minPt[PlaneDim1] || pt[PlaneDim1] > maxPt[PlaneDim1] ||
        pt[PlaneDim2] < minPt[PlaneDim2] || pt[PlaneDim2] > maxPt[PlaneDim2])
    {
      cropBoxInlierMask.at(idx) = false;
      if (!keepInside) {
        indices.push_back(idx);
      }
      else if (extract_removed_indices_) {
        removed_indices_->push_back(idx);
      }
    }
  }

  for (const index_t idx : *indices_)
  {
    if (!cropBoxInlierMask[idx]) {
      continue;
    }
    // iterate over polygons faster than points because we expect this data
    // to be, in general, more cache-local - the point cloud might be huge
    const auto poly_it = std::find_if(
        hull_polygons_.cbegin(), hull_polygons_.cend(),
        [idx, this](const Vertices & poly) -> bool {
          return isPointIn2DPolyWithVertIndices<PlaneDim1,PlaneDim2> (
              input_->points[idx], poly, *hull_cloud_);
        });
    const bool found_in_polygons = (poly_it != hull_polygons_.cend());
    if (keepInside == found_in_polygons) {
      // valid index: either found inside and need to keep inside
      //                     or not found inside and need to remove inside
      indices.push_back (idx);
    }
    else if (extract_removed_indices_) {
      removed_indices_->push_back (idx);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropHull<PointT>::applyFilter3D (pcl::Indices &indices)
{
  // This algorithm could definitely be sped up using kdtree/octree
  // information, if that is available!

  const bool keepInside = crop_outside_ ^ negative_;

  pcl::Indices cropInlierIndices;
  crop_box_.setIndices(indices_);
  crop_box_.filter(cropInlierIndices);
  std::vector<bool> cropBoxInlierMask(input_->size(), false);
  for (index_t idx : cropInlierIndices) {
    cropBoxInlierMask.at(idx) = true;
  }

  for (index_t idx : *indices_)
  {
    if (!cropBoxInlierMask[idx]) {
      continue;
    }
    // test ray-crossings for three random rays, and take vote of crossings
    // counts to determine if each point is inside the hull: the vote avoids
    // tricky edge and corner cases when rays might fluke through the edge
    // between two polygons
    // 'random' rays are arbitrary - basically anything that is less likely to
    // hit the edge between polygons than coordinate-axis aligned rays would
    // be.
    std::size_t crossings[3] = {0,0,0};
    Eigen::Vector3f rays[3] =
    {
      Eigen::Vector3f(0.264882f,  0.688399f, 0.675237f),
      Eigen::Vector3f(0.0145419f, 0.732901f, 0.68018f),
      Eigen::Vector3f(0.856514f,  0.508771f, 0.0868081f)
    };

    for (std::size_t poly = 0; poly < hull_polygons_.size (); poly++)
      for (std::size_t ray = 0; ray < 3; ray++)
        crossings[ray] += rayTriangleIntersect
          (input_->points[idx], rays[ray], hull_polygons_[poly], *hull_cloud_);

    bool isPointInsideHull = (crossings[0]&1) + (crossings[1]&1) + (crossings[2]&1) > 1;
    if (keepInside == isPointInsideHull) {
      indices.push_back (idx);
    }
    else if (extract_removed_indices_) {
      removed_indices_->push_back (idx);
    }
  }

  if (!keepInside)
  {
    indices.insert(
        indices.end(),
        crop_box_.getRemovedIndices()->begin(),
        crop_box_.getRemovedIndices()->end());
  }
  else if (extract_removed_indices_)
  {
    removed_indices_->insert(
        removed_indices_->end(),
        crop_box_.getRemovedIndices()->begin(),
        crop_box_.getRemovedIndices()->end());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> template<unsigned PlaneDim1, unsigned PlaneDim2> bool
pcl::CropHull<PointT>::isPointIn2DPolyWithVertIndices (
  const PointT& point, const Vertices& verts, const PointCloud& cloud)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  const int nr_poly_points = static_cast<int>(verts.vertices.size ());
  double xold = cloud[verts.vertices[nr_poly_points - 1]].getVector3fMap ()[PlaneDim1];
  double yold = cloud[verts.vertices[nr_poly_points - 1]].getVector3fMap ()[PlaneDim2];
  for (int i = 0; i < nr_poly_points; i++)
  {
    const double xnew = cloud[verts.vertices[i]].getVector3fMap ()[PlaneDim1];
    const double ynew = cloud[verts.vertices[i]].getVector3fMap ()[PlaneDim2];
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ((xnew < point.getVector3fMap ()[PlaneDim1]) == (point.getVector3fMap ()[PlaneDim1] <= xold) &&
        (point.getVector3fMap ()[PlaneDim2] - y1) * (x2 - x1) < (y2 - y1) * (point.getVector3fMap ()[PlaneDim1] - x1))
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> bool
pcl::CropHull<PointT>::rayTriangleIntersect (const PointT& point,
                                             const Eigen::Vector3f& ray,
                                             const Vertices& verts,
                                             const PointCloud& cloud)
{
  // Algorithm here is adapted from:
  // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
  //
  // Original copyright notice:
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  //
  assert (verts.vertices.size () == 3);

  const Eigen::Vector3f p = point.getVector3fMap ();
  const Eigen::Vector3f a = cloud[verts.vertices[0]].getVector3fMap ();
  const Eigen::Vector3f b = cloud[verts.vertices[1]].getVector3fMap ();
  const Eigen::Vector3f c = cloud[verts.vertices[2]].getVector3fMap ();
  const Eigen::Vector3f u = b - a;
  const Eigen::Vector3f v = c - a;
  const Eigen::Vector3f n = u.cross (v);
  const float n_dot_ray = n.dot (ray);

  if (std::fabs (n_dot_ray) < 1e-9)
    return (false);

  const float r = n.dot (a - p) / n_dot_ray;

  if (r < 0)
    return (false);

  const Eigen::Vector3f w = p + r * ray - a;
  const float denominator = u.dot (v) * u.dot (v) - u.dot (u) * v.dot (v);
  const float s_numerator = u.dot (v) * w.dot (v) - v.dot (v) * w.dot (u);
  const float s = s_numerator / denominator;
  if (s < 0 || s > 1)
    return (false);

  const float t_numerator = u.dot (v) * w.dot (u) - u.dot (u) * w.dot (v);
  const float t = t_numerator / denominator;
  if (t < 0 || s+t > 1)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_CropHull(T) template class PCL_EXPORTS pcl::CropHull<T>;

#endif // PCL_FILTERS_IMPL_CROP_HULL_H_
