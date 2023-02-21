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

#include <atomic>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::CropHull<PointT>::buildTree ()
{
  tree_.setInputMesh(hull_cloud_, hull_polygons_);
  tree_dirty_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropHull<PointT>::applyFilter (Indices &indices)
{
  indices.clear();
  removed_indices_->clear();
  indices.reserve(indices_->size());
  removed_indices_->reserve(indices_->size());
  if (dim_ == 2)
  {
    // in this case we are assuming all the points lie in the same plane as the
    // 2D convex hull, so the choice of projection just changes the
    // conditioning of the problem: choose to squash the XYZ component of the
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
    for (auto const & idx : poly.vertices)
    {
      Eigen::Vector3f pt = (*hull_cloud_)[idx].getVector3fMap ();
      cloud_min = cloud_min.cwiseMin(pt);
      cloud_max = cloud_max.cwiseMax(pt);
    }
  }

  return (cloud_max - cloud_min);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> template<unsigned PlaneDim1, unsigned PlaneDim2> void 
pcl::CropHull<PointT>::applyFilter2D (Indices &indices)
{
  for (std::size_t index = 0; index < indices_->size (); index++)
  {
    // iterate over polygons faster than points because we expect this data
    // to be, in general, more cache-local - the point cloud might be huge
    std::size_t poly;
    for (poly = 0; poly < hull_polygons_.size (); poly++)
    {
      if (isPointIn2DPolyWithVertIndices<PlaneDim1,PlaneDim2> (
              (*input_)[(*indices_)[index]], hull_polygons_[poly], *hull_cloud_
         ))
      {
        if (crop_outside_)
          indices.push_back ((*indices_)[index]);
        // once a point has tested +ve for being inside one polygon, we can
        // stop checking the others:
        break;
      }
    }
    // If we're removing points *inside* the hull, only remove points that
    // haven't been found inside any polygons
    if (poly == hull_polygons_.size () && !crop_outside_)
      indices.push_back ((*indices_)[index]);
    if (indices.empty() || indices.back() != (*indices_)[index]) {
      removed_indices_->push_back ((*indices_)[index]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::CropHull<PointT>::applyFilter3D(Indices& indices)
{
  if (tree_dirty_)
    buildTree();

  std::array<Eigen::Vector3f, 3> directions = {
      Eigen::Vector3f(0.264882f, 0.688399f, 0.675237f),
      Eigen::Vector3f(0.0145419f, 0.732901f, 0.68018f),
      Eigen::Vector3f(0.856514f, 0.508771f, 0.0868081f)};

  std::vector<std::atomic<bool>> crosses_vec(indices_->size());

  {
    // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(directions, crosses_vec) \
  num_threads(num_threads_)
    // clang-format on
    for (int i = 0; i < static_cast<int>(indices_->size()); ++i) {
      const index_t& index = (*indices_)[i];
      std::atomic<bool>& crosses = crosses_vec[i];

      const PointT& point = (*input_)[index];

      std::array<size_t, 3> crossings;
      std::transform(directions.cbegin(),
                     directions.cend(),
                     crossings.begin(),
                     [this, &point](const auto& direction) {
                       return tree_.numberOfIntersections(point, direction);
                     });

      crosses = (crossings[0] & 1) + (crossings[1] & 1) + (crossings[2] & 1) > 1;
    }
  }

  auto index_itr = indices_->cbegin();
  auto crosses_itr = crosses_vec.cbegin();

  for (; index_itr != indices_->cend(); ++index_itr, ++crosses_itr) {
    const auto& index = *index_itr;
    const auto& crosses = *crosses_itr;

    if ((crop_outside_ && crosses) || (!crop_outside_ && !crosses))
      indices.push_back(index);
    else
      removed_indices_->push_back(index);
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

#define PCL_INSTANTIATE_CropHull(T) template class PCL_EXPORTS pcl::CropHull<T>;

#endif // PCL_FILTERS_IMPL_CROP_HULL_H_
