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
 * $Id$
 *
 */

#ifndef PCL_SEGMENTATION_IMPL_EXTRACT_POLYGONAL_PRISM_DATA_H_
#define PCL_SEGMENTATION_IMPL_EXTRACT_POLYGONAL_PRISM_DATA_H_

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/sample_consensus/sac_model_plane.h> // for SampleConsensusModelPlane
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::isPointIn2DPolygon (const PointT &point, const pcl::PointCloud<PointT> &polygon)
{
  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  computeMeanAndCovarianceMatrix (polygon, covariance_matrix, xyz_centroid);

  // Compute the model coefficients
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  eigen33 (covariance_matrix, eigen_value, eigen_vector);

  model_coefficients[0] = eigen_vector [0];
  model_coefficients[1] = eigen_vector [1];
  model_coefficients[2] = eigen_vector [2];
  model_coefficients[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  float distance_to_plane = model_coefficients[0] * point.x +
                            model_coefficients[1] * point.y +
                            model_coefficients[2] * point.z +
                            model_coefficients[3];
  PointT ppoint;
  // Calculate the projection of the point on the plane
  ppoint.x = point.x - distance_to_plane * model_coefficients[0];
  ppoint.y = point.y - distance_to_plane * model_coefficients[1];
  ppoint.z = point.z - distance_to_plane * model_coefficients[2];

  // Create a X-Y projected representation for within bounds polygonal checking
  int k0, k1, k2;
  // Determine the best plane to project points onto
  k0 = (std::abs (model_coefficients[0] ) > std::abs (model_coefficients[1])) ? 0  : 1;
  k0 = (std::abs (model_coefficients[k0]) > std::abs (model_coefficients[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;
  // Project the convex hull
  pcl::PointCloud<PointT> xy_polygon;
  xy_polygon.resize (polygon.size ());
  for (std::size_t i = 0; i < polygon.size (); ++i)
  {
    Eigen::Vector4f pt (polygon[i].x, polygon[i].y, polygon[i].z, 0);
    xy_polygon[i].x = pt[k1];
    xy_polygon[i].y = pt[k2];
    xy_polygon[i].z = 0;
  }
  PointT xy_point;
  xy_point.z = 0;
  Eigen::Vector4f pt (ppoint.x, ppoint.y, ppoint.z, 0);
  xy_point.x = pt[k1];
  xy_point.y = pt[k2];

  return (pcl::isXYPointIn2DXYPolygon (xy_point, xy_polygon));
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::isXYPointIn2DXYPolygon (const PointT &point, const pcl::PointCloud<PointT> &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  const auto nr_poly_points = polygon.size ();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon[nr_poly_points - 1].x;
  double yold = polygon[nr_poly_points - 1].y;
  for (std::size_t i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon[i].x;
    double ynew = polygon[i].y;
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

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractPolygonalPrismData<PointT>::segment (pcl::PointIndices &output)
{
  output.header = input_->header;

  if (!initCompute ())
  {
    output.indices.clear ();
    return;
  }

  if (static_cast<int> (planar_hull_->size ()) < min_pts_hull_)
  {
    PCL_ERROR("[pcl::%s::segment] Not enough points (%zu) in the hull!\n",
              getClassName().c_str(),
              static_cast<std::size_t>(planar_hull_->size()));
    output.indices.clear ();
    return;
  }

  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  computeMeanAndCovarianceMatrix (*planar_hull_, covariance_matrix, xyz_centroid);

  // Compute the model coefficients
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  eigen33 (covariance_matrix, eigen_value, eigen_vector);

  model_coefficients[0] = eigen_vector [0];
  model_coefficients[1] = eigen_vector [1];
  model_coefficients[2] = eigen_vector [2];
  model_coefficients[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Need to flip the plane normal towards the viewpoint
  Eigen::Vector4f vp (vpx_, vpy_, vpz_, 0);
  // See if we need to flip any plane normals
  vp -= (*planar_hull_)[0].getVector4fMap ();
  vp[3] = 0;
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = vp.dot (model_coefficients);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    model_coefficients *= -1;
    model_coefficients[3] = 0;
    // Hessian form (D = nc . p_plane (centroid here) + p)
    model_coefficients[3] = -1 * (model_coefficients.dot ((*planar_hull_)[0].getVector4fMap ()));
  }

  // Project all points
  PointCloud projected_points;
  SampleConsensusModelPlane<PointT> sacmodel (input_);
  sacmodel.projectPoints (*indices_, model_coefficients, projected_points, false);

  // Create a X-Y projected representation for within bounds polygonal checking
  int k0, k1, k2;
  // Determine the best plane to project points onto
  k0 = (std::abs (model_coefficients[0] ) > std::abs (model_coefficients[1])) ? 0  : 1;
  k0 = (std::abs (model_coefficients[k0]) > std::abs (model_coefficients[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;
  // Project the convex hull
  pcl::PointCloud<PointT> polygon;
  polygon.resize (planar_hull_->size ());
  for (std::size_t i = 0; i < planar_hull_->size (); ++i)
  {
    Eigen::Vector4f pt ((*planar_hull_)[i].x, (*planar_hull_)[i].y, (*planar_hull_)[i].z, 0);
    polygon[i].x = pt[k1];
    polygon[i].y = pt[k2];
    polygon[i].z = 0;
  }

  PointT pt_xy;
  pt_xy.z = 0;

  output.indices.resize (indices_->size ());
  int l = 0;
  for (std::size_t i = 0; i < projected_points.size (); ++i)
  {
    // Check the distance to the user imposed limits from the table planar model
    double distance = pointToPlaneDistanceSigned ((*input_)[(*indices_)[i]], model_coefficients);
    if (distance < height_limit_min_ || distance > height_limit_max_)
      continue;

    // Check what points are inside the hull
    Eigen::Vector4f pt (projected_points[i].x,
                         projected_points[i].y,
                         projected_points[i].z, 0);
    pt_xy.x = pt[k1];
    pt_xy.y = pt[k2];

    if (!pcl::isXYPointIn2DXYPolygon (pt_xy, polygon))
      continue;

    output.indices[l++] = (*indices_)[i];
  }
  output.indices.resize (l);

  deinitCompute ();
}

#define PCL_INSTANTIATE_ExtractPolygonalPrismData(T) template class PCL_EXPORTS pcl::ExtractPolygonalPrismData<T>;
#define PCL_INSTANTIATE_isPointIn2DPolygon(T) template bool PCL_EXPORTS pcl::isPointIn2DPolygon<T>(const T&, const pcl::PointCloud<T> &);
#define PCL_INSTANTIATE_isXYPointIn2DXYPolygon(T) template bool PCL_EXPORTS pcl::isXYPointIn2DXYPolygon<T>(const T &, const pcl::PointCloud<T> &);

#endif    // PCL_SEGMENTATION_IMPL_EXTRACT_POLYGONAL_PRISM_DATA_H_

