/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_QHULL

#ifndef PCL_SURFACE_IMPL_CONVEX_HULL_H_
#define PCL_SURFACE_IMPL_CONVEX_HULL_H_

#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/surface/qhull.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::calculateInputDimension ()
{
  PCL_DEBUG ("[pcl::%s::calculateInputDimension] WARNING: Input dimension not specified.  Automatically determining input dimension.\n", getClassName ().c_str ());
  Eigen::Vector4d xyz_centroid;
  compute3DCentroid (*input_, *indices_, xyz_centroid);
  EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero ();
  computeCovarianceMatrixNormalized (*input_, *indices_, xyz_centroid, covariance_matrix);

  EIGEN_ALIGN16 Eigen::Vector3d eigen_values;
  pcl::eigen33 (covariance_matrix, eigen_values);

  if (std::abs (eigen_values[0]) < std::numeric_limits<double>::epsilon () || std::abs (eigen_values[0] / eigen_values[2]) < 1.0e-3)
    dimension_ = 2;
  else
    dimension_ = 3;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction2D (PointCloud &hull, std::vector<pcl::Vertices> &polygons,
                                                    bool)
{
  int dimension = 2;
  bool xy_proj_safe = true;
  bool yz_proj_safe = true;
  bool xz_proj_safe = true;

  // Check the input's normal to see which projection to use
  PointInT p0 = input_->points[(*indices_)[0]];
  PointInT p1 = input_->points[(*indices_)[indices_->size () - 1]];
  PointInT p2 = input_->points[(*indices_)[indices_->size () / 2]];
  Eigen::Array4f dy1dy2 = (p1.getArray4fMap () - p0.getArray4fMap ()) / (p2.getArray4fMap () - p0.getArray4fMap ());
  while (!( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) ) )
  {
    p0 = input_->points[(*indices_)[rand () % indices_->size ()]];
    p1 = input_->points[(*indices_)[rand () % indices_->size ()]];
    p2 = input_->points[(*indices_)[rand () % indices_->size ()]];
    dy1dy2 = (p1.getArray4fMap () - p0.getArray4fMap ()) / (p2.getArray4fMap () - p0.getArray4fMap ());
  }
    
  pcl::PointCloud<PointInT> normal_calc_cloud;
  normal_calc_cloud.points.resize (3);
  normal_calc_cloud.points[0] = p0;
  normal_calc_cloud.points[1] = p1;
  normal_calc_cloud.points[2] = p2;
    
  Eigen::Vector4d normal_calc_centroid;
  Eigen::Matrix3d normal_calc_covariance;
  pcl::compute3DCentroid (normal_calc_cloud, normal_calc_centroid);
  pcl::computeCovarianceMatrixNormalized (normal_calc_cloud, normal_calc_centroid, normal_calc_covariance);

  // Need to set -1 here. See eigen33 for explanations.
  Eigen::Vector3d::Scalar eigen_value;
  Eigen::Vector3d plane_params;
  pcl::eigen33 (normal_calc_covariance, eigen_value, plane_params);
  float theta_x = fabsf (static_cast<float> (plane_params.dot (x_axis_)));
  float theta_y = fabsf (static_cast<float> (plane_params.dot (y_axis_)));
  float theta_z = fabsf (static_cast<float> (plane_params.dot (z_axis_)));

  // Check for degenerate cases of each projection
  // We must avoid projections in which the plane projects as a line
  if (theta_z > projection_angle_thresh_)
  {
    xz_proj_safe = false;
    yz_proj_safe = false;
  }
  if (theta_x > projection_angle_thresh_)
  {
    xz_proj_safe = false;
    xy_proj_safe = false;
  }
  if (theta_y > projection_angle_thresh_)
  {
    xy_proj_safe = false;
    yz_proj_safe = false;
  }

  // Array of coordinates for each point
  std::vector<double> points (2 * indices_->size ());

  if (!xy_proj_safe && !yz_proj_safe && !xz_proj_safe)
  {
    // This should only happen if we had invalid input
    PCL_ERROR ("[pcl::%s::performReconstruction2D] Invalid input!\n", getClassName ().c_str ());
    return;
  }

  // works because all _proj_safe are mutually exclusive
  const uint8_t c0 = yz_proj_safe;
  const uint8_t c1 = !xy_proj_safe + 1;

  for (size_t i = 0, j = 0; i < indices_->size ();++i)
  {
    points[j++].input_->points[(*indices_)[i]].data[c0];
    points[j++].input_->points[(*indices_)[i]].data[c1];
  }

  orgQhull::Qhull qhull;
  // Compute convex hull
  try
  {
    qhull.runQhull("", dimension, indices_->size (), points.data(), "");

    // 0 if no error from qhull or it doesn't find any vertices
    if (qhull.qhullStatus() != 0 || qhull.vertexCount() == 0)
    {
      throw std::runtime_error(qhull.qhullMessage());
    }
  }
  catch (const std::exception& e)
  {
    PCL_ERROR ("[pcl::%s::performReconstrution2D] ERROR: qhull was unable to compute a convex hull for the given point cloud (%lu)!\nError: %s\n", getClassName ().c_str (), indices_->size (), e.what());

    hull.points.resize (0);
    hull.width = hull.height = 0;
    polygons.resize (0);

    return;
  }

  // Qhull returns the area in volume for 2D
  if (compute_area_)
  {
    total_area_ = qhull.volume();
    total_volume_ = 0.0;
  }

  orgQhull::QhullVertexList vertexs = qhull.vertexList();

  std::vector<std::pair<int, Eigen::Vector4f>, Eigen::aligned_allocator<std::pair<int, Eigen::Vector4f> > > idx_points (vertexs.size ());
  for(orgQhull::QhullVertexList::iterator it = vertexs.begin (); it != vertexs.end (); ++it)
  {
    hull.push_back(input_->points[(*indices_)[it->point().id ()]]);
    idx_points.push_back(std::make_pair (it->point ().id (), Eigen::Vector4f ()));
  }

  // Sort
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (hull, centroid);

  for (size_t j = 0; j < hull.points.size (); ++j)
  {
    idx_points[j].second[0] = hull.points[j][c0] - centroid[c0];
    idx_points[j].second[1] = hull.points[j][c1] - centroid[c1];
  }

  std::sort (idx_points.begin (), idx_points.end (), comparePoints2D);

  polygons.resize (1);
  polygons[0].vertices.resize (hull.points.size ());

  hull_indices_.header = input_->header;
  hull_indices_.indices.clear ();
  hull_indices_.indices.reserve (hull.points.size ());

  for (unsigned int j = 0; j < hull.points.size (); ++j)
  {
    hull_indices_.indices.push_back ((*indices_)[idx_points[j].first]);
    hull.points[j] = input_->points[(*indices_)[idx_points[j].first]];
    polygons[0].vertices[j] = j;
  }

  hull.width = static_cast<uint32_t> (hull.points.size ());
  hull.height = 1;
  hull.is_dense = false;
}

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction3D (
    PointCloud &hull, std::vector<pcl::Vertices> &polygons, bool fill_polygon_data)
{
  int dimension = 3;

  // Array of coordinates for each point
  std::vector<double> points (3 * indices_->size ());

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    points[3*i].push_back(input_->points[(*indices_)[i]].x);
    points[3*i + 1].push_back(input_->points[(*indices_)[i]].y);
    points[3*i + 2].push_back(input_->points[(*indices_)[i]].z);
  }

  orgQhull::Qhull qhull;
  // Compute convex hull
  try {
    qhull.runQhull("", dimension, indices_->size (), points.data(), "");

    // 0 if no error from qhull or it doesn't find any vertices
    if (qhull.qhullStatus() != 0 || qhull.vertexCount() == 0)
    {
      throw std::runtime_error(qhull.qhullMessage());
    }

    qh_triangulate (qhull.qh());
  }
  catch (const std::exception& e)
  {
    PCL_ERROR ("[pcl::%s::performReconstrution3D] ERROR: qhull was unable to compute a convex hull for the given point cloud (%lu)!\nError: %s\n", getClassName ().c_str (), input_->points.size (), e.what());

    hull.points.resize (0);
    hull.width = hull.height = 0;
    polygons.resize (0);
  }

  int num_facets = qhull.facetCount();
  int num_vertices = qhull.vertexCount();
  hull.points.resize (num_vertices);

  // Max vertex id
  unsigned int max_vertex_id = 0;
  orgQhull::QhullVertexList vertexs = qhull.vertexList();
  for(orgQhull::QhullVertexList::iterator it = vertexs.begin (); it != vertexs.end (); ++it)
  {
    if (it->id() + 1 > max_vertex_id)
      max_vertex_id = it->id() + 1;
  }

  std::vector<uint32_t> qhid_to_pcidx (++max_vertex_id);

  hull_indices_.header = input_->header;
  hull_indices_.indices.clear ();
  hull_indices_.indices.reserve (num_vertices);

  uint32_t i = 0;
  for(orgQhull::QhullVertexList::iterator it = vertexs.begin (); it != vertexs.end (); ++it, ++i)
  {
    // Add vertices to hull point_cloud and store index
    hull_indices_.indices.push_back ((*indices_)[it->point().id()]);
    hull.points[i] = input_->points[hull_indices_.indices.back ()];

    qhid_to_pcidx[it->id()] = i; // map the vertex id of qhull to the point cloud index
  }

  if (compute_area_)
  {
    total_area_  = qhull.area();
    total_volume_ = qhull.volume();
  }

  if (fill_polygon_data)
  {
    polygons.resize (num_facets);
    size_t dd = 0;

    orgQhull::QhullFacetList facets = qhull.facetList();
    for(orgQhull::QhullFacetList::iterator it = facets.begin (); it != facets.end (); ++it, ++dd)
    {
      polygons[dd].vertices.resize (3);

      orgQhull::QhullVertexSet vertices = it->vertices();
      size_t vertex_i = 0;
      for(orgQhull::QhullVertexSet::iterator jt = vertices.begin (); jt != vertices.end (); ++jt, ++vertex_i) {
        polygons[dd].vertices[vertex_i] = qhid_to_pcidx[(*jt).id()];
      }
    }
  }

  hull.width = static_cast<uint32_t> (hull.points.size ());
  hull.height = 1;
  hull.is_dense = false;
}
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wold-style-cast"
#endif

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction (PointCloud &hull, std::vector<pcl::Vertices> &polygons,
                                                  bool fill_polygon_data)
{
  if (dimension_ == 0)
    calculateInputDimension ();
  if (dimension_ == 2)
    performReconstruction2D (hull, polygons, fill_polygon_data);
  else if (dimension_ == 3)
    performReconstruction3D (hull, polygons, fill_polygon_data);
  else
    PCL_ERROR ("[pcl::%s::performReconstruction] Error: invalid input dimension requested: %d\n",getClassName ().c_str (),dimension_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::reconstruct (PointCloud &points)
{
  points.header = input_->header;
  if (!initCompute () || input_->points.empty () || indices_->empty ())
  {
    points.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  std::vector<pcl::Vertices> polygons;
  performReconstruction (points, polygons, false);

  points.width = static_cast<uint32_t> (points.points.size ());
  points.height = 1;
  points.is_dense = true;

  deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction (PolygonMesh &output)
{
  // Perform reconstruction
  pcl::PointCloud<PointInT> hull_points;
  performReconstruction (hull_points, output.polygons, true);

  // Convert the PointCloud into a PCLPointCloud2
  pcl::toPCLPointCloud2 (hull_points, output.cloud);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction (std::vector<pcl::Vertices> &polygons)
{
  pcl::PointCloud<PointInT> hull_points;
  performReconstruction (hull_points, polygons, true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::reconstruct (PointCloud &points, std::vector<pcl::Vertices> &polygons)
{
  points.header = input_->header;
  if (!initCompute () || input_->points.empty () || indices_->empty ())
  {
    points.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  performReconstruction (points, polygons, true);

  points.width = static_cast<uint32_t> (points.points.size ());
  points.height = 1;
  points.is_dense = true;

  deinitCompute ();
}
//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::getHullPointIndices (pcl::PointIndices &hull_point_indices) const
{
  hull_point_indices = hull_indices_;
}

#define PCL_INSTANTIATE_ConvexHull(T) template class PCL_EXPORTS pcl::ConvexHull<T>;

#endif    // PCL_SURFACE_IMPL_CONVEX_HULL_H_
#endif
