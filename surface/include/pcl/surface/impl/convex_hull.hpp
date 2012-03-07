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

#include "pcl/surface/convex_hull.h"
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <stdlib.h>

extern "C"
{
#ifdef HAVE_QHULL_2011
#  include "libqhull/libqhull.h"
#  include "libqhull/mem.h"
#  include "libqhull/qset.h"
#  include "libqhull/geom.h"
#  include "libqhull/merge.h"
#  include "libqhull/poly.h"
#  include "libqhull/io.h"
#  include "libqhull/stat.h"
#else
#  include "qhull/qhull.h"
#  include "qhull/mem.h"
#  include "qhull/qset.h"
#  include "qhull/geom.h"
#  include "qhull/merge.h"
#  include "qhull/poly.h"
#  include "qhull/io.h"
#  include "qhull/stat.h"
#endif
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::calculateInputDimension ()
{
  PCL_WARN ("[pcl::%s::calculateInputDimension] WARNING: Input dimension not specified.  Automatically determining input dimension.\n", getClassName ().c_str ());
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;
  computeMeanAndCovarianceMatrix (*input_, *indices_, covariance_matrix, xyz_centroid);

  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  pcl::eigen33 (covariance_matrix, eigen_values);

  if (eigen_values[0] / eigen_values[2] < 1.0e-3)
    dimension_ = 2;
  else
    dimension_ = 3;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction2D (PointCloud &hull, std::vector<pcl::Vertices> &polygons,
                                                    bool fill_polygon_data)
{
  int dimension = 2;
  bool xy_proj_safe = true;
  bool yz_proj_safe = true;
  bool xz_proj_safe = true;

  // Check the input's normal to see which projection to use
  PointInT p0 = input_->points[0];
  PointInT p1 = input_->points[input_->points.size () - 1];
  PointInT p2 = input_->points[input_->points.size () / 2];
  Eigen::Array4f dy1dy2 = (p1.getArray4fMap () - p0.getArray4fMap ()) / (p2.getArray4fMap () - p0.getArray4fMap ());
  while (!( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) ) )
  {
    p0 = input_->points[rand () % input_->points.size ()];
    p1 = input_->points[rand () % input_->points.size ()];
    p2 = input_->points[rand () % input_->points.size ()];
    dy1dy2 = (p1.getArray4fMap () - p0.getArray4fMap ()) / (p2.getArray4fMap () - p0.getArray4fMap ());
  }
    
  pcl::PointCloud<PointInT> normal_calc_cloud;
  normal_calc_cloud.points.resize (3);
  normal_calc_cloud.points[0] = p0;
  normal_calc_cloud.points[1] = p1;
  normal_calc_cloud.points[2] = p2;
    
  Eigen::Vector4f normal_calc_centroid;
  Eigen::Matrix3f normal_calc_covariance;
  pcl::computeMeanAndCovarianceMatrix (normal_calc_cloud, normal_calc_covariance, normal_calc_centroid);
  // Need to set -1 here. See eigen33 for explanations.
  Eigen::Vector3f::Scalar eigen_value;
  Eigen::Vector3f plane_params;
  pcl::eigen33 (normal_calc_covariance, eigen_value, plane_params);
  float theta_x =  fabs (plane_params.dot (x_axis_));
  float theta_y =  fabs (plane_params.dot (y_axis_));
  float theta_z =  fabs (plane_params.dot (z_axis_));

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

  // True if qhull should free points in qh_freeqhull() or reallocation
  boolT ismalloc = True;
  // output from qh_produce_output(), use NULL to skip qh_produce_output()
  FILE *outfile = NULL;

  if (compute_area_)
    outfile = stderr;

  // option flags for qhull, see qh_opt.htm
  char * flags = (char *)qhull_flags.c_str ();
  // error messages from qhull code
  FILE *errfile = stderr;

  // Array of coordinates for each point
  coordT *points = (coordT *)calloc (input_->points.size () * dimension, sizeof(coordT));

  // Build input data, using appropriate projection
  int j = 0;
  if (xy_proj_safe)
  {
    for (size_t i = 0; i < input_->points.size (); ++i, j+=dimension)
    {
      points[j + 0] = (coordT)input_->points[i].x;
      points[j + 1] = (coordT)input_->points[i].y;
    }
  } 
  else if (yz_proj_safe)
  {
    for (size_t i = 0; i < input_->points.size (); ++i, j+=dimension)
    {
      points[j + 0] = (coordT)input_->points[i].y;
      points[j + 1] = (coordT)input_->points[i].z;
    }
  }
  else if (xz_proj_safe)
  {
    for (size_t i = 0; i < input_->points.size (); ++i, j+=dimension)
    {
      points[j + 0] = (coordT)input_->points[i].x;
      points[j + 1] = (coordT)input_->points[i].z;
    }
  }
  else
  {
    // This should only happen if we had invalid input
    PCL_ERROR ("[pcl::%s::performReconstruction2D] Invalid input!\n", getClassName ().c_str ());
  }
   
  // Compute convex hull
  int exitcode = qh_new_qhull (dimension, input_->points.size (), points, ismalloc, flags, outfile, errfile);
    
  // 0 if no error from qhull
  if (exitcode != 0)
  {
    PCL_ERROR ("[pcl::%s::performReconstrution2D] ERROR: qhull was unable to compute a convex hull for the given point cloud (%lu)!\n", getClassName ().c_str (), (unsigned long) input_->points.size ());

    hull.points.resize (0);
    hull.width = hull.height = 0;
    polygons.resize (0);

    qh_freeqhull (!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);

    return;
  }

  // Qhull returns the area in volume for 2D
  if (compute_area_)
  {
    total_area_ = qh totvol;
    total_volume_ = 0.0;
  }

  int num_vertices = qh num_vertices;
  hull.points.resize (num_vertices);
  memset (&hull.points[0], hull.points.size (), sizeof (PointInT));

  vertexT * vertex;
  int i = 0;

  std::vector<std::pair<int, Eigen::Vector4f>, Eigen::aligned_allocator<std::pair<int, Eigen::Vector4f> > > idx_points (num_vertices);
  idx_points.resize (hull.points.size ());
  memset (&idx_points[0], hull.points.size (), sizeof (std::pair<int, Eigen::Vector4f>));

  FORALLvertices
  {
    hull.points[i] = input_->points[qh_pointid (vertex->point)];
    idx_points[i].first = qh_pointid (vertex->point);
    ++i;
  }

  // Sort
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (hull, centroid);
  if (xy_proj_safe)
  {
    for (size_t j = 0; j < hull.points.size (); j++)
    {
      idx_points[j].second[0] = hull.points[j].x - centroid[0];
      idx_points[j].second[1] = hull.points[j].y - centroid[1];
    }
  }
  else if (yz_proj_safe)
  {
    for (size_t j = 0; j < hull.points.size (); j++)
    {
      idx_points[j].second[0] = hull.points[j].y - centroid[0];
      idx_points[j].second[1] = hull.points[j].z - centroid[1];
    }
  }
  else if (xz_proj_safe)
  {
    for (size_t j = 0; j < hull.points.size (); j++)
    {
      idx_points[j].second[0] = hull.points[j].x - centroid[0];
      idx_points[j].second[1] = hull.points[j].z - centroid[1];
    }
  }
  std::sort (idx_points.begin (), idx_points.end (), comparePoints2D);
    
  polygons.resize (1);
  polygons[0].vertices.resize (hull.points.size () + 1);  

  for (int j = 0; j < static_cast<int> (hull.points.size ()); j++)
  {
    hull.points[j] = input_->points[idx_points[j].first];
    polygons[0].vertices[j] = j;
  }
  polygons[0].vertices[hull.points.size ()] = 0;
    
  qh_freeqhull (!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort (&curlong, &totlong);

  hull.width = hull.points.size ();
  hull.height = 1;
  hull.is_dense = false;
  return;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction3D (
    PointCloud &hull, std::vector<pcl::Vertices> &polygons, bool fill_polygon_data)
{
  int dimension = 3;

  // True if qhull should free points in qh_freeqhull() or reallocation
  boolT ismalloc = True;
  // output from qh_produce_output(), use NULL to skip qh_produce_output()
  FILE *outfile = NULL;

  if (compute_area_)
    outfile = stderr;

  // option flags for qhull, see qh_opt.htm
  char * flags = (char *)qhull_flags.c_str ();
  // error messages from qhull code
  FILE *errfile = stderr;

  // Array of coordinates for each point
  coordT *points = (coordT *)calloc (input_->points.size () * dimension, sizeof (coordT));

  int j = 0;
  for (size_t i = 0; i < input_->points.size (); ++i, j+=dimension)
  {
    points[j + 0] = (coordT)input_->points[i].x;
    points[j + 1] = (coordT)input_->points[i].y;
    points[j + 2] = (coordT)input_->points[i].z;
  }

  // Compute convex hull
  int exitcode = qh_new_qhull (dimension, input_->points.size (), points, ismalloc, flags, outfile, errfile);

  // 0 if no error from qhull
  if (exitcode != 0)
  {
    PCL_ERROR ("[pcl::%s::performReconstrution3D] ERROR: qhull was unable to compute a convex hull for the given point cloud (%lu)!\n", getClassName ().c_str (), (unsigned long) input_->points.size ());

    //check if it fails because of NaN values...
    if (!input_->is_dense)
      PCL_ERROR ("[pcl::%s::performReconstruction3D] ERROR: point cloud contains NaN values, consider running pcl::PassThrough filter first to remove NaNs!\n", getClassName ().c_str ());

    hull.points.resize (0);
    hull.width = hull.height = 0;
    polygons.resize (0);

    qh_freeqhull (!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);

    return;
  }

  qh_triangulate ();

  int num_facets = qh num_facets;

  int num_vertices = qh num_vertices;
  hull.points.resize (num_vertices);

  vertexT * vertex;
  int i = 0;
  // Max vertex id
  int max_vertex_id = -1;
  FORALLvertices
  {
    if ((int)vertex->id > max_vertex_id)
      max_vertex_id = vertex->id;
  }

  ++max_vertex_id;
  std::vector<int> qhid_to_pcidx (max_vertex_id);

  FORALLvertices
  {
    // Add vertices to hull point_cloud
    hull.points[i] = input_->points[qh_pointid (vertex->point)];

    qhid_to_pcidx[vertex->id] = i; // map the vertex id of qhull to the point cloud index
    ++i;
  }

  if (compute_area_)
  {
    total_area_  = qh totarea;
    total_volume_ = qh totvol;
  }

  if (fill_polygon_data)
  {
    polygons.resize (num_facets);
    int dd = 0;

    facetT * facet;
    FORALLfacets
    {
      polygons[dd].vertices.resize (3);

      // Needed by FOREACHvertex_i_
      int vertex_n, vertex_i;
      FOREACHvertex_i_ ((*facet).vertices)
      //facet_vertices.vertices.push_back (qhid_to_pcidx[vertex->id]);
      polygons[dd].vertices[vertex_i] = qhid_to_pcidx[vertex->id];
      ++dd;
    }
  }
  // Deallocates memory (also the points)
  qh_freeqhull (!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort (&curlong, &totlong);

  hull.width = hull.points.size ();
  hull.height = 1;
  hull.is_dense = false;
}

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
pcl::ConvexHull<PointInT>::reconstruct (PointCloud &output)
{
  output.header = input_->header;
  if (!initCompute () || input_->points.empty ())
  {
    output.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  std::vector<pcl::Vertices> polygons;
  performReconstruction (output, polygons, false);

  output.width = output.points.size ();
  output.height = 1;
  output.is_dense = true;

  deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull<PointInT>::performReconstruction (PolygonMesh &output)
{
  // Perform reconstruction
  pcl::PointCloud<PointInT> hull_points;
  performReconstruction (hull_points, output.polygons, true);

  // Convert the PointCloud into a PointCloud2
  pcl::toROSMsg (hull_points, output.cloud);
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
  if (!initCompute () || input_->points.empty ())
  {
    points.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  performReconstruction (points, polygons, true);

  points.width = points.points.size ();
  points.height = 1;
  points.is_dense = true;

  deinitCompute ();
}

#define PCL_INSTANTIATE_ConvexHull(T) template class PCL_EXPORTS pcl::ConvexHull<T>;

#endif    // PCL_SURFACE_IMPL_CONVEX_HULL_H_
#endif
