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
 * $Id: convex_hull.hpp 36101 2011-02-21 19:39:30Z aaldoma $
 *
 */

#ifndef PCL_SURFACE_IMPL_CONCAVE_HULL_H_
#define PCL_SURFACE_IMPL_CONCAVE_HULL_H_

#include "pcl/surface/concave_hull.h"
#include "pcl/common/common.h"
#include "pcl/registration/transforms.h"

//////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::ConcaveHull<PointInT>::reconstruct (PointCloud &output)
{
  output.header = input_->header;
  if (!initCompute ())
  {
    output.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  std::vector<pcl::Vertices> polygons;
  performReconstruction (output, polygons);

  output.width = output.points.size ();
  output.height = 1;
  output.is_dense = true;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::ConcaveHull<PointInT>::reconstruct (PointCloud &points, 
                                         std::vector<pcl::Vertices> &polygons)
{
  points.header = input_->header;
  if (!initCompute ())
  {
    points.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  performReconstruction (points, polygons);

  points.width = points.points.size ();
  points.height = 1;
  points.is_dense = true;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////
template<typename PointInT>
void
pcl::ConcaveHull<PointInT>::performReconstruction (PointCloud &alpha_shape, 
                                                   std::vector<pcl::Vertices> &polygons)
{
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;
  compute3DCentroid (*input_, *indices_, xyz_centroid);
  computeCovarianceMatrix (*input_, *indices_, xyz_centroid, covariance_matrix);
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  Eigen::Affine3f transform1;
  transform1.setIdentity();
  int dim = 3;
  if (eigen_values[0] / eigen_values[2] < 1.0e-5)
  {
   //we have points laying on a plane, using 2d convex hull
   //compute transformation bring eigen_vectors.col(i) to z-axis

   eigen_vectors.col (2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
   eigen_vectors.col (1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

   transform1 (0, 2) = eigen_vectors (0, 0);
   transform1 (1, 2) = eigen_vectors (1, 0);
   transform1 (2, 2) = eigen_vectors (2, 0);

   transform1 (0, 1) = eigen_vectors (0, 1);
   transform1 (1, 1) = eigen_vectors (1, 1);
   transform1 (2, 1) = eigen_vectors (2, 1);
   transform1 (0, 0) = eigen_vectors (0, 2);
   transform1 (1, 0) = eigen_vectors (1, 2);
   transform1 (2, 0) = eigen_vectors (2, 2);


   transform1 = transform1.inverse ();
   dim = 2;
  }
  else
  {
   transform1.setIdentity ();
  }

  // For 3D point clouds, alpha shapes is not yet implemented!
  if (dim == 3) 
  {
    ROS_WARN ("Alpha shapes for 3-dimensional point clouds not yet implemented..., returning empty PolygonMesh and alpha_shape_points");
    alpha_shape.points.resize (0);
    alpha_shape.width = alpha_shape.height = 0;
    polygons.resize (0);
    return;
  }

  PointCloud cloud_transformed;
  pcl::demeanPointCloud(*input_, *indices_,xyz_centroid,cloud_transformed);
  pcl::transformPointCloud (cloud_transformed, cloud_transformed, transform1);

  // True if qhull should free points in qh_freeqhull() or reallocation
  boolT ismalloc = True; 
  // option flags for qhull, see qh_opt.htm
  char flags[] = "qhull d QJ"; 
  // output from qh_produce_output(), use NULL to skip qh_produce_output()
  FILE *outfile = NULL; 
  // error messages from qhull code
  FILE *errfile = stderr;
  // 0 if no error from qhull
  int exitcode; 

  // Array of coordinates for each point
  coordT *points = (coordT *)calloc (cloud_transformed.points.size () * dim, sizeof (coordT));

  for (size_t i = 0; i < cloud_transformed.points.size (); ++i)
  {
    points[i * dim + 0] = (coordT)cloud_transformed.points[i].x;
    points[i * dim + 1] = (coordT)cloud_transformed.points[i].y;

    if (dim > 2)
      points[i * dim + 2] = (coordT)cloud_transformed.points[i].z;
  }

  // Compute concave hull
  exitcode = qh_new_qhull (dim, cloud_transformed.points.size (), points, ismalloc, flags, 
                           outfile, errfile);
  qh_setvoronoi_all ();

  int num_vertices = qh num_vertices;
  alpha_shape.points.resize (num_vertices);

  vertexT *vertex;
  // Max vertex id
  int max_vertex_id=-1;
  FORALLvertices
  {
    if (vertex->id > max_vertex_id)
      max_vertex_id = vertex->id;
  }

  facetT *facet; // set by FORALLfacets
  int non_upperdelaunay=0;
  FORALLfacets
  {
    if (!facet->upperdelaunay)
      non_upperdelaunay++;
  }

  ++max_vertex_id;
  std::vector<int> qhid_to_pcidx (max_vertex_id);

  int num_facets = qh num_facets;
  int dd = 0;
  // Compute the alpha complex for the set of points
  // Filters the delaunay triangles
  setT* edges_set = qh_settemp (3 * num_facets);
  voronoi_centers_.points.resize (num_facets);
  FORALLfacets
  { 
    // Facets are the delaunay triangles (2d) or tetrahedrons (3d)
    if (!facet->upperdelaunay) 
    {
      // Check if the distance from any vertex to the facet->center 
      // (center of the voronoi cell) is smaller than alpha
      vertexT* anyVertex = (vertexT *)(facet->vertices->e[0].p);
      double r = computeDistVertexCenter2D (anyVertex, facet->center);
      if (r < alpha_) 
      {
        pcl::Vertices facet_vertices;
        qh_makeridges (facet);
        facet->good = true;

        ridgeT *ridge, **ridgep;
        FOREACHridge_(facet->ridges) 
          qh_setappend (&edges_set, ridge);

        voronoi_centers_.points[dd].x = facet->center[0];
        voronoi_centers_.points[dd].y = facet->center[1];
        voronoi_centers_.points[dd].z = 0;
        ++dd;
      } 
      else 
        facet->good = false;
    }
  }

  int vertices = 0;
  std::vector<bool> added_vertices (max_vertex_id, false);
  ridgeT *ridge, **ridgep;
  FOREACHridge_(edges_set) 
  {
    if (ridge->bottom->upperdelaunay || ridge->top->upperdelaunay || !ridge->top->good || !ridge->bottom->good) 
    {
      int vertex_n, vertex_i;
      FOREACHvertex_i_((*ridge).vertices)
      {
        if (!added_vertices[vertex->id]) 
        {
          alpha_shape.points[vertices].x = vertex->point[0];
          alpha_shape.points[vertices].y = vertex->point[1];

          if (dim > 2)
            alpha_shape.points[vertices].z = vertex->point[2];
          else
            alpha_shape.points[vertices].z = 0;

          qhid_to_pcidx[vertex->id] = vertices; //map the vertex id of qhull to the point cloud index
          added_vertices[vertex->id] = true;
          vertices++;
        }
      }
    }
  }

  alpha_shape.points.resize (vertices);
  polygons.resize (dd);
  voronoi_centers_.points.resize (dd);

  if (dim == 2)
  {
    Eigen::Affine3f transInverse = transform1.inverse ();
    pcl::transformPointCloud (alpha_shape, alpha_shape, transInverse);
    xyz_centroid[0] = -xyz_centroid[0];
    xyz_centroid[1] = -xyz_centroid[1];
    xyz_centroid[2] = -xyz_centroid[2];
    pcl::demeanPointCloud(alpha_shape,xyz_centroid,alpha_shape);
  }
}

#define PCL_INSTANTIATE_ConcaveHull(T) template class pcl::ConcaveHull<T>;

#endif    // PCL_SURFACE_IMPL_CONCAVE_HULL_H_
