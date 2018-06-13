/**
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_SURFACE_IMPL_CONCAVE_HULL_H_
#define PCL_SURFACE_IMPL_CONCAVE_HULL_H_

#include <map>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/surface/qhull.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::reconstruct (PointCloud &output)
{
  output.header = input_->header;
  if (alpha_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::reconstruct] Alpha parameter must be set to a positive number!\n", getClassName ().c_str ());
    output.points.clear ();
    return;
  }

  if (!initCompute ())
  {
    output.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  std::vector<pcl::Vertices> polygons;
  performReconstruction (output, polygons);

  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1;
  output.is_dense = true;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::reconstruct (PointCloud &output, std::vector<pcl::Vertices> &polygons)
{
  output.header = input_->header;
  if (alpha_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::reconstruct] Alpha parameter must be set to a positive number!\n", getClassName ().c_str ());
    output.points.clear ();
    return;
  }

  if (!initCompute ())
  {
    output.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  performReconstruction (output, polygons);

  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1;
  output.is_dense = true;

  deinitCompute ();
}

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::performReconstruction (PointCloud &alpha_shape, std::vector<pcl::Vertices> &polygons)
{
  Eigen::Vector4d xyz_centroid;
  compute3DCentroid (*input_, *indices_, xyz_centroid);
  EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero ();
  computeCovarianceMatrixNormalized (*input_, *indices_, xyz_centroid, covariance_matrix);

  // Check if the covariance matrix is finite or not.
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
          return;

  EIGEN_ALIGN16 Eigen::Vector3d eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3d eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  Eigen::Affine3d transform1;
  transform1.setIdentity ();

  // If no input dimension is specified, determine automatically
  if (dim_ == 0)
  {
    PCL_DEBUG ("[pcl::%s] WARNING: Input dimension not specified.  Automatically determining input dimension.\n", getClassName ().c_str ());
    if (std::abs (eigen_values[0]) < std::numeric_limits<double>::epsilon () || std::abs (eigen_values[0] / eigen_values[2]) < 1.0e-3)
      dim_ = 2;
    else
      dim_ = 3;
  } 

  if (dim_ == 2)
  {
    // we have points laying on a plane, using 2d convex hull
    // compute transformation bring eigen_vectors.col(i) to z-axis

    transform1 (2, 0) = eigen_vectors (0, 0);
    transform1 (2, 1) = eigen_vectors (1, 0);
    transform1 (2, 2) = eigen_vectors (2, 0);

    transform1 (1, 0) = eigen_vectors (0, 1);
    transform1 (1, 1) = eigen_vectors (1, 1);
    transform1 (1, 2) = eigen_vectors (2, 1);
    transform1 (0, 0) = eigen_vectors (0, 2);
    transform1 (0, 1) = eigen_vectors (1, 2);
    transform1 (0, 2) = eigen_vectors (2, 2);
  }
  else
  {
    transform1.setIdentity ();
  }
  
  PointCloud cloud_transformed;
  pcl::demeanPointCloud (*input_, *indices_, xyz_centroid, cloud_transformed);
  pcl::transformPointCloud (cloud_transformed, cloud_transformed, transform1);

  // option flags for qhull, see qh_opt.htm
  std::string flags = "d QJ";

  // Array of coordinates for each point
  std::vector<double> points (cloud_transformed.points.size () * dim_);

  for (size_t i = 0; i < cloud_transformed.points.size (); ++i)
  {
    points[i * dim_ + 0] = cloud_transformed.points[i].x;
    points[i * dim_ + 1] = cloud_transformed.points[i].y;

    if (dim_ > 2)
      points[i * dim_ + 2] = cloud_transformed.points[i].z;
  }

  // Compute concave hull
  try {
    orgQhull::Qhull qhull;
    qhull.runQhull("", dim_, cloud_transformed.size (), points.data(), flags.c_str());
  
    if (qhull.qhullStatus() != 0)
    {
      throw std::runtime_error(qhull.qhullMessage());
    }
  
    qh_setvoronoi_all(qhull.qh());
  
    int num_vertices = qhull.vertexCount();
    alpha_shape.points.resize (num_vertices);
  
    // Max vertex id
    int max_vertex_id = 0;
    orgQhull::QhullVertexList vertexs = qhull.vertexList();
    for(orgQhull::QhullVertexList::iterator it = vertexs.begin();it!=vertexs.end();it++)
    {
      if (it->id() + 1 > max_vertex_id)
        max_vertex_id = it->id() + 1;
    }
  
    ++max_vertex_id;
    std::vector<int> qhid_to_pcidx (max_vertex_id);
  
    int num_facets = qhull.facetCount();
    int dd = 0;
  
    orgQhull::QhullFacetList facets = qhull.facetList();
  
    if (dim_ == 3)
    {
      std::vector<orgQhull::QhullRidge> triangles_set;
      if (voronoi_centers_)
        voronoi_centers_->points.resize (num_facets);
  
      int non_upper = 0;
      for(orgQhull::QhullFacetList::iterator it = facets.begin();it!=facets.end();it++)
      {
        // Facets are tetrahedrons (3d)
        if (!it->isUpperDelaunay())
        {
          orgQhull::QhullPoint anyVertex = it->vertices().first().point();
          orgQhull::QhullPoint center = it->getCenter();
          anyVertex.setDimension(center.dimension());
          double r = center.distance(anyVertex);
  
          if (voronoi_centers_)
          {
            voronoi_centers_->points[non_upper].x = static_cast<float> (center[0]);
            voronoi_centers_->points[non_upper].y = static_cast<float> (center[1]);
            voronoi_centers_->points[non_upper].z = static_cast<float> (center[2]);
          }
  
          non_upper++;
  
          qh_makeridges(qhull.qh(), it->getFacetT());
          orgQhull::QhullRidgeSet ridges = it->ridges();
  
          if (r <= alpha_)
          {
            // all triangles in tetrahedron are good, add them all to the alpha shape (triangles_set)
            it->getFacetT()->good = true;
            it->getFacetT()->visitid = qhull.qh()->visit_id;
            for(orgQhull::QhullRidgeSet::iterator rt = ridges.begin(); rt != ridges.end(); rt++)
            {
              orgQhull::QhullRidge ridge = *rt;
              orgQhull::QhullFacet neighb = ridge.otherFacet(*it);
              if (neighb.getFacetT()->visitid != qhull.qh()->visit_id)
                triangles_set.push_back(ridge);
            }
          }
          else
          {
            // consider individual triangles from the tetrahedron...
            it->getFacetT()->good = false;
            it->getFacetT()->visitid = qhull.qh()->visit_id;
            for(orgQhull::QhullRidgeSet::iterator rt = ridges.begin(); rt != ridges.end(); rt++)
            {
              orgQhull::QhullRidge ridge = *rt;
              orgQhull::QhullFacet neighb = ridge.otherFacet(*it);
              if (neighb.getFacetT()->visitid != qhull.qh()->visit_id)
              {
                // check if individual triangle is good and add it to triangles_set
                orgQhull::QhullVertexSet ridgeVertices = ridge.vertices();
  
                PointInT a, b, c;
                a.x = static_cast<float> (ridgeVertices[0].point()[0]);
                a.y = static_cast<float> (ridgeVertices[0].point()[1]);
                a.z = static_cast<float> (ridgeVertices[0].point()[2]);
                b.x = static_cast<float> (ridgeVertices[1].point()[0]);
                b.y = static_cast<float> (ridgeVertices[1].point()[1]);
                b.z = static_cast<float> (ridgeVertices[1].point()[2]);
                c.x = static_cast<float> (ridgeVertices[2].point()[0]);
                c.y = static_cast<float> (ridgeVertices[2].point()[1]);
                c.z = static_cast<float> (ridgeVertices[2].point()[2]);
  
                double r = pcl::getCircumcircleRadius (a, b, c);
                if (r <= alpha_)
                  triangles_set.push_back(ridge);
              }
            }
          }
        }
      }
  
      if (voronoi_centers_)
        voronoi_centers_->points.resize (non_upper);
  
      // filter, add points to alpha_shape and create polygon structure
  
      int num_good_triangles = 0;
      for(std::vector<orgQhull::QhullRidge>::iterator it = triangles_set.begin();it!=triangles_set.end();it++)
      {
        orgQhull::QhullRidge& ridge = *it;
        if (ridge.bottomFacet().isUpperDelaunay() || ridge.topFacet().isUpperDelaunay() || !ridge.topFacet().isGood() || !ridge.bottomFacet().isGood())
          num_good_triangles++;
      }
  
      polygons.resize (num_good_triangles);
  
      int vertices = 0;
      std::vector<bool> added_vertices (max_vertex_id, false);
  
      int triangles = 0;
      for(std::vector<orgQhull::QhullRidge>::iterator it = triangles_set.begin();it!=triangles_set.end();it++)
      {
        orgQhull::QhullRidge& ridge = *it;
        if (ridge.bottomFacet().isUpperDelaunay() || ridge.topFacet().isUpperDelaunay() || !ridge.topFacet().isGood() || !ridge.bottomFacet().isGood())
        {
          polygons[triangles].vertices.resize (3);
          int vertex_n, vertex_i;
          orgQhull::QhullVertexSet ridgeVertices = ridge.vertices();
          for(orgQhull::QhullVertexSet::iterator jt = ridgeVertices.begin();jt!=ridgeVertices.end();jt++)  //3 vertices per ridge!
          {
            orgQhull::QhullVertex vertex = *jt;
            if (!added_vertices[vertex.id()])
            {
              orgQhull::QhullPoint point = vertex.point();
              alpha_shape.points[vertices].x = static_cast<float> (point[0]);
              alpha_shape.points[vertices].y = static_cast<float> (point[1]);
              alpha_shape.points[vertices].z = static_cast<float> (point[2]);
  
              qhid_to_pcidx[vertex.id()] = vertices;   //map the vertex id of qhull to the point cloud index
              added_vertices[vertex.id()] = true;
              vertices++;
            }
  
            polygons[triangles].vertices[vertex_i] = qhid_to_pcidx[vertex.id()];
  
          }
  
          triangles++;
        }
      }
  
      alpha_shape.points.resize (vertices);
      alpha_shape.width = static_cast<uint32_t> (alpha_shape.points.size ());
      alpha_shape.height = 1;
    }
    else
    {
      // Compute the alpha complex for the set of points
      // Filters the delaunay triangles
      std::vector<orgQhull::QhullRidge> edges_set;
      if (voronoi_centers_)
        voronoi_centers_->points.resize (num_facets);
  
      for(orgQhull::QhullFacetList::iterator it = facets.begin();it!=facets.end();it++)
      {
        // Facets are the delaunay triangles (2d)
        if (!it->isUpperDelaunay())
        {
          orgQhull::QhullVertexSet vertices = it->vertices();
  
          // Check if the distance from any vertex to the facet->center
          // (center of the voronoi cell) is smaller than alpha
          orgQhull::QhullPoint anyVertex = it->vertices().first().point();
          orgQhull::QhullPoint center = it->getCenter();
          anyVertex.setDimension(center.dimension());
          double r = center.distance(anyVertex);
  
          if (r <= alpha_)
          {
            qh_makeridges(qhull.qh(), it->getFacetT());
            orgQhull::QhullRidgeSet ridges = it->ridges();
            for(orgQhull::QhullRidgeSet::iterator rt = ridges.begin(); rt != ridges.end(); rt++) {
              edges_set.push_back(*rt);
            }
  
            if (voronoi_centers_)
            {
              voronoi_centers_->points[dd].x = static_cast<float> (center[0]);
              voronoi_centers_->points[dd].y = static_cast<float> (center[1]);
              voronoi_centers_->points[dd].z = 0.0f;
            }
  
            ++dd;
          }
          else
            it->getFacetT()->good = false;
        }
      }
  
      int vertices = 0;
      std::vector<bool> added_vertices (max_vertex_id, false);
      std::map<int, std::vector<int> > edges;
  
      for(std::vector<orgQhull::QhullRidge>::iterator rt = edges_set.begin(); rt != edges_set.end(); rt++)
      {
        orgQhull::QhullRidge& ridge = *rt;
        if (ridge.bottomFacet().isUpperDelaunay() || ridge.topFacet().isUpperDelaunay() || !ridge.topFacet().isGood() || !ridge.bottomFacet().isGood())
        {
          orgQhull::QhullVertexSet ridgeVertices = ridge.vertices();
          int vertices_in_ridge=0;
          std::vector<int> pcd_indices;
          pcd_indices.resize (2);
  
          for(orgQhull::QhullVertexSet::iterator jt = ridgeVertices.begin();jt!=ridgeVertices.end();jt++)  //in 2-dim, 2 vertices per ridge!
          {
            orgQhull::QhullVertex vertex = *jt;
            if (!added_vertices[vertex.id()])
            {
              orgQhull::QhullPoint point = vertex.point();
              alpha_shape.points[vertices].x = static_cast<float> (point[0]);
              alpha_shape.points[vertices].y = static_cast<float> (point[1]);
  
              if (dim_ > 2)
                alpha_shape.points[vertices].z = static_cast<float> (point[2]);
              else
                alpha_shape.points[vertices].z = 0;
  
              qhid_to_pcidx[vertex.id()] = vertices;   //map the vertex id of qhull to the point cloud index
              added_vertices[vertex.id()] = true;
              pcd_indices[vertices_in_ridge] = vertices;
              vertices++;
            }
            else
            {
              pcd_indices[vertices_in_ridge] = qhid_to_pcidx[vertex.id()];
            }
  
            vertices_in_ridge++;
          }
  
          // make edges bidirectional and pointing to alpha_shape pointcloud...
          edges[pcd_indices[0]].push_back (pcd_indices[1]);
          edges[pcd_indices[1]].push_back (pcd_indices[0]);
        }
      }
  
      alpha_shape.points.resize (vertices);
  
      std::vector<std::vector<int> > connected;
      PointCloud alpha_shape_sorted;
      alpha_shape_sorted.points.resize (vertices);
  
      // iterate over edges until they are empty!
      std::map<int, std::vector<int> >::iterator curr = edges.begin ();
      int next = - 1;
      std::vector<bool> used (vertices, false);   // used to decide which direction should we take!
      std::vector<int> pcd_idx_start_polygons;
      pcd_idx_start_polygons.push_back (0);
  
      // start following edges and removing elements
      int sorted_idx = 0;
      while (!edges.empty ())
      {
        alpha_shape_sorted.points[sorted_idx] = alpha_shape.points[(*curr).first];
        // check where we can go from (*curr).first
        for (size_t i = 0; i < (*curr).second.size (); i++)
        {
          if (!used[((*curr).second)[i]])
          {
            // we can go there
            next = ((*curr).second)[i];
            break;
          }
        }
  
        used[(*curr).first] = true;
        edges.erase (curr);   // remove edges starting from curr
  
        sorted_idx++;
  
        if (edges.empty ())
          break;
  
        // reassign current
        curr = edges.find (next);   // if next is not found, then we have unconnected polygons.
        if (curr == edges.end ())
        {
          // set current to any of the remaining in edge!
          curr = edges.begin ();
          pcd_idx_start_polygons.push_back (sorted_idx);
        }
      }
  
      pcd_idx_start_polygons.push_back (sorted_idx);
  
      alpha_shape.points = alpha_shape_sorted.points;
  
      polygons.reserve (pcd_idx_start_polygons.size () - 1);
  
      for (size_t poly_id = 0; poly_id < pcd_idx_start_polygons.size () - 1; poly_id++)
      {
        // Check if we actually have a polygon, and not some degenerated output from QHull
        if (pcd_idx_start_polygons[poly_id + 1] - pcd_idx_start_polygons[poly_id] >= 3)
        {
          pcl::Vertices vertices;
          vertices.vertices.resize (pcd_idx_start_polygons[poly_id + 1] - pcd_idx_start_polygons[poly_id]);
          // populate points in the corresponding polygon
          for (int j = pcd_idx_start_polygons[poly_id]; j < pcd_idx_start_polygons[poly_id + 1]; ++j)
            vertices.vertices[j - pcd_idx_start_polygons[poly_id]] = static_cast<uint32_t> (j);
  
          polygons.push_back (vertices);
        }
      }
  
      if (voronoi_centers_)
        voronoi_centers_->points.resize (dd);
    }
  
    Eigen::Affine3d transInverse = transform1.inverse ();
    pcl::transformPointCloud (alpha_shape, alpha_shape, transInverse);
    xyz_centroid[0] = - xyz_centroid[0];
    xyz_centroid[1] = - xyz_centroid[1];
    xyz_centroid[2] = - xyz_centroid[2];
    pcl::demeanPointCloud (alpha_shape, xyz_centroid, alpha_shape);
  
    // also transform voronoi_centers_...
    if (voronoi_centers_)
    {
      pcl::transformPointCloud (*voronoi_centers_, *voronoi_centers_, transInverse);
      pcl::demeanPointCloud (*voronoi_centers_, xyz_centroid, *voronoi_centers_);
    }
  
    if (keep_information_)
    {
      // build a tree with the original points
      pcl::KdTreeFLANN<PointInT> tree (true);
      tree.setInputCloud (input_, indices_);
  
      std::vector<int> neighbor;
      std::vector<float> distances;
      neighbor.resize (1);
      distances.resize (1);
  
      // for each point in the concave hull, search for the nearest neighbor in the original point cloud
      hull_indices_.header = input_->header;
      hull_indices_.indices.clear ();
      hull_indices_.indices.reserve (alpha_shape.points.size ());
  
      for (size_t i = 0; i < alpha_shape.points.size (); i++)
      {
        tree.nearestKSearch (alpha_shape.points[i], 1, neighbor, distances);
        hull_indices_.indices.push_back (neighbor[0]);
      }
  
      // replace point with the closest neighbor in the original point cloud
      pcl::copyPointCloud (*input_, hull_indices_.indices, alpha_shape);
    }
  }
  catch (const std::exception& e)
  {
    PCL_ERROR ("[pcl::%s::performReconstrution] ERROR: qhull was unable to compute a concave hull for the given point cloud (%lu)!\nError: %s\n", getClassName ().c_str (), cloud_transformed.points.size (), e.what());
  
    //check if it fails because of NaN values...
    if (!cloud_transformed.is_dense)
    {
      bool NaNvalues = false;
      for (size_t i = 0; i < cloud_transformed.size (); ++i)
      {
        if (!pcl_isfinite (cloud_transformed.points[i].x) ||
            !pcl_isfinite (cloud_transformed.points[i].y) ||
            !pcl_isfinite (cloud_transformed.points[i].z))
        {
          NaNvalues = true;
          break;
        }
      }
    
      if (NaNvalues)
        PCL_ERROR ("[pcl::%s::performReconstruction] ERROR: point cloud contains NaN values, consider running pcl::PassThrough filter first to remove NaNs!\n", getClassName ().c_str ());
    }
    
    alpha_shape.points.resize (0);
    alpha_shape.width = alpha_shape.height = 0;
    polygons.resize (0);
  }
}
#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wold-style-cast"
#endif

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::performReconstruction (PolygonMesh &output)
{
  // Perform reconstruction
  pcl::PointCloud<PointInT> hull_points;
  performReconstruction (hull_points, output.polygons);

  // Convert the PointCloud into a PCLPointCloud2
  pcl::toPCLPointCloud2 (hull_points, output.cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::performReconstruction (std::vector<pcl::Vertices> &polygons)
{
  pcl::PointCloud<PointInT> hull_points;
  performReconstruction (hull_points, polygons);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConcaveHull<PointInT>::getHullPointIndices (pcl::PointIndices &hull_point_indices) const
{
  hull_point_indices = hull_indices_;
}

#define PCL_INSTANTIATE_ConcaveHull(T) template class PCL_EXPORTS pcl::ConcaveHull<T>;

#endif    // PCL_SURFACE_IMPL_CONCAVE_HULL_H_
#endif
