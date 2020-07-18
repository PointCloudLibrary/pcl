#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_greedy.h>

#include "typedefs.h"


class Mesh
{
  public:
    Mesh () : points (new PointCloud) {}
    PointCloudPtr points;
    std::vector<pcl::Vertices> faces;
};

using MeshPtr = std::shared_ptr<Mesh>;

PointCloudPtr
smoothPointCloud (const PointCloudPtr & input, float radius, int polynomial_order)
{
  pcl::MovingLeastSquares<PointT, NormalT> mls;
  mls.setSearchMethod (pcl::KdTreeFLANN<PointT>::Ptr (new pcl::KdTreeFLANN<PointT>));
  mls.setSearchRadius (radius);
  mls.setSqrGaussParam (radius*radius);
  mls.setPolynomialFit (polynomial_order > 1);
  mls.setPolynomialOrder (polynomial_order);
  
  mls.setInputCloud (input);

  PointCloudPtr output (new PointCloud);
  mls.reconstruct (*output);

  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, float radius, int polynomial_order)
{
  pcl::MovingLeastSquares<PointT, NormalT> mls;
  mls.setSearchMethod (pcl::KdTreeFLANN<PointT>::Ptr (new pcl::KdTreeFLANN<PointT>));
  mls.setSearchRadius (radius);
  mls.setSqrGaussParam (radius*radius);
  mls.setPolynomialFit (polynomial_order > 1);
  mls.setPolynomialOrder (polynomial_order);
  
  mls.setInputCloud (input);

  PointCloudPtr points (new PointCloud);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  mls.setOutputNormals (normals);
  mls.reconstruct (*points);

  SurfaceElementsPtr surfels (new SurfaceElements);
  pcl::copyPointCloud (*points, *surfels);
  pcl::copyPointCloud (*normals, *surfels);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  pcl::ConvexHull<PointT> convex_hull;
  convex_hull.setInputCloud (input);

  MeshPtr output (new Mesh);
  convex_hull.reconstruct (*(output->points), output->faces);

  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, float alpha)
{
  pcl::ConcaveHull<PointT> concave_hull;
  concave_hull.setInputCloud (input);
  concave_hull.setAlpha (alpha);

  MeshPtr output (new Mesh);
  concave_hull.reconstruct (*(output->points), output->faces);

  return (output);
}

pcl::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, float radius, float mu, int max_nearest_neighbors, 
                     float max_surface_angle, float min_angle, float max_angle)

{
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
  gpt.setSearchMethod (pcl::KdTreeFLANN<pcl::PointNormal>::Ptr (new pcl::KdTreeFLANN<pcl::PointNormal>));

  gpt.setSearchRadius (radius);
  gpt.setMaximumNearestNeighbors (max_nearest_neighbors);
  gpt.setMu (mu);
  gpt.setMaximumSurfaceAgle (max_surface_angle);
  gpt.setMinimumAngle (min_angle);
  gpt.setMaximumAngle (max_angle);
  gpt.setNormalConsistency (true);

  gpt.setInputCloud (surfels);
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  gpt.reconstruct (*output);

  return (output);
}


pcl::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, float leaf_size, float iso_level)
{
  pcl::MarchingCubesGreedy<SurfelT> marching_cubes;
  marching_cubes.setSearchMethod (pcl::KdTree<SurfelT>::Ptr (new pcl::KdTreeFLANN<SurfelT> ()));
  marching_cubes.setLeafSize (leaf_size);
  marching_cubes.setIsoLevel (iso_level);

  marching_cubes.setInputCloud (surfels);
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  marching_cubes.reconstruct (*output);
  
  return (output);
}
