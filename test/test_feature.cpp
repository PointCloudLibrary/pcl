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
/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d_tbb.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/rsd.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/rift.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BaseFeature)
{
  // compute3DCentroid (indices)
  Eigen::Vector4f centroid3;
  compute3DCentroid (cloud, indices, centroid3);
  EXPECT_NEAR (centroid3[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid3[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroid3[2],  0.027302,  1e-4);
  EXPECT_NEAR (centroid3[3],  0,         1e-4);

  // compute3Dcentroid
  compute3DCentroid (cloud, centroid3);
  EXPECT_NEAR (centroid3[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid3[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroid3[2],  0.027302,  1e-4);
  EXPECT_NEAR (centroid3[3],  0,         1e-4);

  // computeNDCentroid (indices)
  Eigen::VectorXf centroidn;
  computeNDCentroid (cloud, indices, centroidn);
  EXPECT_NEAR (centroidn[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroidn[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroidn[2],  0.027302,  1e-4);

  // computeNDCentroid
  computeNDCentroid (cloud, centroidn);
  EXPECT_NEAR (centroidn[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroidn[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroidn[2],  0.027302,  1e-4);

  // computeCovarianceMatrix (indices)
  Eigen::Matrix3f covariance_matrix;
  computeCovarianceMatrix (cloud, indices, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0),  0.710046,  1e-4);
  EXPECT_NEAR (covariance_matrix (0, 1), -0.234843,  1e-4);
  EXPECT_NEAR (covariance_matrix (0, 2),  0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 0), -0.234843,  1e-4);
  EXPECT_NEAR (covariance_matrix (1, 1),  0.68695,   1e-4);
  EXPECT_NEAR (covariance_matrix (1, 2), -0.220504,  1e-4);
  EXPECT_NEAR (covariance_matrix (2, 0),  0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 1), -0.220504,  1e-4);
  EXPECT_NEAR (covariance_matrix (2, 2),  0.195448,  1e-4);

  // computeCovarianceMatrix
  computeCovarianceMatrix (cloud, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0),  0.710046,  1e-4);
  EXPECT_NEAR (covariance_matrix (0, 1), -0.234843,  1e-4);
  EXPECT_NEAR (covariance_matrix (0, 2),  0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 0), -0.234843,  1e-4);
  EXPECT_NEAR (covariance_matrix (1, 1),  0.68695,   1e-4);
  EXPECT_NEAR (covariance_matrix (1, 2), -0.220504,  1e-4);
  EXPECT_NEAR (covariance_matrix (2, 0),  0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 1), -0.220504,  1e-4);
  EXPECT_NEAR (covariance_matrix (2, 2),  0.195448,  1e-4);

  // computeCovarianceMatrixNormalized (indices)
  computeCovarianceMatrixNormalized (cloud, indices, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0),  1.7930e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 1), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 2),  1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 0), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 1),  1.7347e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 2), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 0),  1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 1), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 2),  4.9356e-04, 1e-5);

  // computeCovarianceMatrixNormalized
  computeCovarianceMatrixNormalized (cloud, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0),  1.7930e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 1), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 2),  1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 0), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 1),  1.7347e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 2), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 0),  1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 1), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 2),  4.9356e-04, 1e-5);


  // solvePlaneParameters (Vector)
  Eigen::Vector4f plane_parameters;
  float curvature;
  solvePlaneParameters (covariance_matrix, centroid3, plane_parameters, curvature);
  EXPECT_NEAR (fabs (plane_parameters[0]), 0.035592,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[1]), 0.369596,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[2]), 0.928511,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[3]), 0.0622552, 1e-4);
  EXPECT_NEAR (curvature,                  0.0693136, 1e-4);

  // solvePlaneParameters
  float nx, ny, nz;
  solvePlaneParameters (covariance_matrix, nx, ny, nz, curvature);
  EXPECT_NEAR (fabs (nx), 0.035592,  1e-4);
  EXPECT_NEAR (fabs (ny), 0.369596,  1e-4);
  EXPECT_NEAR (fabs (nz), 0.928511,  1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimation)
{
  Eigen::Vector4f plane_parameters;
  float curvature;

  NormalEstimation<PointXYZ, Normal> n;

  // computePointNormal (indices, Vector)
  n.computePointNormal (cloud, indices, plane_parameters, curvature);
  EXPECT_NEAR (fabs (plane_parameters[0]), 0.035592,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[1]), 0.369596,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[2]), 0.928511,  1e-4);
  EXPECT_NEAR (fabs (plane_parameters[3]), 0.0622552, 1e-4);
  EXPECT_NEAR (curvature,                  0.0693136, 1e-4);

  float nx, ny, nz;
  // computePointNormal (indices)
  n.computePointNormal (cloud, indices, nx, ny, nz, curvature);
  EXPECT_NEAR (fabs (nx), 0.035592,  1e-4);
  EXPECT_NEAR (fabs (ny), 0.369596,  1e-4);
  EXPECT_NEAR (fabs (nz), 0.928511,  1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  // computePointNormal (Vector)
/*  n.computePointNormal (cloud, plane_parameters, curvature);
  EXPECT_NEAR (plane_parameters[0],  0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1],  0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2],  0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3], -0.0622552, 1e-4);
  EXPECT_NEAR (curvature,            0.0693136, 1e-4);

  // computePointNormal
  n.computePointNormal (cloud, nx, ny, nz, curvature);
  EXPECT_NEAR (nx,        0.035592,  1e-4);
  EXPECT_NEAR (ny,        0.369596,  1e-4);
  EXPECT_NEAR (nz,        0.928511,  1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  // flipNormalTowardsViewpoint (Vector)
  n.flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, plane_parameters);
  EXPECT_NEAR (plane_parameters[0], -0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1], -0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2], -0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3],  0.0799743, 1e-4);

  // flipNormalTowardsViewpoint
  n.flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, nx, ny, nz);
  EXPECT_NEAR (nx, -0.035592, 1e-4);
  EXPECT_NEAR (ny, -0.369596, 1e-4);
  EXPECT_NEAR (nz, -0.928511, 1e-4);*/

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (indices.size ());

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());

  for (size_t i = 0; i < normals->points.size (); ++i)
  {
    EXPECT_NEAR (normals->points[i].normal[0], -0.035592,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[1], -0.369596,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[2], -0.928511,  1e-4);
    EXPECT_NEAR (normals->points[i].curvature,  0.0693136, 1e-4);
  }

  PointCloud<PointXYZ>::Ptr surfaceptr = cloudptr;
  n.setSearchSurface (surfaceptr);
  EXPECT_EQ (n.getSearchSurface (), surfaceptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimationOpenMP)
{
  NormalEstimationOMP<PointXYZ, Normal> n (4);    // instantiate 4 threads

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (indices.size ());

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());

  for (size_t i = 0; i < normals->points.size (); ++i)
  {
    EXPECT_NEAR (normals->points[i].normal[0], -0.035592,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[1], -0.369596,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[2], -0.928511,  1e-4);
    EXPECT_NEAR (normals->points[i].curvature,  0.0693136, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*TEST (PCL, NormalEstimationTBB)
{
  NormalEstimationTBB<PointXYZ, Normal> n;

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (indices.size ());

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());

  for (size_t i = 0; i < normals->points.size (); ++i)
  {
    EXPECT_NEAR (normals->points[i].normal[0], -0.035592,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[1], -0.369596,  1e-4);
    EXPECT_NEAR (normals->points[i].normal[2], -0.928511,  1e-4);
    EXPECT_NEAR (normals->points[i].curvature,  0.0693136, 1e-4);
  }
}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MomentInvariantsEstimation)
{
  float j1, j2, j3;

  MomentInvariantsEstimation<PointXYZ, MomentInvariants> mi;

  // computePointMomentInvariants (indices))
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244,  1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // computePointMomentInvariants
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244,  1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // Object
  PointCloud<MomentInvariants>::Ptr moments (new PointCloud<MomentInvariants> ());

  // set parameters
  mi.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  mi.setIndices (indicesptr);
  mi.setSearchMethod (tree);
  mi.setKSearch (indices.size ());

  // estimate
  mi.compute (*moments);
  EXPECT_EQ (moments->points.size (), indices.size ());

  for (size_t i = 0; i < moments->points.size (); ++i)
  {
    EXPECT_NEAR (moments->points[i].j1, 1.59244,  1e-4);
    EXPECT_NEAR (moments->points[i].j2, 0.652063, 1e-4);
    EXPECT_NEAR (moments->points[i].j3, 0.053917, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BoundaryEstimation)
{
  Eigen::Vector3f u = Eigen::Vector3f::Zero ();
  Eigen::Vector3f v = Eigen::Vector3f::Zero ();

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (indices.size ());
  // estimate
  n.compute (*normals);

  BoundaryEstimation<PointXYZ, Normal, Boundary> b;
  b.setInputNormals (normals);
  EXPECT_EQ (b.getInputNormals (), normals);

  // getCoordinateSystemOnPlane
  for (size_t i = 0; i < normals->points.size (); ++i)
  {
    b.getCoordinateSystemOnPlane (normals->points[i], u, v);
    pcl::Vector3fMap n4uv = normals->points[i].getNormalVector3fMap ();
    EXPECT_NEAR (n4uv.dot(u),  0,  1e-4);
    EXPECT_NEAR (n4uv.dot(v),  0,  1e-4);
    EXPECT_NEAR (u.dot(v),     0,  1e-4);
  }

  // isBoundaryPoint (indices)
  bool pt = false;
  pt = b.isBoundaryPoint (cloud, 0, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () / 3, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () / 2, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () - 1, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, true);

  // isBoundaryPoint (points)
  pt = false;
  pt = b.isBoundaryPoint (cloud, cloud.points[0], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () / 3], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () / 2], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () - 1], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, true);

  // Object
  PointCloud<Boundary>::Ptr bps (new PointCloud<Boundary> ());

  // set parameters
  b.setInputCloud (cloud.makeShared ());
  b.setIndices (indicesptr);
  b.setSearchMethod (tree);
  b.setKSearch (indices.size ());

  // estimate
  b.compute (*bps);
  EXPECT_EQ (bps->points.size (), indices.size ());

  pt = bps->points[0].boundary_point;
  EXPECT_EQ (pt, false);
  pt = bps->points[indices.size () / 3].boundary_point;
  EXPECT_EQ (pt, false);
  pt = bps->points[indices.size () / 2].boundary_point;
  EXPECT_EQ (pt, false);
  pt = bps->points[indices.size () - 1].boundary_point;
  EXPECT_EQ (pt, true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PrincipalCurvaturesEstimation)
{
  float pcx, pcy, pcz, pc1, pc2;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PrincipalCurvaturesEstimation<PointXYZ, Normal, PrincipalCurvatures> pc;
  pc.setInputNormals (normals);
  EXPECT_EQ (pc.getInputNormals (), normals);

  // computePointPrincipalCurvatures (indices)
  pc.computePointPrincipalCurvatures (*normals, 0, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (fabs (pcx), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcy), 0.10714, 1e-4);
  EXPECT_NEAR (fabs (pcz), 0.13462, 1e-4);
  EXPECT_NEAR (pc1, 95.26991,       1e-3);
  EXPECT_NEAR (pc2, 77.0189,        1e-3);

  pc.computePointPrincipalCurvatures (*normals, 2, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.98079,   1e-4);
  EXPECT_NEAR (pcy, -0.04019,  1e-4);
  EXPECT_NEAR (pcz, 0.19086,   1e-4);
  EXPECT_NEAR (pc1, 108.01375, 1e-4);
  EXPECT_NEAR (pc2,  77.27593, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, indices.size () - 3, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725,   1e-4);
  EXPECT_NEAR (pcy, -0.37599,  1e-4);
  EXPECT_NEAR (pcz, 0.32635,   1e-4);
  EXPECT_NEAR (pc1, 102.82327, 1e-4);
  EXPECT_NEAR (pc2, 71.0906,   1e-4);

  pc.computePointPrincipalCurvatures (*normals, indices.size () - 1, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725,   1e-4);
  EXPECT_NEAR (pcy, -0.37599,  1e-4);
  EXPECT_NEAR (pcz, 0.32636,   1e-4);
  EXPECT_NEAR (pc1, 102.82323, 1e-4);
  EXPECT_NEAR (pc2,  71.09056, 1e-4);

  // Object
  PointCloud<PrincipalCurvatures>::Ptr pcs (new PointCloud<PrincipalCurvatures> ());

  // set parameters
  pc.setInputCloud (cloud.makeShared ());
  pc.setIndices (indicesptr);
  pc.setSearchMethod (tree);
  pc.setKSearch (indices.size ());

  // estimate
  pc.compute (*pcs);
  EXPECT_EQ (pcs->points.size (), indices.size ());

  // Adjust for small numerical inconsitencies (due to nn_indices not being sorted)
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[0]), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[1]), 0.10713, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[2]), 0.13462, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].pc1), 95.26995, 1e-3);
  EXPECT_NEAR (fabs (pcs->points[0].pc2), 77.01882, 1e-3);

  EXPECT_NEAR (pcs->points[2].principal_curvature[0], 0.98079, 1e-4);
  EXPECT_NEAR (pcs->points[2].principal_curvature[1], -0.04019, 1e-4);
  EXPECT_NEAR (pcs->points[2].principal_curvature[2], 0.19086, 1e-4);
  EXPECT_NEAR (pcs->points[2].pc1, 108.0137481, 1e-4);
  EXPECT_NEAR (pcs->points[2].pc2,  77.2759780, 1e-4);

  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[0],  0.86725, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[1], -0.37599, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[2],  0.32636, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].pc1, 102.82320, 1e-3);
  EXPECT_NEAR (pcs->points[indices.size () - 3].pc2,  71.09062, 1e-4);

  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[0],  0.86725, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[1], -0.37599, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[2],  0.32636, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].pc1, 102.82326, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].pc2,  71.09061, 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PFHEstimation)
{
  float f1, f2, f3, f4;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PFHEstimation<PointXYZ, Normal, PFHSignature125> pfh;
  pfh.setInputNormals (normals);
  EXPECT_EQ (pfh.getInputNormals (), normals);

  // computePairFeatures
  pfh.computePairFeatures (cloud, *normals, 0, 12, f1, f2, f3, f4);
  EXPECT_NEAR (f1, -0.0725751,   1e-4);
  EXPECT_NEAR (f2, -0.0402214,   1e-4);
  EXPECT_NEAR (f3,  0.0681325,   1e-4);
  EXPECT_NEAR (f4,  0.006130435, 1e-4);

  // computePointPFHSignature
  int nr_subdiv = 3;
  Eigen::VectorXf pfh_histogram (nr_subdiv * nr_subdiv * nr_subdiv);
  pfh.computePointPFHSignature (cloud, *normals, indices, nr_subdiv, pfh_histogram);
  EXPECT_NEAR (pfh_histogram[0],   0.534875, 1e-4);
  EXPECT_NEAR (pfh_histogram[1],   1.48149,  1e-4);
  EXPECT_NEAR (pfh_histogram[2],   0.211284, 1e-4);
  EXPECT_NEAR (pfh_histogram[3],   0.751871, 1e-4);
  EXPECT_NEAR (pfh_histogram[4],   3.25473,  1e-4);
  EXPECT_NEAR (pfh_histogram[5],   0.190981, 1e-4);
  EXPECT_NEAR (pfh_histogram[6],   1.07038,  1e-4);
  EXPECT_NEAR (pfh_histogram[7],   2.53465,  1e-4);
  EXPECT_NEAR (pfh_histogram[8],   0.190346, 1e-4);
  EXPECT_NEAR (pfh_histogram[9],   1.78031,  1e-4);
  EXPECT_NEAR (pfh_histogram[10],  4.37654,  1e-4);
  EXPECT_NEAR (pfh_histogram[11],  0.707457, 1e-4);
  EXPECT_NEAR (pfh_histogram[12],  2.16224,  1e-4);
  EXPECT_NEAR (pfh_histogram[13], 18.23467,  1e-4);
  EXPECT_NEAR (pfh_histogram[14],  0.737912, 1e-4);
  EXPECT_NEAR (pfh_histogram[15],  3.29406,  1e-4);
  EXPECT_NEAR (pfh_histogram[16],  8.59094,  1e-4);
  EXPECT_NEAR (pfh_histogram[17],  0.831817, 1e-4);
  EXPECT_NEAR (pfh_histogram[18],  5.92704,  1e-4);
  EXPECT_NEAR (pfh_histogram[19],  3.8936,   1e-4);
  EXPECT_NEAR (pfh_histogram[20],  1.00313,  1e-4);
  EXPECT_NEAR (pfh_histogram[21],  11.1144,  1e-4);
  EXPECT_NEAR (pfh_histogram[22],  11.2659,  1e-4);
  EXPECT_NEAR (pfh_histogram[23],  1.32352,  1e-4);
  EXPECT_NEAR (pfh_histogram[24],  6.23105,  1e-4);
  EXPECT_NEAR (pfh_histogram[25],  6.21518,  1e-4);
  EXPECT_NEAR (pfh_histogram[26],  1.83741,  1e-4);

  // Object
  PointCloud<PFHSignature125>::Ptr pfhs (new PointCloud<PFHSignature125> ());

  // set parameters
  pfh.setInputCloud (cloud.makeShared ());
  pfh.setIndices (indicesptr);
  pfh.setSearchMethod (tree);
  pfh.setNrSubdivisions (3);
  pfh.setKSearch (indices.size ());

  // estimate
  pfh.compute (*pfhs);
  EXPECT_EQ (pfhs->points.size (), indices.size ());

  for (size_t i = 0; i < pfhs->points.size (); ++i)
  {
    EXPECT_NEAR (pfhs->points[i].histogram[0],   0.534875, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[1],   1.48149,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[2],   0.211284, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[3],   0.751871, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[4],   3.25473,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[5],   0.190981, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[6],   1.07038,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[7],   2.53465,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[8],   0.190346, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[9],   1.78031,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[10],  4.37654,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[11],  0.707457, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[12],  2.16224,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[13], 18.23467,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[14],  0.737912, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[15],  3.29406,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[16],  8.59094,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[17],  0.831817, 1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[18],  5.92704,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[19],  3.8936,   1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[20],  1.00313,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[21],  11.1144,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[22],  11.2659,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[23],  1.32352,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[24],  6.23105,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[25],  6.21518,  1e-4);
    EXPECT_NEAR (pfhs->points[i].histogram[26],  1.83741,  1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
  fpfh.setInputNormals (normals);
  EXPECT_EQ (fpfh.getInputNormals (), normals);

  // computePointSPFHSignature
  int nr_subdiv = 11;       // use the same number of bins for all three angular features
  Eigen::MatrixXf hist_f1 (indices.size (), nr_subdiv), hist_f2 (indices.size (), nr_subdiv), hist_f3 (indices.size (), nr_subdiv);
  hist_f1.setZero (); hist_f2.setZero (); hist_f3.setZero ();
  for (size_t i = 0; i < indices.size (); ++i)
    fpfh.computePointSPFHSignature (cloud, *normals, i, indices, hist_f1, hist_f2, hist_f3);

  EXPECT_NEAR (hist_f1 (0, 0),   2.77778,   1e-4);
  EXPECT_NEAR (hist_f1 (0, 1),   1.010101,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 2),   4.0404,    1e-4);
  EXPECT_NEAR (hist_f1 (0, 3),  19.1919,    1e-4);
  EXPECT_NEAR (hist_f1 (0, 4),  40.1515,    1e-4);
  EXPECT_NEAR (hist_f1 (0, 5),  20.4545,    1e-4);
  EXPECT_NEAR (hist_f1 (0, 6),   8.58586,   1e-4);
  EXPECT_NEAR (hist_f1 (0, 7),   1.0101,    1e-4);
  EXPECT_NEAR (hist_f1 (0, 8),   1.26263,   1e-4);
  EXPECT_NEAR (hist_f1 (0, 9),   0,         1e-4);
  EXPECT_NEAR (hist_f1 (0, 10),  1.51515,   1e-4);

  EXPECT_NEAR (hist_f2 (0, 0),   0,         1e-4);
  EXPECT_NEAR (hist_f2 (0, 1),   0.50505,   1e-4);
  EXPECT_NEAR (hist_f2 (0, 2),   2.27273,   1e-4);
  EXPECT_NEAR (hist_f2 (0, 3),  10.6061,    1e-4);
  EXPECT_NEAR (hist_f2 (0, 4),  24.495,     1e-4);
  EXPECT_NEAR (hist_f2 (0, 5),  20.7071,    1e-4);
  EXPECT_NEAR (hist_f2 (0, 6),  17.1717,    1e-4);
  EXPECT_NEAR (hist_f2 (0, 7),  11.8687,    1e-4);
  EXPECT_NEAR (hist_f2 (0, 8),   8.08081,   1e-4);
  EXPECT_NEAR (hist_f2 (0, 9),   1.76768,   1e-4);
  EXPECT_NEAR (hist_f2 (0, 10),  2.52525,   1e-4);

  EXPECT_NEAR (hist_f3 (0, 0),   0,          1e-4);
  EXPECT_NEAR (hist_f3 (0, 1),   0,          1e-4);
  EXPECT_NEAR (hist_f3 (0, 2),   0,          1e-4);
  EXPECT_NEAR (hist_f3 (0, 3),   0,          1e-4);
  EXPECT_NEAR (hist_f3 (0, 4),   0.252525,   1e-4);
  EXPECT_NEAR (hist_f3 (0, 5),  14.1414,     1e-4);
  EXPECT_NEAR (hist_f3 (0, 6),  26.0101,     1e-4);
  EXPECT_NEAR (hist_f3 (0, 7),  42.6768,     1e-4);
  EXPECT_NEAR (hist_f3 (0, 8),  13.8889,     1e-4);
  EXPECT_NEAR (hist_f3 (0, 9),   3.0303,     1e-4);
  EXPECT_NEAR (hist_f3 (0, 10),  0,          1e-4);

  // weightPointSPFHSignature
  Eigen::VectorXf fpfh_histogram (nr_subdiv + nr_subdiv + nr_subdiv);
  fpfh_histogram.setZero ();
  vector<float> dists (indices.size ());
  for (size_t i = 0; i < dists.size (); ++i) dists[i] = i;
  fpfh.weightPointSPFHSignature (hist_f1, hist_f2, hist_f3, indices, dists, fpfh_histogram);

  EXPECT_NEAR (fpfh_histogram[0],   2.25806,   1e-4);
  EXPECT_NEAR (fpfh_histogram[1],   4.20886,   1e-4);
  EXPECT_NEAR (fpfh_histogram[2],   8.72928,   1e-4);
  EXPECT_NEAR (fpfh_histogram[3],  23.2465,    1e-4);
  EXPECT_NEAR (fpfh_histogram[4],  29.5395,    1e-4);
  EXPECT_NEAR (fpfh_histogram[5],  17.46889,   1e-4);
  EXPECT_NEAR (fpfh_histogram[6],   8.41421,   1e-4);
  EXPECT_NEAR (fpfh_histogram[7],   2.30139,   1e-4);
  EXPECT_NEAR (fpfh_histogram[8],   1.41585,   1e-4);
  EXPECT_NEAR (fpfh_histogram[9],   1.00892,   1e-4);
  EXPECT_NEAR (fpfh_histogram[10],  1.40861,   1e-4);
  EXPECT_NEAR (fpfh_histogram[11],  1.21368,   1e-4);
  EXPECT_NEAR (fpfh_histogram[12],  3.28168,   1e-4);
  EXPECT_NEAR (fpfh_histogram[13],  7.06985,   1e-4);
  EXPECT_NEAR (fpfh_histogram[14], 11.0509,    1e-4);
  EXPECT_NEAR (fpfh_histogram[15], 18.9273,    1e-4);
  EXPECT_NEAR (fpfh_histogram[16], 17.6086,    1e-4);
  EXPECT_NEAR (fpfh_histogram[17], 13.3466,    1e-4);
  EXPECT_NEAR (fpfh_histogram[18], 10.4234,    1e-4);
  EXPECT_NEAR (fpfh_histogram[19],  7.71453,   1e-4);
  EXPECT_NEAR (fpfh_histogram[20],  4.90723,   1e-4);
  EXPECT_NEAR (fpfh_histogram[21],  4.45617,   1e-4);
  EXPECT_NEAR (fpfh_histogram[22],  0.364497,  1e-4);
  EXPECT_NEAR (fpfh_histogram[23],  0.653451,  1e-4);
  EXPECT_NEAR (fpfh_histogram[24],  1.7847,    1e-4);
  EXPECT_NEAR (fpfh_histogram[25],  3.74132,   1e-4);
  EXPECT_NEAR (fpfh_histogram[26],  5.708313,  1e-4);
  EXPECT_NEAR (fpfh_histogram[27],  9.507835,  1e-4);
  EXPECT_NEAR (fpfh_histogram[28], 17.638830,  1e-4);
  EXPECT_NEAR (fpfh_histogram[29], 22.766557,  1e-4);
  EXPECT_NEAR (fpfh_histogram[30], 19.5883,    1e-4);
  EXPECT_NEAR (fpfh_histogram[31], 13.063,     1e-4);
  EXPECT_NEAR (fpfh_histogram[32],  5.18325,   1e-4);

  // Object
  PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (indices.size ());

  // estimate
  fpfh.compute (*fpfhs);
  EXPECT_EQ (fpfhs->points.size (), indices.size ());

  EXPECT_NEAR (fpfhs->points[0].histogram[0],   2.11328,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[1],   3.13866,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[2],   7.07176,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[3],  23.0986,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[4],  32.988,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[5],  18.74372,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[6],   8.118416,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[7],   1.9162,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[8],   1.19554,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[9],   0.577558,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[10],  1.03827,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[11],  0.631236,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[12],  2.13356,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[13],  5.67842,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[14], 10.8759,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[15], 20.2439,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[16], 19.674,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[17], 15.3302,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[18], 10.773,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[19],  6.80136,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[20],  4.03065,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[21],  3.82776,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[22],  0.208905,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[23],  0.392544,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[24],  1.27637,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[25],  2.61976,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[26],  5.12960,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[27], 12.35568,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[28], 21.89877,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[29], 25.55738,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[30], 19.1552,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[31],  9.22763,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[32],  2.17815,   1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimationOpenMP)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh (4);    // instantiate 4 threads
  fpfh.setInputNormals (normals);

  // Object
  PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (indices.size ());

  // estimate
  fpfh.compute (*fpfhs);
  EXPECT_EQ (fpfhs->points.size (), indices.size ());

  EXPECT_NEAR (fpfhs->points[0].histogram[0],   2.11328,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[1],   3.13866,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[2],   7.07176,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[3],  23.0986,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[4],  32.988,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[5],  18.74372,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[6],   8.118416,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[7],   1.9162,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[8],   1.19554,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[9],   0.577558,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[10],  1.03827,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[11],  0.631236,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[12],  2.13356,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[13],  5.67842,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[14], 10.8759,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[15], 20.2439,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[16], 19.674,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[17], 15.3302,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[18], 10.773,     1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[19],  6.80136,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[20],  4.03065,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[21],  3.82776,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[22],  0.208905,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[23],  0.392544,  1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[24],  1.27637,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[25],  2.61976,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[26],  5.12960,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[27], 12.35568,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[28], 21.89877,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[29], 25.55738,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[30], 19.1552,    1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[31],  9.22763,   1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[32],  2.17815,   1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, VFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  VFHEstimation<PointXYZ, Normal, VFHSignature308> vfh;
  vfh.setInputNormals (normals);

//  PointCloud<PointNormal> cloud_normals;
//  pcl::concatenateFields (cloud, normals, cloud_normals);
//  savePCDFile ("bun0_n.pcd", cloud_normals);

  // Object
  PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

  // set parameters
  vfh.setInputCloud (cloud.makeShared ());
  vfh.setNrSubdivisions (45, 45, 45, 45);
  vfh.setNrViewpointSubdivisions (128);
  vfh.setIndices (indicesptr);
  vfh.setSearchMethod (tree);

  // estimate
  vfh.compute (*vfhs);
  EXPECT_EQ ((int)vfhs->points.size (), 1);

  //for (size_t d = 0; d < 308; ++d)
  //  std::cerr << vfhs.points[0].histogram[d] << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, RSDEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD> rsd;
  rsd.setInputNormals (normals);

  // Object
  PointCloud<PrincipalRadiiRSD>::Ptr rsds (new PointCloud<PrincipalRadiiRSD> ());

  // set parameters
  rsd.setInputCloud (cloud.makeShared ());
  rsd.setIndices (indicesptr);
  rsd.setSearchMethod (tree);
  rsd.setRadiusSearch (0.015);

  // estimate
  rsd.compute (*rsds);
//  EXPECT_NEAR (rsds->points[0].r_min, 0.04599, 0.005);
//  EXPECT_NEAR (rsds->points[0].r_max, 0.07053, 0.005);

  // Save output
  //PointCloud<PointNormal> normal_cloud;
  //pcl::concatenateFields (cloud, *normals, normal_cloud);
  //savePCDFile ("./test/bun0-normal.pcd", normal_cloud);
  //savePCDFile ("./test/bun0-rsd.pcd", *rsds);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensityGradientEstimation)
{
  // Create a test cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -5.0; x <= 5.0; x += 0.1)
  {
    for (float y = -5.0; y <= 5.0; y += 0.1)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = 0.1*pow (x,2) + 0.5*y + 1.0;
      p.intensity = 0.1*pow (x,3) + 0.2*pow (y,2) + 1.0*p.z + 2.0;

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();
  PointCloud<PointXYZI>::ConstPtr cloud_ptr = cloud_xyzi.makeShared ();

  // Estimate surface normals
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  NormalEstimation<PointXYZI, Normal> norm_est;
  norm_est.setInputCloud (cloud_ptr);
  KdTreeFLANN<PointXYZI>::Ptr treept1 (new KdTreeFLANN<PointXYZI> (false));
  norm_est.setSearchMethod (treept1);
  norm_est.setRadiusSearch (0.25);
  norm_est.compute (*normals);

  // Estimate intensity gradient
  PointCloud<IntensityGradient> gradient;
  IntensityGradientEstimation<PointXYZI, Normal, IntensityGradient> grad_est;
  grad_est.setInputCloud (cloud_ptr);
  grad_est.setInputNormals (normals);
  KdTreeFLANN<PointXYZI>::Ptr treept2 (new KdTreeFLANN<PointXYZI> (false));
  grad_est.setSearchMethod (treept2);
  grad_est.setRadiusSearch (0.25);
  grad_est.compute (gradient);

  // Compare to gradient estimates to actual values
  for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
  {
    const PointXYZI &p = cloud_ptr->points[i];

    // A pointer to the estimated gradient values
    const float * g_est = gradient.points[i].gradient;

    // Compute the surface normal analytically.
    float nx = -0.2*p.x;
    float ny = -0.5;
    float nz = 1.0;
    float magnitude = sqrt(nx*nx + ny*ny + nz*nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = (0.3 * pow (p.x,2));
    float tmpy = (0.4 * p.y);
    float tmpz = (1.0);
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1-nx*nx)*tmpx +  (-nx*ny)*tmpy +  (-nx*nz)*tmpz;
    float gy =  (-ny*nx)*tmpx + (1-ny*ny)*tmpy +  (-ny*nz)*tmpz;
    float gz =  (-nz*nx)*tmpx +  (-nz*ny)*tmpy + (1-nz*nz)*tmpz;

    // Compare the estimates to the derived values.
    const float tolerance = 0.1;
    EXPECT_NEAR (g_est[0], gx, tolerance);
    EXPECT_NEAR (g_est[1], gy, tolerance);
    EXPECT_NEAR (g_est[2], gz, tolerance);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensitySpinEstimation)
{
  // Generate a sample point cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;

  for (float x = -10.0; x <= 10.0; x += 1.0)
  {
    for (float y = -10.0; y <= 10.0; y += 1.0)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = sqrt(400 - x*x - y*y);
      p.intensity = 
        exp (-(pow (x-3, 2) + pow (y+2, 2)) / (2*25.0)) +
        exp (-(pow (x+5, 2) + pow (y-5, 2)) / (2*4.0));

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();

  // Compute the intensity-domain spin features
  typedef Histogram<20> IntensitySpin;
  IntensitySpinEstimation<PointXYZI, IntensitySpin> ispin_est;
  KdTreeFLANN<PointXYZI>::Ptr treept3 (new KdTreeFLANN<PointXYZI> (false));
  ispin_est.setSearchMethod (treept3);
  ispin_est.setRadiusSearch (10.0);
  ispin_est.setNrDistanceBins (4);
  ispin_est.setNrIntensityBins (5);

  ispin_est.setInputCloud (cloud_xyzi.makeShared ());
  PointCloud<IntensitySpin> ispin_output;
  ispin_est.compute (ispin_output);

  // Compare to independently verified values
  const IntensitySpin &ispin = ispin_output.points[220];
  const float correct_ispin_feature_values[20] = 
  {
     2.4387,   9.4737,  21.3232,  28.3025,  22.5639,
    13.2426,  35.7026,  60.0755,  66.9240,  50.4225,
    42.7086,  83.5818, 105.4513,  97.8454,  67.3801,
    75.7127, 119.4726, 120.9649,  93.4829,  55.4045
  };
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_NEAR (ispin.histogram[i], correct_ispin_feature_values[i], 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, RIFTEstimation)
{
  // Generate a sample point cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -10.0; x <= 10.0; x += 1.0)
  {
    for (float y = -10.0; y <= 10.0; y += 1.0)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = sqrt(400 - x*x - y*y);
      p.intensity = 
        exp ((-pow (x-3, 2) + pow (y+2, 2)) / (2*25.0)) +
        exp ((-pow (x+5, 2) + pow (y-5, 2)) / (2*4.0));

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();

  // Generate the intensity gradient data
  PointCloud<IntensityGradient> gradient;
  gradient.height = 1;
  gradient.width = cloud_xyzi.points.size ();
  gradient.is_dense = true;
  gradient.points.resize (gradient.width);
  for (size_t i = 0; i < cloud_xyzi.points.size (); ++i)
  {
    const PointXYZI &p = cloud_xyzi.points[i];

    // Compute the surface normal analytically.
    float nx = p.x;
    float ny = p.y;
    float nz = p.z;
    float magnitude = sqrt(nx*nx + ny*ny + nz*nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = -(p.x + 5)/4.0  / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2))/8.0) - 
                  (p.x - 3)/25.0 / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2))/50.0);
    float tmpy = -(p.y - 5)/4.0  / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2))/8.0) - 
                  (p.y + 2)/25.0 / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2))/50.0);
    float tmpz = 0;
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1-nx*nx)*tmpx +  (-nx*ny)*tmpy +  (-nx*nz)*tmpz;
    float gy =  (-ny*nx)*tmpx + (1-ny*ny)*tmpy +  (-ny*nz)*tmpz;
    float gz =  (-nz*nx)*tmpx +  (-nz*ny)*tmpy + (1-nz*nz)*tmpz;

    gradient.points[i].gradient[0] = gx;
    gradient.points[i].gradient[1] = gy;
    gradient.points[i].gradient[2] = gz;
  }

  // Compute the RIFT features
  typedef Histogram<32> RIFTDescriptor;
  RIFTEstimation<PointXYZI, IntensityGradient, RIFTDescriptor> rift_est;
  KdTreeFLANN<PointXYZI>::Ptr treept4 (new KdTreeFLANN<PointXYZI> (false));
  rift_est.setSearchMethod (treept4);
  rift_est.setRadiusSearch (10.0);
  rift_est.setNrDistanceBins (4);
  rift_est.setNrGradientBins (8);

  rift_est.setInputCloud (cloud_xyzi.makeShared ());
  rift_est.setInputGradient (gradient.makeShared ());
  PointCloud<RIFTDescriptor> rift_output;
  rift_est.compute (rift_output);

  // Compare to independently verified values
  const RIFTDescriptor &rift = rift_output.points[220];
  const float correct_rift_feature_values[32] = 
    {
      0.0187, 0.0349, 0.0647, 0.0881,
      0.0042, 0.0131, 0.0346, 0.0030,
      0.0076, 0.0218, 0.0463, 0.0030,
      0.0087, 0.0288, 0.0920, 0.0472,
      0.0076, 0.0420, 0.0726, 0.0669,
      0.0090, 0.0901, 0.1274, 0.2185,
      0.0147, 0.1222, 0.3568, 0.4348,
      0.0149, 0.0806, 0.2787, 0.6864
    };
  for (int i = 0; i < 32; ++i)
  {
    EXPECT_NEAR (rift.histogram[i], correct_rift_feature_values[i], 1e-4);
  }
}

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  sensor_msgs::PointCloud2 cloud_blob;
  if (loadPCDFile (argv[1], cloud_blob) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  fromROSMsg (cloud_blob, cloud);

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i) { indices[i] = i; }

  tree.reset (new KdTreeFLANN<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
