/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_SURFACE_IMPL_ADVANCING_FRONT_H_
#define PCL_SURFACE_IMPL_ADVANCING_FRONT_H_

#include <pcl/surface/advancing_front.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/common/normals.h>

#ifdef AFRONTDEBUG
#include <pcl/surface/organized_fast_mesh.h>
#include <sstream>
#endif

#include <eigen3/Eigen/LU>

#include <boost/lambda/lambda.hpp>
#include <boost/make_shared.hpp>

template <typename PointNT>
pcl::AfrontMesher<PointNT>::AfrontMesher () : search_radius_ (0.0)
{
  pcl::PointCloud<typename MeshTraits::VertexData> &mesh_data = mesh_.getVertexDataCloud ();
  mesh_vertex_data_ptr_ = typename pcl::PointCloud<typename MeshTraits::VertexData>::Ptr (&mesh_data, boost::bind (std::plus<int> (), 0, 0));

#ifdef _OPENMP
  setNumberOfThreads (AFRONT_DEFAULT_THREADS);
#endif

  setRho (AFRONT_DEFAULT_RHO);
  setReduction (AFRONT_DEFAULT_REDUCTION);
  setPolynomialOrder (AFRONT_DEFAULT_POLYNOMIAL_ORDER);
  setBoundaryAngleThreshold (AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD);
  setSampleSize(AFRONT_DEFAULT_SAMPLE_SIZE);
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::initialize ()
{
  initialized_ = false;
  finished_ = false;
  queue_.clear ();
  boundary_.clear ();
  mesh_.clear ();

#ifdef AFRONTDEBUG
  fence_counter_ = 0;
#endif

  // Check the only parameter that does not have a default.
  if (search_radius_ <= 0)
  {
    PCL_ERROR ("Invalid search radius (%f)!\n", search_radius_);
    return false;
  }

  // Generate the MLS surface
  if (!computeGuidanceField ())
  {
    PCL_ERROR ("Failed to compute Guidance Field! Try increasing radius.\n");
    return false;
  }

  mesh_octree_ = typename pcl::octree::OctreePointCloudSearch<typename MeshTraits::VertexData>::Ptr (new pcl::octree::OctreePointCloudSearch<typename MeshTraits::VertexData> (search_radius_));
  mesh_vertex_data_indices_ = pcl::IndicesPtr (new std::vector<int>);
  mesh_octree_->setInputCloud (mesh_vertex_data_ptr_, mesh_vertex_data_indices_);

  // Create first triangle
  createFirstTriangle (rand () % mls_cloud_->size ());

  initialized_ = true;

#ifdef AFRONTDEBUG
  viewer_ = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_->initCameraParameters ();
  viewer_->setBackgroundColor (0, 0, 0);
  pcl::PolygonMesh out_mesh;
  pcl::geometry::toFaceVertexMesh (mesh_, out_mesh);
  viewer_->addPolygonMesh (out_mesh);

  int v1 = 1;
  viewer_->createViewPort (0.0, 0.5, 0.5, 1.0, v1);
  pcl::visualization::PointCloudColorHandlerCustom<PointNT> single_color (input_, 0, 255, 0);
  viewer_->addPointCloud<PointNT> (input_, single_color, "sample cloud", v1);

  //Show just mesh
  int v2 = 2;
  viewer_->createViewPort (0.5, 0.5, 1.0, 1.0, v2);

  //Show Final mesh results over the mls point cloud
  int v3 = 3;
  viewer_->createViewPort (0.0, 0.0, 0.5, 0.5, v3);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::afront::AfrontGuidanceFieldPointType> handler_k (mls_cloud_, "curvature");
  viewer_->addPointCloud<pcl::afront::AfrontGuidanceFieldPointType> (mls_cloud_, handler_k, "mls_cloud", v3);

  //Show mls information
  int v4 = 4;
  viewer_->createViewPort (0.5, 0.0, 1.0, 0.5, v4);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::afront::AfrontGuidanceFieldPointType> single_color2 (mls_cloud_, 0, 255, 0);
  viewer_->addPointCloud<pcl::afront::AfrontGuidanceFieldPointType> (mls_cloud_, single_color2, "mls_cloud2", v4);

  viewer_->registerKeyboardCallback (&pcl::AfrontMesher<PointNT>::keyboardEventOccurred, *this);
  viewer_->spin ();
#endif

  return true;
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::computeGuidanceField ()
{
  PCL_INFO ("Computing Guidance Field Started!\n");

  // Calculate MLS
  mls_cloud_ = pcl::PointCloud<pcl::afront::AfrontGuidanceFieldPointType>::Ptr (new pcl::PointCloud<pcl::afront::AfrontGuidanceFieldPointType> ());

#ifdef _OPENMP
  mls_.setNumberOfThreads (threads_);
#endif
  mls_.setComputeNormals (true);
  mls_.setInputCloud (input_);
  mls_.setIndices (indices_);
  mls_.setPolynomialOrder (polynomial_order_);
  mls_.setSearchMethod (tree_);
  mls_.setSearchRadius (search_radius_);
  mls_.setCacheMLSResults (true);

  mls_.process (*mls_cloud_);
  if (mls_cloud_->empty ())
    return false;

  // This is required because getCorrespondingIndices is not a const function
  mls_corresponding_input_indices_ = mls_.getCorrespondingIndices ();

  // Calculate the max principle curvature using mls result polynomial data
  max_curvature_ = std::numeric_limits<double>::min ();
  min_curvature_ = std::numeric_limits<double>::max ();
  for (int i = 0; i < mls_cloud_->size (); ++i)
  {
    Eigen::Vector3d point = mls_cloud_->at (i).getVector3fMap ().template cast<double> ();
    int index = mls_corresponding_input_indices_->indices[i];
    const pcl::MLSResult &mls_result = mls_.getMLSResults ()[index];

    double u, v;
    mls_result.getMLSCoordinates (point, u, v);
    double k = mls_result.calculatePrincipleCurvatures (u, v).cwiseAbs ().maxCoeff ();

    mls_cloud_->at (i).curvature = k;
    mls_cloud_->at (i).ideal_edge_length = 2.0 * std::sin (rho_ / 2.0) / k;

    // Store min and max curvature
    if (k > max_curvature_)
      max_curvature_ = k;

    if (k < min_curvature_)
      min_curvature_ = k;
  }

  mls_cloud_tree_ = pcl::search::KdTree<pcl::afront::AfrontGuidanceFieldPointType>::Ptr (new pcl::search::KdTree<pcl::afront::AfrontGuidanceFieldPointType> ());
  mls_cloud_tree_->setSortedResults (true); // Need to figure out how to use unsorted. Must rewrite getMaxStep.
  mls_cloud_tree_->setInputCloud (mls_cloud_);

  PCL_INFO ("Computing Guidance Field Finished!\n");
  return true;
}

template <typename PointNT> pcl::PointCloud<pcl::Normal>::ConstPtr
pcl::AfrontMesher<PointNT>::getMeshVertexNormals () const
{
  pcl::PointCloud<pcl::Normal>::Ptr pn (new pcl::PointCloud<pcl::Normal> ());
  ;
  pcl::copyPointCloud (*mesh_vertex_data_ptr_, *pn);

  return pn;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::performReconstruction (pcl::PolygonMesh &output)
{
  if (!initialize ())
  {
    PCL_ERROR ("Afront mesher failed to been initialized!\n");
    return;
  }

  PCL_INFO ("Meshing Started!\n");
  long unsigned int cnt = 0;
  while (!finished_)
    stepReconstruction (++cnt);

  pcl::geometry::toFaceVertexMesh (mesh_, output);

  PCL_INFO ("Meshing Finished!\n");
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                    std::vector<pcl::Vertices> &polygons)
{
  if (!initialize ())
  {
    PCL_ERROR ("Afront mesher failed to been initialized!\n");
    return;
  }

  PCL_INFO ("Meshing Started!\n");
  long unsigned int cnt = 0;
  while (!finished_)
    stepReconstruction (++cnt);

  points.clear ();
  pcl::copyPointCloud (*mesh_vertex_data_ptr_, points);

  pcl::Vertices polygon;

  polygons.clear ();
  polygons.reserve (mesh_.sizeFaces ());
  for (size_t i = 0; i < mesh_.sizeFaces (); ++i)
  {
    VAFC circ = mesh_.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    polygon.vertices.clear ();
    do
    {
      polygon.vertices.push_back (circ.getTargetIndex ().get ());
    } while (++circ != circ_end);
    polygons.push_back (polygon);
  }

  PCL_INFO ("Meshing Finished!\n");
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::stepReconstruction (const long unsigned int id)
{
  HalfEdgeIndex half_edge = queue_.front ();
  queue_.pop_front ();

  AdvancingFrontData afront = getAdvancingFrontData (half_edge);

#ifdef AFRONTDEBUG
  std::printf ("\x1B[35mAdvancing Front: %lu\x1B[0m\n", id);

  // Remove previous fence from viewer
  for (long unsigned int j = 1; j <= fence_counter_; ++j)
    viewer_->removeShape ("fence" + static_cast<std::ostringstream *> (&(std::ostringstream () << j))->str (), 2);
  fence_counter_ = 0;

  // remove previouse iterations objects
  viewer_->removeShape ("HalfEdge");
  viewer_->removeShape ("NextHalfEdge", 1);
  viewer_->removeShape ("PrevHalfEdge", 1);
  viewer_->removeShape ("LeftSide", 1);
  viewer_->removeShape ("RightSide", 1);
  viewer_->removeShape ("Closest", 1);
  viewer_->removeShape ("ProxRadius", 1);
  viewer_->removePointCloud ("Mesh_Vertex_Cloud", 2);
  viewer_->removePointCloud ("Mesh_Vertex_Cloud_Normals", 4);
  viewer_->removeShape ("MLSSurface", 4);
  viewer_->removeShape ("MLSClosest", 4);
  viewer_->removeShape ("MLSRadius", 4);
  viewer_->removeShape ("MLSXAxis", 4);
  viewer_->removeShape ("MLSYAxis", 4);
  viewer_->removeShape ("MLSZAxis", 4);
  viewer_->removeShape ("MLSProjection", 4);

  pcl::PointXYZ p1, p2, p3, p4;
  p1 = pcl::PointXYZ (afront.next.tri.p[0] (0), afront.next.tri.p[0] (1), afront.next.tri.p[0] (2));
  p2 = pcl::PointXYZ (afront.next.tri.p[1] (0), afront.next.tri.p[1] (1), afront.next.tri.p[1] (2));
  p3 = pcl::PointXYZ (afront.next.tri.p[2] (0), afront.next.tri.p[2] (1), afront.next.tri.p[2] (2));
  p4 = pcl::PointXYZ (afront.next.tri.p[2] (0), afront.next.tri.p[2] (1), afront.next.tri.p[2] (2));

  viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 0, 255, 0, "HalfEdge");          // Green
  viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p2, p3, 255, 0, 0, "NextHalfEdge", 1);   // Red
  viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p4, 255, 0, 255, "PrevHalfEdge", 1); // Magenta
  viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "HalfEdge");
  viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "NextHalfEdge", 1);
  viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "PrevHalfEdge", 1);

  typename pcl::visualization::PointCloudColorHandlerCustom<typename MeshTraits::VertexData> single_color (mesh_vertex_data_ptr_, 255, 128, 0);
  viewer_->addPointCloud<typename MeshTraits::VertexData> (mesh_vertex_data_ptr_, single_color, "Mesh_Vertex_Cloud", 2);

  viewer_->addPointCloudNormals<typename MeshTraits::VertexData> (mesh_vertex_data_ptr_, 1, 0.005, "Mesh_Vertex_Cloud_Normals", 4);
#endif

  if (mesh_.getOppositeFaceIndex (afront.next.secondary) != mesh_.getOppositeFaceIndex (afront.prev.secondary) && afront.next.vi[2] == afront.prev.vi[2]) // This indicates a closed area
  {
#ifdef AFRONTDEBUG
    std::printf ("\x1B[34m  Closing Area\x1B[0m\n");
#endif
    typename MeshTraits::FaceData new_fd = createFaceData (afront.next.tri);
    // TODO: Need to check face normal to vertex normal. If not consistant to merge triangles together.
    //       This function has not been written yet. But it should look at the surrounding triangle and
    //       determine how to merge them.

    mesh_.addFace (afront.prev.vi[0], afront.prev.vi[1], afront.prev.vi[2], new_fd);
    removeFromQueue (afront.prev.secondary, afront.next.secondary);
    removeFromBoundary (afront.prev.secondary, afront.next.secondary);
  }
  else
  {
    PredictVertexResults pvr = predictVertex (afront);

#ifdef AFRONTDEBUG
    if (pvr.status == PredictVertexResults::Valid)
    {
      pcl::PolygonMesh mls_surface = getPolynomialSurface (pvr, search_radius_ / 50);
      viewer_->addPolygonMesh (mls_surface, "MLSSurface", 4);
    }

    Eigen::Vector3f cpt = (mls_cloud_->at (pvr.pv.closest)).getVector3fMap ();
    viewer_->addSphere (pcl::PointXYZ (cpt (0), cpt (1), cpt (2)), 0.1 * search_radius_, 255, 0, 0, "MLSClosest", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSClosest", 4);

    Eigen::Vector3f projected_pt = pvr.pv.point.getVector3fMap ();
    viewer_->addSphere (pcl::PointXYZ (cpt (0), cpt (1), cpt (2)), search_radius_, 0, 255, 128, "MLSRadius", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSRadius", 4);

    pcl::PointXYZ mls_mean = pcl::PointXYZ (pvr.pv.mls.mean (0), pvr.pv.mls.mean (1), pvr.pv.mls.mean (2));
    Eigen::Vector3d mls_xaxis = pvr.pv.mls.mean + 0.1 * search_radius_ * pvr.pv.mls.u_axis;
    Eigen::Vector3d mls_yaxis = pvr.pv.mls.mean + 0.1 * search_radius_ * pvr.pv.mls.v_axis;
    Eigen::Vector3d mls_zaxis = pvr.pv.mls.mean + 0.1 * search_radius_ * pvr.pv.mls.plane_normal;
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_xaxis (0), mls_xaxis (1), mls_xaxis (2)), 255, 0, 0, "MLSXAxis", 4);
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_yaxis (0), mls_yaxis (1), mls_yaxis (2)), 0, 255, 0, "MLSYAxis", 4);
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_zaxis (0), mls_zaxis (1), mls_zaxis (2)), 0, 0, 255, "MLSZAxis", 4);

    viewer_->addSphere (pcl::PointXYZ (projected_pt (0), projected_pt (1), projected_pt (2)), 0.02 * search_radius_, 255, 128, 0, "MLSProjection", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSProjection", 4);

    p3 = pcl::PointXYZ (pvr.tri.p[2] (0), pvr.tri.p[2] (1), pvr.tri.p[2] (2));
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p3, 0, 255, 0, "RightSide", 1); // Green
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p2, p3, 0, 255, 0, "LeftSide", 1);  // Green
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "RightSide", 1);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "LeftSide", 1);
#endif

    if (pvr.status == PredictVertexResults::InvalidMLSResults)
    {
#ifdef AFRONTDEBUG
      std::printf ("\x1B[31m  Invalid MLS Results for nearest point!\x1B[0m\n");
#endif
      boundary_.push_back (afront.front.he);
    }
    else if (pvr.status == PredictVertexResults::InvalidProjection)
    {
#ifdef AFRONTDEBUG
      std::printf ("\x1B[31m  Invalid Projection!\x1B[0m\n");
#endif
      boundary_.push_back (afront.front.he);
    }
    else if (pvr.status == PredictVertexResults::AtBoundary)
    {
#ifdef AFRONTDEBUG
      std::printf ("\x1B[36m  At Point Cloud Boundary!\x1B[0m\n");
#endif
      boundary_.push_back (afront.front.he);
    }
    else if (pvr.status == PredictVertexResults::InvalidStepSize)
    {
#ifdef AFRONTDEBUG
      std::printf ("\x1B[31m  Calculated Invalid Step Size!\x1B[0m\n");
#endif
      boundary_.push_back (afront.front.he);
    }
    else
    {
      TriangleToCloseResults ttcr = isTriangleToClose (pvr);
      if (!ttcr.found)
      {
        if (pvr.status == PredictVertexResults::Valid)
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[32m  Performed Grow Opperation\x1B[0m\n");
#endif
          grow (pvr);
        }
        else if (pvr.status == PredictVertexResults::InvalidVertexNormal)
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[31m  Projection point has inconsistant vertex normal!\x1B[0m\n");
#endif
          boundary_.push_back (afront.front.he);
        }
        else if (pvr.status == PredictVertexResults::InvalidTriangleNormal)
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[31m  Projection point triangle normal is inconsistant with vertex normals!\x1B[0m\n");
#endif
          boundary_.push_back (afront.front.he);
        }
      }
      else
      {
        if (!ttcr.tri.vertex_normals_valid)
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[31m  Closest point has inconsistant vertex normal!\x1B[0m\n");
#endif
          boundary_.push_back (afront.front.he);
        }
        else if (!ttcr.tri.triangle_normal_valid)
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[31m  Closest point triangle normal is inconsistant with vertex normals!\x1B[0m\n");
#endif
          boundary_.push_back (afront.front.he);
        }
        else
        {
#ifdef AFRONTDEBUG
          std::printf ("\x1B[33m  Performed Topology Event Opperation\x1B[0m\n");
#endif
          merge (ttcr);
        }
      }
    }
  }

  if (queue_.size () == 0 || (samples_ != 0 && id >= samples_))
    finished_ = true;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::createFirstTriangle (const double &x, const double &y, const double &z)
{
  std::vector<int> K;
  std::vector<float> K_dist;

  pcl::afront::AfrontGuidanceFieldPointType middle_pt;
  middle_pt.x = x;
  middle_pt.y = y;
  middle_pt.z = z;
  mls_cloud_tree_->nearestKSearch (middle_pt, 1, K, K_dist);
  createFirstTriangle (K[0]);
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::createFirstTriangle (const int &index)
{
  SamplePointResults sp1 = samplePoint (mls_cloud_->points[index]);
  Eigen::Vector3f p1 = sp1.point.getVector3fMap ();

  // Get the allowed grow distance and control the first triangle size
  sp1.point.max_step_search_radius = search_radius_;
  sp1.point.max_step = getMaxStep (p1, sp1.point.max_step_search_radius);

  // search for the nearest neighbor
  std::vector<int> K;
  std::vector<float> K_dist;
  mls_cloud_tree_->nearestKSearch (pcl::afront::AfrontGuidanceFieldPointType (sp1.point, rho_), 2, K, K_dist);

  // use l1 and nearest neighbor to extend edge
  pcl::afront::AfrontGuidanceFieldPointType dp;
  SamplePointResults sp2, sp3;
  Eigen::Vector3f p2, p3, v1, v2, mp, norm, proj;

  dp = mls_cloud_->points[K[1]];
  v1 = dp.getVector3fMap () - p1;
  v1 = v1.normalized ();
  norm = dp.getNormalVector3fMap ();

  proj = p1 + sp1.point.max_step * v1;
  sp2 = samplePoint (proj (0), proj (1), proj (2));
  p2 = sp2.point.getVector3fMap ();
  sp2.point.max_step_search_radius = sp1.point.max_step_search_radius + sp1.point.max_step;
  sp2.point.max_step = getMaxStep (p2, sp2.point.max_step_search_radius);

  mp = (p1 + p2) / 2.0;
  double d = (p2 - p1).norm ();
  max_edge_length_ = (p2 - p1).norm ();

  v2 = norm.cross (v1).normalized ();

  double max_step = std::min (sp1.point.max_step, sp2.point.max_step);
  double l = std::sqrt (pow (max_step, 2.0) - pow (d / 2.0, 2.0)); // Calculate the height of the triangle
  proj = mp + l * v2;
  sp3 = samplePoint (proj (0), proj (1), proj (2));
  p3 = sp3.point.getVector3fMap ();
  sp3.point.max_step_search_radius = std::max (sp1.point.max_step_search_radius, sp2.point.max_step_search_radius) + max_step;
  sp3.point.max_step = getMaxStep (p3, sp3.point.max_step_search_radius);

  d = (p1 - p3).norm ();
  if (d > max_edge_length_)
    max_edge_length_ = d;

  d = (p2 - p3).norm ();
  if (d > max_edge_length_)
    max_edge_length_ = d;

  // Align normals
  pcl::alignNormals (sp2.point.getNormalVector3fMap (), sp1.point.getNormalVector3fMap ());
  pcl::alignNormals (sp3.point.getNormalVector3fMap (), sp1.point.getNormalVector3fMap ());

  typename MeshTraits::FaceData fd;
  Eigen::Vector3f center = (p1 + p2 + p3) / 3;
  Eigen::Vector3f normal = ((p2 - p1).cross (p3 - p1)).normalized ();
  pcl::alignNormals (normal, sp1.point.getNormalVector3fMap ());

  fd.x = center (0);
  fd.y = center (1);
  fd.z = center (2);
  fd.normal_x = normal (0);
  fd.normal_y = normal (1);
  fd.normal_z = normal (2);

  VertexIndices vi;
  vi.push_back (mesh_.addVertex (sp1.point));
  vi.push_back (mesh_.addVertex (sp2.point));
  vi.push_back (mesh_.addVertex (sp3.point));

  FaceIndex fi = mesh_.addFace (vi[0], vi[1], vi[2], fd);

  mesh_octree_->addPointFromCloud (0, mesh_vertex_data_indices_);
  mesh_octree_->addPointFromCloud (1, mesh_vertex_data_indices_);
  mesh_octree_->addPointFromCloud (2, mesh_vertex_data_indices_);

  addToQueue (fi);
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::cutEar (const CutEarData &ccer)
{
  assert (ccer.tri.point_valid);
  if (ccer.tri.B > max_edge_length_)
    max_edge_length_ = ccer.tri.B;

  if (ccer.tri.C > max_edge_length_)
    max_edge_length_ = ccer.tri.C;

  typename MeshTraits::FaceData new_fd = createFaceData (ccer.tri);
  FaceIndex fi = mesh_.addFace (ccer.vi[0], ccer.vi[1], ccer.vi[2], new_fd);

  if (addToQueue (fi))
  {
    removeFromQueue (ccer.secondary);
    removeFromBoundary (ccer.secondary);
  }
  else
  {
    addToBoundary (ccer.primary);
  }
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::AdvancingFrontData
pcl::AfrontMesher<PointNT>::getAdvancingFrontData (const HalfEdgeIndex &half_edge) const
{
  AdvancingFrontData result;
  result.front.he = half_edge;

  FaceIndex face_indx = mesh_.getOppositeFaceIndex (half_edge);
  typename MeshTraits::FaceData fd = mesh_.getFaceDataCloud ()[face_indx.get ()];

  // Get Half Edge Vertexs
  result.front.vi[0] = mesh_.getOriginatingVertexIndex (half_edge);
  result.front.vi[1] = mesh_.getTerminatingVertexIndex (half_edge);

  typename MeshTraits::VertexData p1, p2;
  p1 = mesh_vertex_data_ptr_->at (result.front.vi[0].get ());
  p2 = mesh_vertex_data_ptr_->at (result.front.vi[1].get ());

  result.front.p[0] = p1.getVector3fMap ();
  result.front.p[1] = p2.getVector3fMap ();
  result.front.n[0] = p1.getNormalVector3fMap ();
  result.front.n[1] = p2.getNormalVector3fMap ();

  // Calculate the half edge length
  result.front.length = (result.front.p[0] - result.front.p[1]).norm ();

  // Get half edge midpoint
  result.front.mp = (result.front.p[0] + result.front.p[1]) / 2.0;

  // Calculate the grow direction vector
  result.front.d = getGrowDirection (result.front.p[0], result.front.mp, fd);

  // Get the maximum grow distance
  result.front.max_step = std::min (p1.max_step, p2.max_step);

  // Get the approximate search radius for the calculating the max step for the grow operation.
  result.front.max_step_search_radius = std::max (p1.max_step_search_radius, p2.max_step_search_radius) + result.front.max_step;

  // Get Next Half Edge
  result.next = getNextHalfEdge (result.front);

  // Get Prev Half Edge
  result.prev = getPrevHalfEdge (result.front);

  return result;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::SamplePointResults
pcl::AfrontMesher<PointNT>::samplePoint (const pcl::afront::AfrontGuidanceFieldPointType &pt) const
{
  if (!pcl_isfinite (pt.x))
    PCL_ERROR ("MLS Sample point is not finite\n");

  SamplePointResults result;
  result.orig = pcl::PointXYZ (pt.x, pt.y, pt.z);

  // Get 3D position of point
  //Eigen::Vector3f pos = distinct_cloud_->points[dp_i].getVector3fMap ();
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  mls_cloud_tree_->nearestKSearch (pt, 1, nn_indices, nn_dists);
  result.closest = nn_indices.front ();
  result.dist = nn_dists.front ();

  // If the closest point did not have a valid MLS fitting result
  // OR if it is too far away from the sampled point
  int index = mls_corresponding_input_indices_->indices[result.closest];
  const pcl::MLSResult &mls_result = mls_.getMLSResults ()[index];
  if (mls_result.valid == false)
    std::printf ("\x1B[31m\tMLS Results Not Valid!\n");

  Eigen::Vector3d add_point = pt.getVector3fMap ().template cast<double> ();

  result.mls = mls_result;
  pcl::MLSResult::MLSProjectionResults proj_result;
  if (mls_result.num_neighbors >= required_neighbors_)
  {
    proj_result = mls_result.projectPoint (add_point, pcl::MLSResult::ORTHOGONAL);
    result.point.curvature = mls_result.calculatePrincipleCurvatures (proj_result.u, proj_result.v).cwiseAbs ().maxCoeff ();
  }
  else
  {
    double u, v;
    mls_result.getMLSCoordinates (add_point, u, v);
    proj_result = mls_result.projectPointToMLSPlane (u, v);
    result.point.curvature = 1e-8;
  }

  result.point.x = static_cast<float> (proj_result.point[0]);
  result.point.y = static_cast<float> (proj_result.point[1]);
  result.point.z = static_cast<float> (proj_result.point[2]);

  result.point.normal_x = static_cast<float> (proj_result.normal[0]);
  result.point.normal_y = static_cast<float> (proj_result.normal[1]);
  result.point.normal_z = static_cast<float> (proj_result.normal[2]);

  return result;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::SamplePointResults
pcl::AfrontMesher<PointNT>::samplePoint (float x, float y, float z) const
{
  pcl::afront::AfrontGuidanceFieldPointType search_pt;
  search_pt.x = x;
  search_pt.y = y;
  search_pt.z = z;

  return samplePoint (search_pt);
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::CutEarData
pcl::AfrontMesher<PointNT>::getNextHalfEdge (const FrontData &front) const
{
  CutEarData next;
  next.primary = front.he;
  next.secondary = mesh_.getNextHalfEdgeIndex (front.he);
  next.vi[0] = front.vi[0];
  next.vi[1] = front.vi[1];
  next.vi[2] = mesh_.getTerminatingVertexIndex (next.secondary);
  next.tri = getTriangleData (front, mesh_vertex_data_ptr_->at (next.vi[2].get ()));

  OHEAVC circ_next = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (front.vi[1]);
  const OHEAVC circ_next_end = circ_next;
  do
  {
    HalfEdgeIndex he = circ_next.getTargetIndex ();
    if (!mesh_.isValid (he))
      continue;

    VertexIndex evi = mesh_.getTerminatingVertexIndex (he);
    if (!mesh_.isBoundary (he))
    {
      he = mesh_.getOppositeHalfEdgeIndex (he);
      if (!mesh_.isBoundary (he))
        continue;
    }

    if (he == front.he)
      continue;

    TriangleData tri = getTriangleData (front, mesh_vertex_data_ptr_->at (evi.get ()));
    if ((tri.point_valid && !next.tri.point_valid) || (tri.point_valid && next.tri.point_valid && tri.c < next.tri.c))
    {
      next.secondary = he;
      next.vi[0] = front.vi[0];
      next.vi[1] = front.vi[1];
      next.vi[2] = evi;
      next.tri = tri;
    }
  } while (++circ_next != circ_next_end);

  return next;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::CutEarData
pcl::AfrontMesher<PointNT>::getPrevHalfEdge (const FrontData &front) const
{
  CutEarData prev;
  prev.primary = front.he;
  prev.secondary = mesh_.getPrevHalfEdgeIndex (front.he);
  prev.vi[0] = front.vi[0];
  prev.vi[1] = front.vi[1];
  prev.vi[2] = mesh_.getOriginatingVertexIndex (prev.secondary);
  prev.tri = getTriangleData (front, mesh_vertex_data_ptr_->at (prev.vi[2].get ()));

  OHEAVC circ_prev = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (front.vi[0]);
  const OHEAVC circ_prev_end = circ_prev;
  do
  {
    HalfEdgeIndex he = circ_prev.getTargetIndex ();
    if (!mesh_.isValid (he))
      continue;

    VertexIndex evi = mesh_.getTerminatingVertexIndex (he);
    if (!mesh_.isBoundary (he))
    {
      he = mesh_.getOppositeHalfEdgeIndex (he);
      if (!mesh_.isBoundary (he))
        continue;
    }

    if (he == front.he)
      continue;

    TriangleData tri = getTriangleData (front, mesh_vertex_data_ptr_->at (evi.get ()));

    if ((tri.point_valid && !prev.tri.point_valid) || (tri.point_valid && prev.tri.point_valid && tri.b < prev.tri.b))
    {
      prev.secondary = he;
      prev.vi[0] = front.vi[0];
      prev.vi[1] = front.vi[1];
      prev.vi[2] = evi;
      prev.tri = tri;
    }
  } while (++circ_prev != circ_prev_end);

  return prev;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::PredictVertexResults
pcl::AfrontMesher<PointNT>::predictVertex (const AdvancingFrontData &afront) const
{
  // Local Variables
  PredictVertexResults result;
  const FrontData &front = afront.front;

  result.status = PredictVertexResults::Valid;
  result.afront = afront;

  // Get new point for growing a triangle to be projected onto the mls surface
  double l = std::sqrt (pow (front.max_step, 2.0) - pow (front.length / 2.0, 2.0)); // Calculate the height of the triangle

  // This is required because 2 * step size < front.length results in nan
  if (!std::isnan (l))
  {
    // Get predicted vertex
    Eigen::Vector3f p = front.mp + l * front.d;
    result.pv = samplePoint (p (0), p (1), p (2));
    pcl::alignNormals (result.pv.point.getNormalVector3fMap (), mesh_vertex_data_ptr_->at (afront.front.vi[0].get ()).getNormalVector3fMap ());
    if (result.pv.mls.num_neighbors < required_neighbors_) // Maybe we should use the 5 * DOF that PCL uses
    {
      result.status = PredictVertexResults::InvalidMLSResults;
      return result;
    }

    // Check if the projected point is to far from the original point.
    double dist = (result.pv.orig.getVector3fMap () - result.pv.point.getVector3fMap ()).norm ();
    if (dist > front.max_step)
    {
      result.status = PredictVertexResults::InvalidProjection;
      return result;
    }

    // Check and see if there any point in the grow direction of the front.
    // If not then it is at the boundary of the point cloud.
    if (nearBoundary (front, result.pv.closest))
    {
      result.status = PredictVertexResults::AtBoundary;
      return result;
    }

    // Get triangle Data
    result.tri = getTriangleData (front, result.pv.point);
    if (!result.tri.point_valid)
      result.status = PredictVertexResults::InvalidProjection;
    else if (!result.tri.vertex_normals_valid)
      result.status = PredictVertexResults::InvalidVertexNormal;
    else if (!result.tri.triangle_normal_valid)
      result.status = PredictVertexResults::InvalidTriangleNormal;

    return result;
  }
  else
  {
    result.status = PredictVertexResults::InvalidStepSize;
    return result;
  }
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::CloseProximityResults
pcl::AfrontMesher<PointNT>::isCloseProximity (const PredictVertexResults &pvr) const
{
  CloseProximityResults results;
  std::vector<int> K;
  std::vector<float> K_dist;

  mesh_octree_->radiusSearch (pvr.pv.point, pvr.afront.front.max_step, K, K_dist);

  results.fences.reserve (K.size ());
  results.verticies.reserve (K.size ());
  results.found = false;

  // First check for closest proximity violation
  for (int i = 0; i < K.size (); ++i)
  {
    typename MeshTraits::VertexData &data = mesh_vertex_data_ptr_->at (K[i]);
    VertexIndex vi = mesh_.getVertexIndex (data);

    // Don't include the front vertices
    if (vi == pvr.afront.front.vi[0] || vi == pvr.afront.front.vi[1])
      continue;

    if (mesh_.isBoundary (vi))
    {
      typename MeshTraits::VertexData chkpn = mesh_vertex_data_ptr_->at (vi.get ());
      Eigen::Vector3f chkpt = chkpn.getVector3fMap ();
      bool chkpt_valid = isPointValid (pvr.afront.front, chkpt);

      OHEAVC circ = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (vi);
      const OHEAVC circ_end = circ;
      do
      {
        HalfEdgeIndex he = circ.getTargetIndex ();
        VertexIndex evi = mesh_.getTerminatingVertexIndex (he);
        if (!mesh_.isBoundary (he))
        {
          he = mesh_.getOppositeHalfEdgeIndex (he);
          if (!mesh_.isBoundary (he))
            continue;
        }

        // Don't include any edge attached to the half edge.
        if (evi == pvr.afront.front.vi[0] || evi == pvr.afront.front.vi[1])
          continue;

        Eigen::Vector3f endpt = mesh_vertex_data_ptr_->at (evi.get ()).getVector3fMap ();
        bool endpt_valid = isPointValid (pvr.afront.front, endpt);
        if (chkpt_valid || endpt_valid) // If either vertex is valid add as a valid fence check.
        {
          if (std::find (results.fences.begin (), results.fences.end (), he) == results.fences.end ())
          {
            results.fences.push_back (he);

#ifdef AFRONTDEBUG
            fence_counter_ += 1;
            std::string fence_name = "fence" + static_cast<std::ostringstream *> (&(std::ostringstream () << fence_counter_))->str ();
            viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (pcl::PointXYZ (chkpt (0), chkpt (1), chkpt (2)), pcl::PointXYZ (endpt (0), endpt (1), endpt (2)), 0, 255, 255, fence_name, 2); // orange
            viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, fence_name, 2);
#endif
          }
        }

      } while (++circ != circ_end);

      if (!chkpt_valid)
        continue;

      double dist = (pvr.tri.p[2] - chkpt).norm ();
      results.verticies.push_back (vi);
      if (dist < 0.5 * pvr.afront.front.max_step)
      {
        TriangleData tri = getTriangleData (pvr.afront.front, chkpn);
        if (!results.found)
        {
          results.found = true;
          results.dist = dist;
          results.closest = vi;
          results.tri = tri;
        }
        else
        {
          if ((results.tri.valid && tri.valid && dist < results.dist) ||
              (!results.tri.valid && !tri.valid && dist < results.dist) ||
              (!results.tri.valid && tri.valid))
          {
            results.dist = dist;
            results.closest = vi;
            results.tri = tri;
          }
        }
      }
    }
  }

  // If either the prev or next half edge vertice is with in tolerance default to it
  // over closest point.
  // TODO: Should we be checking if triangle normals are valid?
  double prev_dist = pcl::afront::distPoint2Line (pvr.afront.prev.tri.p[0], pvr.afront.prev.tri.p[2], pvr.tri.p[2]).d;
  double next_dist = pcl::afront::distPoint2Line (pvr.afront.next.tri.p[1], pvr.afront.next.tri.p[2], pvr.tri.p[2]).d;
  if (pvr.afront.prev.tri.point_valid && pvr.afront.next.tri.point_valid)
  {
    if (pvr.afront.prev.tri.aspect_ratio >= AFRONT_ASPECT_RATIO_TOLERANCE || pvr.afront.next.tri.aspect_ratio >= AFRONT_ASPECT_RATIO_TOLERANCE)
    {
      if (pvr.afront.prev.tri.aspect_ratio > pvr.afront.next.tri.aspect_ratio)
      {
        results.found = true;
        results.dist = prev_dist;
        results.closest = pvr.afront.prev.vi[2];
        results.tri = pvr.afront.prev.tri;
      }
      else
      {
        results.found = true;
        results.dist = next_dist;
        results.closest = pvr.afront.next.vi[2];
        results.tri = pvr.afront.next.tri;
      }
    }
    else
    {
      if (prev_dist < next_dist)
      {
        if (prev_dist < AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step)
        {
          results.found = true;
          results.dist = prev_dist;
          results.closest = pvr.afront.prev.vi[2];
          results.tri = pvr.afront.prev.tri;
        }
      }
      else
      {
        if (next_dist < AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step)
        {
          results.found = true;
          results.dist = next_dist;
          results.closest = pvr.afront.next.vi[2];
          results.tri = pvr.afront.next.tri;
        }
      }
    }
  }
  else if ((pvr.afront.prev.tri.point_valid && prev_dist < AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step) || (pvr.afront.prev.tri.point_valid && pvr.afront.prev.tri.aspect_ratio >= AFRONT_ASPECT_RATIO_TOLERANCE))
  {
    results.found = true;
    results.dist = prev_dist;
    results.closest = pvr.afront.prev.vi[2];
    results.tri = pvr.afront.prev.tri;
  }
  else if ((pvr.afront.next.tri.point_valid && next_dist < AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step) || (pvr.afront.next.tri.point_valid && pvr.afront.next.tri.aspect_ratio >= AFRONT_ASPECT_RATIO_TOLERANCE))
  {
    results.found = true;
    results.dist = next_dist;
    results.closest = pvr.afront.next.vi[2];
    results.tri = pvr.afront.next.tri;
  }

  // If nothing was found check and make sure new vertex is not close to a fence
  if (!results.found)
  {
    for (int i = 0; i < results.fences.size (); ++i)
    {
      HalfEdgeIndex he = results.fences[i];

      VertexIndex vi[2];
      vi[0] = mesh_.getOriginatingVertexIndex (he);
      vi[1] = mesh_.getTerminatingVertexIndex (he);

      // It is not suffecient to just compare half edge indexs because non manifold mesh is allowed.
      // Must check vertex index
      if (vi[0] == pvr.afront.front.vi[0] || vi[1] == pvr.afront.front.vi[0] || vi[0] == pvr.afront.front.vi[1] || vi[1] == pvr.afront.front.vi[1])
        continue;

      Eigen::Vector3f p1 = mesh_vertex_data_ptr_->at (vi[0].get ()).getVector3fMap ();
      Eigen::Vector3f p2 = mesh_vertex_data_ptr_->at (vi[1].get ()).getVector3fMap ();

      pcl::afront::DistPoint2LineResults dist = pcl::afront::distPoint2Line (p1, p2, pvr.tri.p[2]);
      if (dist.d < AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step)
      {

        bool check_p1 = isPointValid (pvr.afront.front, p1);
        bool check_p2 = isPointValid (pvr.afront.front, p2);
        int index;
        if (check_p1 && check_p2)
        {
          // TODO: Should check if triangle is valid for these points. If both are then check distance
          //       otherwise use the one that creates a valid triangle.
          if ((p1 - pvr.tri.p[2]).norm () < (p2 - pvr.tri.p[2]).norm ())
            index = 0;
          else
            index = 1;
        }
        else if (check_p1)
        {
          index = 0;
        }
        else
        {
          index = 1;
        }

        if (results.found && dist.d < results.dist)
        {
          results.found = true;
          results.dist = dist.d;
          results.closest = vi[index];
          results.tri = getTriangleData (pvr.afront.front, mesh_vertex_data_ptr_->at (results.closest.get ()));
        }
        else
        {
          results.found = true;
          results.dist = dist.d;
          results.closest = vi[index];
          results.tri = getTriangleData (pvr.afront.front, mesh_vertex_data_ptr_->at (results.closest.get ()));
        }
      }
    }
  }

#ifdef AFRONTDEBUG
  Eigen::Vector3f p;
  p = pvr.tri.p[2];
  viewer_->addSphere (pcl::PointXYZ (p (0), p (1), p (2)), AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step, 0, 255, 128, "ProxRadius", 1);
  viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "ProxRadius", 1);
#endif

  return results;
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::checkPrevNextHalfEdge (const AdvancingFrontData &afront, TriangleData &tri, VertexIndex &closest) const
{
  if (mesh_.isValid (closest) && closest == afront.prev.vi[2])
    if (afront.next.tri.point_valid && tri.c >= afront.next.tri.c)
    {
      closest = afront.next.vi[2];
      tri = afront.next.tri;
      return true;
    }

  if (mesh_.isValid (closest) && closest == afront.next.vi[2])
    if (afront.prev.tri.point_valid && tri.b >= afront.prev.tri.b)
    {
      closest = afront.prev.vi[2];
      tri = afront.prev.tri;
      return true;
    }

  if ((afront.next.tri.point_valid && tri.c >= afront.next.tri.c) && (afront.prev.tri.point_valid && tri.b >= afront.prev.tri.b))
  {
    if (afront.next.tri.c < afront.prev.tri.b)
    {
      closest = afront.next.vi[2];
      tri = afront.next.tri;
      return true;
    }
    else
    {
      closest = afront.prev.vi[2];
      tri = afront.prev.tri;
      return true;
    }
  }
  else if (afront.next.tri.point_valid && tri.c >= afront.next.tri.c)
  {
    closest = afront.next.vi[2];
    tri = afront.next.tri;
    return true;
  }
  else if (afront.prev.tri.point_valid && tri.b >= afront.prev.tri.b)
  {
    closest = afront.prev.vi[2];
    tri = afront.prev.tri;
    return true;
  }

  pcl::afront::IntersectionLine2PlaneResults lpr;
  if (afront.next.tri.point_valid && isFenceViolated (afront.front.p[0], tri.p[2], afront.next.secondary, AFRONT_FENCE_HEIGHT_FACTOR * afront.next.tri.B * hausdorff_error_, lpr))
  {
    closest = afront.next.vi[2];
    tri = afront.next.tri;
    return true;
  }

  if (afront.prev.tri.point_valid && isFenceViolated (afront.front.p[1], tri.p[2], afront.prev.secondary, AFRONT_FENCE_HEIGHT_FACTOR * afront.prev.tri.C * hausdorff_error_, lpr))
  {
    closest = afront.prev.vi[2];
    tri = afront.prev.tri;
    return true;
  }

  return false;
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::isFenceViolated (const Eigen::Vector3f &sp, const Eigen::Vector3f &ep, const HalfEdgeIndex &fence, const double fence_height, pcl::afront::IntersectionLine2PlaneResults &lpr) const
{
  // Check for fence intersection
  Eigen::Vector3f he_p1, he_p2;
  he_p1 = (mesh_vertex_data_ptr_->at (mesh_.getOriginatingVertexIndex (fence).get ())).getVector3fMap ();
  he_p2 = (mesh_vertex_data_ptr_->at (mesh_.getTerminatingVertexIndex (fence).get ())).getVector3fMap ();

  typename MeshTraits::FaceData fd = mesh_.getFaceDataCloud ()[mesh_.getOppositeFaceIndex (fence).get ()];

  Eigen::Vector3f u = he_p2 - he_p1;
  Eigen::Vector3f n = fd.getNormalVector3fMap ();
  Eigen::Vector3f v = n * fence_height;

  // Project the line on to the triangle plane. Note this is different from the paper
  Eigen::Vector3f sp_proj = sp - (sp - he_p1).dot (n) * n;
  Eigen::Vector3f ep_proj = ep - (ep - he_p1).dot (n) * n;
  lpr = pcl::afront::intersectionLine2Plane (sp_proj, ep_proj, he_p1, u, v);

  if (!lpr.parallel)                     // May need to add additional check if parallel
    if (lpr.mw <= 1 && lpr.mw >= 0)      // This checks if line segement intersects the plane
      if (lpr.mu <= 1 && lpr.mu >= 0)    // This checks if intersection point is within the x range of the plane
        if (lpr.mv <= 1 && lpr.mv >= -1) // This checks if intersection point is within the y range of the plane
          return true;

  return false;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::FenceViolationResults
pcl::AfrontMesher<PointNT>::isFencesViolated (const VertexIndex &vi, const Eigen::Vector3f &p, const std::vector<HalfEdgeIndex> &fences, const VertexIndex &closest, const PredictVertexResults &pvr) const
{
  // Now need to check for fence violation
  FenceViolationResults results;
  results.found = false;
  Eigen::Vector3f sp = mesh_vertex_data_ptr_->at (vi.get ()).getVector3fMap ();

  for (int i = 0; i < fences.size (); ++i)
  {
    // The list of fences should not include any that are associated to the requesting vi
    assert (vi != mesh_.getOriginatingVertexIndex (fences[i]));
    assert (vi != mesh_.getTerminatingVertexIndex (fences[i]));

    // Need to ignore fence check if closest point is associated to the fence half edge
    if (mesh_.isValid (closest))
      if (closest == mesh_.getOriginatingVertexIndex (fences[i]) || closest == mesh_.getTerminatingVertexIndex (fences[i]))
        continue;

    pcl::afront::IntersectionLine2PlaneResults lpr;
    double fence_height = 2.0 * pvr.afront.front.max_step * hausdorff_error_;
    bool fence_violated = isFenceViolated (sp, p, fences[i], fence_height, lpr);

    if (fence_violated)
    {
      double dist = (lpr.mw * lpr.w).dot (pvr.afront.front.d);
      if (!results.found)
      {
        results.he = fences[i];
        results.index = i;
        results.lpr = lpr;
        results.dist = dist;
        results.found = true;
      }
      else
      {
        if (dist < results.dist)
        {
          results.he = fences[i];
          results.index = i;
          results.lpr = lpr;
          results.dist = dist;
        }
      }
    }
  }

  return results;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::TriangleToCloseResults
pcl::AfrontMesher<PointNT>::isTriangleToClose (const PredictVertexResults &pvr) const
{
  TriangleToCloseResults results;
  results.pvr = pvr;

  // Check if new vertex is in close proximity to exising mesh verticies
  CloseProximityResults cpr = isCloseProximity (pvr);

  results.found = cpr.found;
  results.tri = pvr.tri;
  if (results.found)
  {
    results.closest = cpr.closest;
    results.tri = cpr.tri;
  }

  // Check if the proposed triangle interfers with the prev or next half edge
  if (checkPrevNextHalfEdge (pvr.afront, results.tri, results.closest))
    results.found = true;

  // Check if any fences are violated.
  FenceViolationResults fvr;
  for (int i = 0; i < cpr.fences.size (); ++i)
  {
    if (results.found && results.closest == pvr.afront.prev.vi[2])
    {
      fvr = isFencesViolated (pvr.afront.prev.vi[1], pvr.afront.prev.tri.p[2], cpr.fences, results.closest, pvr);
    }
    else if (results.found && results.closest == pvr.afront.next.vi[2])
    {
      fvr = isFencesViolated (pvr.afront.prev.vi[0], pvr.afront.next.tri.p[2], cpr.fences, results.closest, pvr);
    }
    else
    {
      FenceViolationResults fvr1, fvr2, min_fvr;
      fvr1 = isFencesViolated (pvr.afront.front.vi[0], results.tri.p[2], cpr.fences, results.closest, pvr);
      fvr2 = isFencesViolated (pvr.afront.front.vi[1], results.tri.p[2], cpr.fences, results.closest, pvr);
      if (fvr1.found && fvr2.found)
      {
        fvr = fvr1;
        if (fvr2.dist < min_fvr.dist)
          fvr = fvr2;
      }
      else if (fvr1.found)
      {
        fvr = fvr1;
      }
      else
      {
        fvr = fvr2;
      }
    }

    if (fvr.found)
    {
      results.found = true;

#ifdef AFRONTDEBUG
      assert (fence_counter_ != 0);
      std::string fence_name = "fence" + static_cast<std::ostringstream *> (&(std::ostringstream () << (fvr.index + 1)))->str ();
      viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 140, 0, fence_name, 1);
#endif

      VertexIndex fvi[2];
      Eigen::Vector3f fp[2];
      fvi[0] = mesh_.getOriginatingVertexIndex (fvr.he);
      fvi[1] = mesh_.getTerminatingVertexIndex (fvr.he);
      fp[0] = (mesh_vertex_data_ptr_->at (fvi[0].get ())).getVector3fMap ();
      fp[1] = (mesh_vertex_data_ptr_->at (fvi[1].get ())).getVector3fMap ();

      bool vp1 = isPointValid (pvr.afront.front, fp[0]);
      bool vp2 = isPointValid (pvr.afront.front, fp[1]);
      assert (vp1 || vp2);
      int index;
      if (vp1 && vp2)
      {
        // TODO: Should check if triangle is valid for these points. If both are then check distance
        //       otherwise use the one that creates a valid triangle.
        double dist1 = pcl::afront::distPoint2Line (pvr.afront.front.p[0], pvr.afront.front.p[1], fp[0]).d;
        double dist2 = pcl::afront::distPoint2Line (pvr.afront.front.p[0], pvr.afront.front.p[1], fp[1]).d;
        if (dist1 < dist2)
          index = 0;
        else
          index = 1;
      }
      else if (vp1)
        index = 0;
      else
        index = 1;

      results.closest = fvi[index];
      results.tri = getTriangleData (pvr.afront.front, mesh_vertex_data_ptr_->at (results.closest.get ()));

      checkPrevNextHalfEdge (pvr.afront, results.tri, results.closest);
    }
    else
    {
#ifdef AFRONTDEBUG
      Eigen::Vector3f p = pvr.tri.p[2];
      if (results.found)
        p = results.tri.p[2];

      viewer_->addSphere (pcl::PointXYZ (p (0), p (1), p (2)), 0.1 * pvr.afront.front.max_step, 255, 255, 0, "Closest", 1);
#endif
      return results;
    }
  }

  // Need to print a warning message it should never get here.
  return results;
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::isPointValid (const FrontData &front, const Eigen::Vector3f p) const
{
  Eigen::Vector3f v = p - front.mp;
  double dot = v.dot (front.d);

  if ((dot > 0.0))
    return true;

  return false;
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::isBoundaryPoint (const int index) const
{
  pcl::afront::AfrontGuidanceFieldPointType closest = mls_cloud_->at (index);

  Eigen::Vector4f u;
  Eigen::Vector4f v;
  std::vector<int> K;
  std::vector<float> K_dist;

  const pcl::MLSResult &mls_result = mls_.getMLSResults ()[index];
  u << mls_result.u_axis.cast<float> ().array (), 0.0;
  v << mls_result.v_axis.cast<float> ().array (), 0.0;

  // Need to modify mls library to store indicies instead of just the number of neighbors
  mls_cloud_tree_->radiusSearch (closest, search_radius_, K, K_dist);

  if (K.size () < 3)
    return (false);

  if (!pcl_isfinite (closest.x) || !pcl_isfinite (closest.y) || !pcl_isfinite (closest.z))
    return (false);

  // Compute the angles between each neighboring point and the query point itself
  std::vector<float> angles (K.size ());
  float max_dif = FLT_MIN, dif;
  int cp = 0;

  for (size_t i = 0; i < K.size (); ++i)
  {
    if (!pcl_isfinite (mls_cloud_->points[K[i]].x) ||
        !pcl_isfinite (mls_cloud_->points[K[i]].y) ||
        !pcl_isfinite (mls_cloud_->points[K[i]].z))
      continue;

    Eigen::Vector4f delta = mls_cloud_->points[K[i]].getVector4fMap () - closest.getVector4fMap ();
    if (delta == Eigen::Vector4f::Zero ())
      continue;

    angles[cp++] = atan2f (v.dot (delta), u.dot (delta)); // the angles are fine between -PI and PI too
  }
  if (cp == 0)
    return (false);

  angles.resize (cp);
  std::sort (angles.begin (), angles.end ());

  // Compute the maximal angle difference between two consecutive angles
  for (size_t i = 0; i < angles.size () - 1; ++i)
  {
    dif = angles[i + 1] - angles[i];
    if (max_dif < dif)
      max_dif = dif;
  }

  // Get the angle difference between the last and the first
  dif = 2 * static_cast<float> (M_PI) - angles[angles.size () - 1] + angles[0];
  if (max_dif < dif)
    max_dif = dif;

  // Check results
  if (max_dif > boundary_angle_threshold_)
    return (true);
  else
    return (false);
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::nearBoundary (const FrontData &front, const int index) const
{
  pcl::afront::AfrontGuidanceFieldPointType closest = mls_cloud_->at (index);

  Eigen::Vector3f v1 = (front.p[1] - front.mp).normalized ();
  Eigen::Vector3f v2 = closest.getVector3fMap () - front.mp;
  double dot1 = v2.dot (front.d);
  double dot2 = v2.dot (v1);

  if ((dot1 > 0.0) && (std::abs (dot2) <= front.length / 2.0))
    return false;

  // Try the pcl boundary search only if the simple check says it is a boundary
  return isBoundaryPoint (index);

  return true;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::grow (const PredictVertexResults &pvr)
{
  // Add new face
  typename MeshTraits::FaceData new_fd = createFaceData (pvr.tri);
  typename MeshTraits::VertexData p = pvr.pv.point;
  p.max_step_search_radius = pvr.afront.front.max_step_search_radius;
  p.max_step = getMaxStep (pvr.tri.p[2], p.max_step_search_radius);
  FaceIndex fi = mesh_.addFace (pvr.afront.front.vi[0], pvr.afront.front.vi[1], mesh_.addVertex (p), new_fd);
  mesh_octree_->addPointFromCloud (mesh_vertex_data_ptr_->size () - 1, mesh_vertex_data_indices_);

  if (pvr.tri.B > max_edge_length_)
    max_edge_length_ = pvr.tri.B;

  if (pvr.tri.C > max_edge_length_)
    max_edge_length_ = pvr.tri.C;

  // Add new half edges to the queue
  if (!addToQueue (fi))
    addToBoundary (pvr.afront.front.he);
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::merge (const TriangleToCloseResults &ttcr)
{
  assert (ttcr.closest != ttcr.pvr.afront.front.vi[0]);
  assert (ttcr.closest != ttcr.pvr.afront.front.vi[1]);
  // Need to make sure at lease one vertex of the half edge is in the grow direction
  if (ttcr.closest == ttcr.pvr.afront.prev.vi[2])
  {
#ifdef AFRONTDEBUG
    std::printf ("\x1B[32m\tAborting Merge, Forced Ear Cut Opperation with Previous Half Edge!\x1B[0m\n");
#endif
    cutEar (ttcr.pvr.afront.prev);
    return;
  }
  else if (ttcr.closest == ttcr.pvr.afront.next.vi[2])
  {
#ifdef AFRONTDEBUG
    std::printf ("\x1B[32m\tAborting Merge, Forced Ear Cut Opperation with Next Half Edge!\x1B[0m\n");
#endif
    cutEar (ttcr.pvr.afront.next);
    return;
  }

  if (ttcr.tri.B > max_edge_length_)
    max_edge_length_ = ttcr.tri.B;

  if (ttcr.tri.C > max_edge_length_)
    max_edge_length_ = ttcr.tri.C;

  typename MeshTraits::FaceData new_fd = createFaceData (ttcr.tri);
  FaceIndex fi = mesh_.addFace (ttcr.pvr.afront.front.vi[0], ttcr.pvr.afront.front.vi[1], ttcr.closest, new_fd);

  if (!addToQueue (fi))
    addToBoundary (ttcr.pvr.afront.front.he);
}

template <typename PointNT> Eigen::Vector3f
pcl::AfrontMesher<PointNT>::getGrowDirection (const Eigen::Vector3f &p, const Eigen::Vector3f &mp, const typename MeshTraits::FaceData &fd) const
{
  Eigen::Vector3f v1, v2, v3, norm;
  v1 = mp - p;
  norm = fd.getNormalVector3fMap ();
  v2 = norm.cross (v1).normalized ();

  // Check direction from origin of triangle
  v3 = fd.getVector3fMap () - mp;
  if (v2.dot (v3) > 0.0)
    v2 *= -1.0;

  return v2;
}

template <typename PointNT> double
pcl::AfrontMesher<PointNT>::getMaxStep (const Eigen::Vector3f &p, float &radius_found) const
{
  pcl::afront::AfrontGuidanceFieldPointType pn;
  std::vector<int> k;
  std::vector<float> k_dist;
  pn.x = p (0);
  pn.y = p (1);
  pn.z = p (2);

  // What is shown in the afront paper. Need to figure out how to transverse the kdtree.
  double len = std::numeric_limits<double>::max ();
  double radius = 0;
  int j = 1;
  int pcnt = 0;
  bool finished = false;
  double search_radius = search_radius_;

  // This is to provide the ability to seed the search
  if (radius_found > 0)
  {
    search_radius = radius_found;
  }

  while (pcnt < (mls_cloud_->points.size () - 1))
  {
    int cnt = mls_cloud_tree_->radiusSearch (pn, j * search_radius, k, k_dist);
    for (int i = pcnt; i < cnt; ++i)
    {
      int neighbors = mls_.getMLSResults ()[i].num_neighbors;
      if (neighbors < required_neighbors_)
        continue;

      pcl::afront::AfrontGuidanceFieldPointType &gp = mls_cloud_->at (k[i]);
      radius = sqrt (k_dist[i]);

      double step_required = (1.0 - reduction_) * radius + reduction_ * gp.ideal_edge_length;
      len = std::min (len, step_required);

      if (radius >= (len / (1.0 - reduction_)))
      {
        radius_found = radius;
        finished = true;
        break;
      }
    }

    if (finished)
      break;

    if (cnt != 0)
      pcnt = cnt - 1;

    ++j;
  }

  return len;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::TriangleData
pcl::AfrontMesher<PointNT>::getTriangleData (const FrontData &front, const pcl::afront::AfrontVertexPointType &p) const
{
  TriangleData result;
  Eigen::Vector3f v1, v2, v3, cross;
  double dot, sina, cosa, top, bottom, area;
  Eigen::Vector3f p3 = p.getArray3fMap ();
  Eigen::Vector3f p3_normal = p.getNormalVector3fMap ();

  result.valid = false;
  result.point_valid = true;
  result.vertex_normals_valid = true;
  result.triangle_normal_valid = true;

  v1 = front.p[1] - front.p[0];
  v2 = p3 - front.p[1];
  v3 = p3 - front.p[0];

  dot = v2.dot (front.d);
  if (dot <= 0.0)
    result.point_valid = false;

  result.A = v1.norm ();
  result.B = v2.norm ();
  result.C = v3.norm ();

  // Calculate the first angle of triangle
  dot = v1.dot (v3);
  cross = v1.cross (v3);
  bottom = result.A * result.C;
  top = cross.norm ();
  sina = top / bottom;
  cosa = dot / bottom;
  result.b = atan2 (sina, cosa);

  area = 0.5 * top;
  result.aspect_ratio = (4.0 * area * std::sqrt (3)) / (result.A * result.A + result.B * result.B + result.C * result.C);
  result.normal = cross.normalized ();
  pcl::alignNormals (result.normal, front.n[0]);
  assert (!std::isnan (result.normal[0]));

  // Lets check triangle and vertex normals
  if (!pcl::checkNormalsEqual (front.n[0], p3_normal, vertex_normal_tol_) || !pcl::checkNormalsEqual (front.n[1], p3_normal, vertex_normal_tol_))
  {
    result.vertex_normals_valid = false;
    result.triangle_normal_valid = false;
  }
  else
  {
    Eigen::Vector3f avg_normal = (front.n[0] + front.n[1] + p3_normal) / 3.0;
    if (!pcl::checkNormalsEqual (avg_normal, result.normal, triangle_normal_tol_))
      result.triangle_normal_valid = false;
  }

  v1 *= -1.0;
  dot = v1.dot (v2);
  cross = v1.cross (v2);
  bottom = result.A * result.B;
  sina = cross.norm () / bottom;
  cosa = dot / bottom;
  result.c = atan2 (sina, cosa);

  result.a = M_PI - result.b - result.c;

  // Store point information
  result.p[0] = front.p[0];
  result.p[1] = front.p[1];
  result.p[2] = p3;

  result.valid = result.point_valid && result.vertex_normals_valid && result.triangle_normal_valid;
  return result;
}

template <typename PointNT> typename pcl::AfrontMesher<PointNT>::MeshTraits::FaceData
pcl::AfrontMesher<PointNT>::createFaceData (const TriangleData &tri) const
{
  typename MeshTraits::FaceData center_pt;
  Eigen::Vector3f cp = (tri.p[0] + tri.p[1] + tri.p[2]) / 3.0;

  center_pt.x = cp (0);
  center_pt.y = cp (1);
  center_pt.z = cp (2);
  center_pt.normal_x = tri.normal (0);
  center_pt.normal_y = tri.normal (1);
  center_pt.normal_z = tri.normal (2);

  return center_pt;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::addToQueueHelper (const HalfEdgeIndex &half_edge)
{
  assert (std::find (queue_.begin (), queue_.end (), half_edge) == queue_.end ());
  queue_.push_back (half_edge);
}

template <typename PointNT> bool
pcl::AfrontMesher<PointNT>::addToQueue (const FaceIndex &face)
{
  // This occures if the face is non-manifold.
  // It appears that non-manifold vertices are allowed but not faces.
  if (mesh_.isValid (face))
  {
    OHEAFC circ = mesh_.getOuterHalfEdgeAroundFaceCirculator (face);
    const OHEAFC circ_end = circ;
    do
    {
      HalfEdgeIndex he = circ.getTargetIndex ();
      if (mesh_.isBoundary (he))
        addToQueueHelper (he);

    } while (++circ != circ_end);
  }
  else
  {
    PCL_ERROR ("Unable to perform merge, invalid face index!\n");
    return false;
  }

  return true;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::removeFromQueue (const HalfEdgeIndex &half_edge)
{
  queue_.erase (std::remove_if (queue_.begin (), queue_.end (), boost::lambda::_1 == half_edge), queue_.end ());
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::removeFromQueue (const HalfEdgeIndex &half_edge1, const HalfEdgeIndex &half_edge2)
{
  queue_.erase (std::remove_if (queue_.begin (), queue_.end (), (boost::lambda::_1 == half_edge1 || boost::lambda::_1 == half_edge2)), queue_.end ());
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::addToBoundary (const HalfEdgeIndex &half_edge)
{
  assert (std::find (boundary_.begin (), boundary_.end (), half_edge) == boundary_.end ());
  boundary_.push_back (half_edge);
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::removeFromBoundary (const HalfEdgeIndex &half_edge)
{
  boundary_.erase (std::remove_if (boundary_.begin (), boundary_.end (), boost::lambda::_1 == half_edge), boundary_.end ());
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::removeFromBoundary (const HalfEdgeIndex &half_edge1, const HalfEdgeIndex &half_edge2)
{
  boundary_.erase (std::remove_if (boundary_.begin (), boundary_.end (), (boost::lambda::_1 == half_edge1 || boost::lambda::_1 == half_edge2)), boundary_.end ());
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::printVertices () const
{
  std::cout << "Vertices:\n   ";
  for (unsigned int i = 0; i < mesh_.sizeVertices (); ++i)
  {
    std::cout << mesh_vertex_data_ptr_->at (i) << " ";
  }
  std::cout << std::endl;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::printFaces () const
{
  std::cout << "Faces:\n";
  for (unsigned int i = 0; i < mesh_.sizeFaces (); ++i)
    printFace (FaceIndex (i));
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::printEdge (const HalfEdgeIndex &half_edge) const
{
  std::cout << "  "
            << mesh_vertex_data_ptr_->at (mesh_.getOriginatingVertexIndex (half_edge).get ())
            << " "
            << mesh_vertex_data_ptr_->at (mesh_.getTerminatingVertexIndex (half_edge).get ())
            << std::endl;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::printFace (const FaceIndex &idx_face) const
{
  // Circulate around all vertices in the face
  VAFC circ = mesh_.getVertexAroundFaceCirculator (idx_face);
  const VAFC circ_end = circ;
  std::cout << "  ";
  do
  {
    std::cout << mesh_vertex_data_ptr_->at (circ.getTargetIndex ().get ()) << " ";
  } while (++circ != circ_end);
  std::cout << std::endl;
}

#ifdef AFRONTDEBUG
template <typename PointNT> pcl::PolygonMesh
pcl::AfrontMesher<PointNT>::getPolynomialSurface (const PredictVertexResults &pvr, const double step) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr poly (new pcl::PointCloud<pcl::PointXYZ> ());

  int wh = 2 * search_radius_ / step + 1;
  poly->width = wh;
  poly->height = wh;
  poly->points.resize (wh * wh);
  int npoints = 0;
  double u, v;
  for (int i = 0; i < wh; i++)
  {
    u = i * step - search_radius_;
    for (int j = 0; j < wh; j++)
    {
      v = j * step - search_radius_;
      double w = pvr.pv.mls.getPolynomialValue (u, v);
      poly->points[npoints].x = static_cast<float> (pvr.pv.mls.mean[0] + pvr.pv.mls.u_axis[0] * u + pvr.pv.mls.v_axis[0] * v + pvr.pv.mls.plane_normal[0] * w);
      poly->points[npoints].y = static_cast<float> (pvr.pv.mls.mean[1] + pvr.pv.mls.u_axis[1] * u + pvr.pv.mls.v_axis[1] * v + pvr.pv.mls.plane_normal[1] * w);
      poly->points[npoints].z = static_cast<float> (pvr.pv.mls.mean[2] + pvr.pv.mls.u_axis[2] * u + pvr.pv.mls.v_axis[2] * v + pvr.pv.mls.plane_normal[2] * w);
      npoints++;
    }
  }

  pcl::PolygonMesh output;
  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  ofm.setInputCloud (poly);
  ofm.setMaxEdgeLength (2 * step);
  ofm.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::QUAD_MESH);
  ofm.reconstruct (output);
  return output;
}

template <typename PointNT> void
pcl::AfrontMesher<PointNT>::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *)
{
  static long unsigned int cnt = 0;
  if (event.keyUp ())
  {
    std::string k = event.getKeySym ();
    if (isdigit (k[0]))
    {
      int num;
      std::stringstream (k) >> num;
      int length = std::pow (10, num);
      if (!isFinished ())
      {
        for (int i = 0; i < length; ++i)
        {
          try
          {
            stepReconstruction (++cnt);
          }
          catch (const std::exception &e)
          {
            std::printf ("\x1B[31m\tFailed to step mesh!\x1B[0m\n");
            break;
          }

          if (isFinished ())
            break;
        }
      }
      viewer_->removePolygonMesh ();
      pcl::PolygonMesh out_mesh;
      pcl::geometry::toFaceVertexMesh (mesh_, out_mesh);
      viewer_->addPolygonMesh (out_mesh);
      return;
    }
  }
}
#endif

#define PCL_INSTANTIATE_AfrontMesher(T) template class PCL_EXPORTS pcl::AfrontMesher<T>;

#endif // PCL_SURFACE_IMPL_ADVANCING_FRONT_H_
