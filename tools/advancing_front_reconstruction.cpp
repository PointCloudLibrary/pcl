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

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/advancing_front.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/** \brief A debugging tool for the advancing front reconstruction algorithm.
  *
  * \author Levi H. Armstrong
  * \group surface
  */
template <typename PointNT>
class AfrontMesherDebugTool : public AfrontMesher<PointNT>
{
public:

  /** \brief AfrontMesher Constructor */
  AfrontMesherDebugTool () : AfrontMesher<PointNT> ()
  {
  }

  /** \brief AfrontMesher Destructor */
  ~AfrontMesherDebugTool () {}

  /**  \brief Get the internal viewer */
  pcl::visualization::PCLVisualizer::Ptr
  getViewer ()
  {
    return viewer_;
  }

protected:
  typedef typename  AfrontMesher<PointNT>::PredictVertexResults PredictVertexResults;
  typedef typename  AfrontMesher<PointNT>::MeshTraits MeshTraits;

  virtual bool
  initialize ()
  {
    if (!AfrontMesher<PointNT>::initialize())
      return false;

    fence_counter_ = 0;

    viewer_ = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_->initCameraParameters ();
    viewer_->setBackgroundColor (0, 0, 0);
    pcl::PolygonMesh out_mesh;
    pcl::geometry::toFaceVertexMesh (this->mesh_, out_mesh);
    viewer_->addPolygonMesh (out_mesh);

    int v1 = 1;
    viewer_->createViewPort (0.0, 0.5, 0.5, 1.0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointNT> single_color (this->input_, 0, 255, 0);
    viewer_->addPointCloud<PointNT> (this->input_, single_color, "sample cloud", v1);

    //Show just mesh
    int v2 = 2;
    viewer_->createViewPort (0.5, 0.5, 1.0, 1.0, v2);

    //Show Final mesh results over the mls point cloud
    int v3 = 3;
    viewer_->createViewPort (0.0, 0.0, 0.5, 0.5, v3);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::afront::AfrontGuidanceFieldPointType> handler_k (this->mls_cloud_, "curvature");
    viewer_->addPointCloud<pcl::afront::AfrontGuidanceFieldPointType> (this->mls_cloud_, handler_k, "mls_cloud", v3);

    //Show mls information
    int v4 = 4;
    viewer_->createViewPort (0.5, 0.0, 1.0, 0.5, v4);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::afront::AfrontGuidanceFieldPointType> single_color2 (this->mls_cloud_, 0, 255, 0);
    viewer_->addPointCloud<pcl::afront::AfrontGuidanceFieldPointType> (this->mls_cloud_, single_color2, "mls_cloud2", v4);

    viewer_->registerKeyboardCallback (&AfrontMesherDebugTool<PointNT>::keyboardEventOccurred, *this);
    viewer_->spin ();

    return true;
  }

  virtual PredictVertexResults
  stepReconstruction (const long unsigned int id)
  {
    std::printf ("\x1B[35mAdvancing Front: %lu\x1B[0m\n", id);

    PredictVertexResults pvr = AfrontMesher<PointNT>::stepReconstruction(id);

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
    p1 = pcl::PointXYZ (pvr.afront.next.tri.p[0] (0), pvr.afront.next.tri.p[0] (1), pvr.afront.next.tri.p[0] (2));
    p2 = pcl::PointXYZ (pvr.afront.next.tri.p[1] (0), pvr.afront.next.tri.p[1] (1), pvr.afront.next.tri.p[1] (2));
    p3 = pcl::PointXYZ (pvr.afront.next.tri.p[2] (0), pvr.afront.next.tri.p[2] (1), pvr.afront.next.tri.p[2] (2));
    p4 = pcl::PointXYZ (pvr.afront.next.tri.p[2] (0), pvr.afront.next.tri.p[2] (1), pvr.afront.next.tri.p[2] (2));

    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 0, 255, 0, "HalfEdge");          // Green
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p2, p3, 255, 0, 0, "NextHalfEdge", 1);   // Red
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p4, 255, 0, 255, "PrevHalfEdge", 1); // Magenta
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "HalfEdge");
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "NextHalfEdge", 1);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "PrevHalfEdge", 1);

    typename pcl::visualization::PointCloudColorHandlerCustom<typename MeshTraits::VertexData> single_color (this->mesh_vertex_data_ptr_, 255, 128, 0);
    viewer_->addPointCloud<typename MeshTraits::VertexData> (this->mesh_vertex_data_ptr_, single_color, "Mesh_Vertex_Cloud", 2);

    viewer_->addPointCloudNormals<typename MeshTraits::VertexData> (this->mesh_vertex_data_ptr_, 1, 0.005, "Mesh_Vertex_Cloud_Normals", 4);

    if (pvr.status == PredictVertexResults::Valid)
    {
      pcl::PolygonMesh mls_surface = getPolynomialSurface (pvr, this->search_radius_ / 50);
      viewer_->addPolygonMesh (mls_surface, "MLSSurface", 4);
    }

    Eigen::Vector3f cpt = (this->mls_cloud_->at (pvr.pv.closest)).getVector3fMap ();
    viewer_->addSphere (pcl::PointXYZ (cpt (0), cpt (1), cpt (2)), 0.1 * this->search_radius_, 255, 0, 0, "MLSClosest", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSClosest", 4);

    Eigen::Vector3f projected_pt = pvr.pv.point.getVector3fMap ();
    viewer_->addSphere (pcl::PointXYZ (cpt (0), cpt (1), cpt (2)), this->search_radius_, 0, 255, 128, "MLSRadius", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSRadius", 4);

    pcl::PointXYZ mls_mean = pcl::PointXYZ (pvr.pv.mls.mean (0), pvr.pv.mls.mean (1), pvr.pv.mls.mean (2));
    Eigen::Vector3d mls_xaxis = pvr.pv.mls.mean + 0.1 * this->search_radius_ * pvr.pv.mls.u_axis;
    Eigen::Vector3d mls_yaxis = pvr.pv.mls.mean + 0.1 * this->search_radius_ * pvr.pv.mls.v_axis;
    Eigen::Vector3d mls_zaxis = pvr.pv.mls.mean + 0.1 * this->search_radius_ * pvr.pv.mls.plane_normal;
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_xaxis (0), mls_xaxis (1), mls_xaxis (2)), 255, 0, 0, "MLSXAxis", 4);
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_yaxis (0), mls_yaxis (1), mls_yaxis (2)), 0, 255, 0, "MLSYAxis", 4);
    viewer_->addLine (mls_mean, pcl::PointXYZ (mls_zaxis (0), mls_zaxis (1), mls_zaxis (2)), 0, 0, 255, "MLSZAxis", 4);

    viewer_->addSphere (pcl::PointXYZ (projected_pt (0), projected_pt (1), projected_pt (2)), 0.02 * this->search_radius_, 255, 128, 0, "MLSProjection", 4);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "MLSProjection", 4);

    p3 = pcl::PointXYZ (pvr.tri.p[2] (0), pvr.tri.p[2] (1), pvr.tri.p[2] (2));
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p1, p3, 0, 255, 0, "RightSide", 1); // Green
    viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (p2, p3, 0, 255, 0, "LeftSide", 1);  // Green
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "RightSide", 1);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, "LeftSide", 1);

    if (pvr.status == PredictVertexResults::InvalidClosedArea)
    {
      std::printf ("\x1B[34m  Closing Area\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidMLSResults)
    {
      std::printf ("\x1B[31m  Invalid MLS Results for nearest point!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidProjection)
    {
      std::printf ("\x1B[31m  Invalid Projection!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::AtBoundary)
    {
      std::printf ("\x1B[36m  At Point Cloud Boundary!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidStepSize)
    {
      std::printf ("\x1B[31m  Calculated Invalid Step Size!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidVertexNormal)
    {
      std::printf ("\x1B[31m  Projection point has inconsistant vertex normal!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidTriangleNormal)
    {
      std::printf ("\x1B[31m  Projection point triangle normal is inconsistant with vertex normals!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidCloseProximityVertexNormal)
    {
      std::printf ("\x1B[31m  Closest point has inconsistant vertex normal!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::InvalidCloseProximityTriangleNormal)
    {
      std::printf ("\x1B[31m  Closest point triangle normal is inconsistant with vertex normals!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::ForcePrevEarCut)
    {
      std::printf ("\x1B[32m  Aborting Merge, Forced Ear Cut Opperation with Previous Half Edge!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::ForceNextEarCut)
    {
      std::printf ("\x1B[32m  Aborting Merge, Forced Ear Cut Opperation with Next Half Edge!\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::Valid)
    {
      std::printf ("\x1B[32m  Performed Grow Opperation\x1B[0m\n");
    }
    else if (pvr.status == PredictVertexResults::ValidCloseProximity)
    {
      std::printf ("\x1B[33m  Performed Topology Event Opperation\x1B[0m\n");
    }

    for (int i = 0; i < pvr.ttcr.fences.size (); ++i)
    {
      fence_counter_ += 1;
      Eigen::Vector3f start_pt, end_pt;
      start_pt = (this->mesh_vertex_data_ptr_->at ((this->mesh_.getOriginatingVertexIndex (pvr.ttcr.fences[i])).get ())).getVector3fMap ();
      end_pt = (this->mesh_vertex_data_ptr_->at ((this->mesh_.getTerminatingVertexIndex (pvr.ttcr.fences[i])).get ())).getVector3fMap ();
      std::string fence_name = "fence" + static_cast<std::ostringstream *> (&(std::ostringstream () << fence_counter_))->str ();
      viewer_->addLine<pcl::PointXYZ, pcl::PointXYZ> (pcl::PointXYZ (start_pt (0), start_pt (1), start_pt (2)), pcl::PointXYZ (end_pt (0), end_pt (1), end_pt (2)), 0, 255, 255, fence_name, 2); // orange
      viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, fence_name, 2);
    }

    if (pvr.ttcr.fence_index >= 0)
    {
        std::string fence_name = "fence" + static_cast<std::ostringstream *> (&(std::ostringstream () << (pvr.ttcr.fence_index + 1)))->str ();
        viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 140, 0, fence_name, 1);
    }

    Eigen::Vector3f p;
    p = pvr.tri.p[2];
    viewer_->addSphere (pcl::PointXYZ (p (0), p (1), p (2)), this->AFRONT_CLOSE_PROXIMITY_FACTOR * pvr.afront.front.max_step, 0, 255, 128, "ProxRadius", 1);
    viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "ProxRadius", 1);

    return pvr;
  }

  /** \brief Generate a plygon mesh that represent the mls polynomial surface. */
  pcl::PolygonMesh
  getPolynomialSurface (const PredictVertexResults &pvr, const double step) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr poly (new pcl::PointCloud<pcl::PointXYZ> ());

    int wh = 2 * this->search_radius_ / step + 1;
    poly->width = wh;
    poly->height = wh;
    poly->points.resize (wh * wh);
    int npoints = 0;
    double u, v;
    for (int i = 0; i < wh; i++)
    {
      u = i * step - this->search_radius_;
      for (int j = 0; j < wh; j++)
      {
        v = j * step - this->search_radius_;
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

  /**
   * \brief keyboardEventOccurred
   * \param event
   * \return
   */
  void
  keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *)
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
        if (!this->isFinished ())
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

            if (this->isFinished ())
              break;
          }
        }
        viewer_->removePolygonMesh ();
        pcl::PolygonMesh out_mesh;
        pcl::geometry::toFaceVertexMesh (this->mesh_, out_mesh);
        viewer_->addPolygonMesh (out_mesh);
        return;
      }
    }
  }

  // Debug
  mutable long unsigned int fence_counter_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;

};

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X         = radius used for local surface reconstruction at each point. (Required)\n");
  print_info ("                     -order X          = the polynomial order for local surface reconstruction at each point. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_POLYNOMIAL_ORDER); print_info (")\n");
  print_info ("                     -rho X            = used to control mesh triangle size (0 > rho > 1.570796). (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_RHO); print_info (")\n");
  print_info ("                     -reduction X      = defines how fast the mesh triangles can grow and shrink (0 > reduction < 1). (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_REDUCTION); print_info (")\n");
  print_info ("                     -boundary_angle X = threshold used to determine if a point is on the boundary of the point cloud. (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD); print_info (")\n");
  print_info ("                     -sample_size X    = the number of sample triangles to generate. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_SAMPLE_SIZE); print_info (")\n");
#ifdef _OPENMP
  print_info ("                     -threads X        = the number of threads to use. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_THREADS); print_info (")\n");
#endif
  print_info ("                     -debug            = launch debug tool.\n");
}

bool
loadCloud (const std::string &filename, const std::string &extension, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (extension == ".pcd")
  {
    if (loadPCDFile (filename, cloud) < 0)
      return (false);
  }
  else if (extension == ".ply")
  {
    if (loadPLYFile (filename, cloud) < 0)
      return (false);
  }
  else
  {
    return (false);
  }

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const std::string &extension, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  if (extension == ".vtk")
    saveVTKFile (filename, output);
  else if (extension == ".ply")
    savePLYFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the surface reconstruction of a point cloud using the advancing front algorithm. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  std::string input_file = argv[1];
  std::string input_extension = boost::filesystem::extension(input_file);
  if (input_extension != ".pcd" && input_extension != ".ply")
  {
    print_error("Only input file types supported are: pcd and ply.\n");
    return (-1);
  }

  std::string output_file = argv[2];
  std::string output_extension = boost::filesystem::extension(output_file);
  if (output_extension != ".vtk" && output_extension != ".ply")
  {
    print_error("Only output file types supported are: vtk and ply.\n");
    return (-1);
  }

  // Parse command line arguments
  double radius = 0;
  parse_argument (argc, argv, "-radius", radius);
  if (radius <= 0)
  {
    print_error("Argument -radius is required and must be greater than zero.\n");
    return (-1);
  }
  print_info ("Setting a radius of: "); print_value ("%f\n", radius);

  int order = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_POLYNOMIAL_ORDER;
  parse_argument (argc, argv, "-order", order);
  print_info ("Setting a polynomial order of: "); print_value ("%d\n", order);

  double rho = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_RHO;
  parse_argument (argc, argv, "-rho", rho);
  print_info ("Setting a rho of: "); print_value ("%f\n", rho);

  double reduction = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_REDUCTION;
  parse_argument (argc, argv, "-reduction", reduction);
  print_info ("Setting a reduction of: "); print_value ("%f\n", reduction);

  double boundary_angle = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD;
  parse_argument (argc, argv, "-boundary_angle", boundary_angle);
  print_info ("Setting a boundary angle threshold of: "); print_value ("%f\n", boundary_angle);

  int sample_size = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_SAMPLE_SIZE;
  parse_argument (argc, argv, "-sample_size", sample_size);
  print_info ("Setting a sample size of: "); print_value ("%d\n", sample_size);

#ifdef _OPENMP
  int threads = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_THREADS;
  parse_argument (argc, argv, "-threads", threads);
  print_info ("Setting a number of threads of: "); print_value ("%d\n", threads);
#endif

  int debug = false;
  if (find_argument (argc, argv, "-debug") > 0)
    debug = true;

  print_info ("Setting debug to: "); print_info (debug ? "true" : "false");

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (input_file, input_extension, *cloud))
    return (-1);

  PointCloud<PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*cloud,  *xyz_cloud);

  // Apply the marching cubes algorithm
  PolygonMesh output;
  AfrontMesher<PointXYZ>::Ptr mesher;
  if (debug)
  {
      mesher = AfrontMesher<PointXYZ>::Ptr(new AfrontMesherDebugTool<PointXYZ> ());
  }
  else
  {
      mesher = AfrontMesher<PointXYZ>::Ptr(new AfrontMesher<PointXYZ> ());
  }

  mesher->setRho (rho);
  mesher->setReduction (reduction);
  mesher->setSearchRadius (radius);
  mesher->setPolynomialOrder (order);
  mesher->setBoundaryAngleThreshold (boundary_angle);
  mesher->setInputCloud (xyz_cloud);
  mesher->setSampleSize (sample_size);
#ifdef _OPENMP
  mesher->setNumberOfThreads (threads);
#endif

  TicToc tt;
  tt.tic ();

  print_highlight ("Computing ");
  mesher->reconstruct (output);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

  // Save into the second file
  saveCloud (output_file, output_extension, output);
}
