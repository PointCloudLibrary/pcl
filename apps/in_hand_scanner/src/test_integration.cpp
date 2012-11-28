/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

////////////////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <vector>
#include <limits>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>
#include <pcl/apps/in_hand_scanner/integration.h>
#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::ihs::PointModel         PointModel;
typedef pcl::ihs::CloudModel         CloudModel;
typedef pcl::ihs::CloudModelPtr      CloudModelPtr;
typedef pcl::ihs::CloudModelConstPtr CloudModelConstPtr;

typedef pcl::ihs::PointProcessed         PointProcessed;
typedef pcl::ihs::CloudProcessed         CloudProcessed;
typedef pcl::ihs::CloudProcessedPtr      CloudProcessedPtr;
typedef pcl::ihs::CloudProcessedConstPtr CloudProcessedConstPtr;

typedef pcl::ihs::Integration             Integration;
typedef pcl::visualization::PCLVisualizer Visualizer;

typedef Integration::Mesh                           Mesh;
typedef Integration::MeshPtr                        MeshPtr;
typedef Integration::MeshConstPtr                   MeshConstPtr;
typedef Mesh::SizeType                              SizeType;
typedef Mesh::Vertex                                Vertex;
typedef Mesh::HalfEdge                              HalfEdge;
typedef Mesh::HalfEdgeIndex                         HalfEdgeIndex;
typedef Mesh::FaceIndex                             FaceIndex;
typedef Mesh::FaceIndexes                           FaceIndexes;
typedef Mesh::VertexConstIterator                   VCI;
typedef Mesh::HalfEdgeConstIterator                 HECI;
typedef Mesh::FaceConstIterator                     FCI;
typedef Mesh::VertexAroundFaceConstCirculator       VAFCCirc;
typedef Mesh::HalfEdgeAroundBoundaryConstCirculator HEABCC;

typedef pcl::Vertices      Face;
typedef std::vector <Face> Faces;

void
drawMesh (const MeshConstPtr& mesh, pcl::visualization::PCLVisualizer& visualizer, int ms, const bool update=true)
{
  Face          triangle; triangle.vertices.resize (3);
  Faces         triangles;
  CloudModelPtr vertexes (new CloudModel ());

  // Convert to polygon mesh for visualization
  vertexes->reserve (mesh->sizeVertexes ());
  triangles.reserve (3 * mesh->sizeFaces ());

  for (VCI it=mesh->beginVertexes (); it!=mesh->endVertexes (); ++it)
  {
    vertexes->push_back (*it);
  }

  for (FCI it=mesh->beginFaces (); it!=mesh->endFaces (); ++it)
  {
    VAFCCirc circ = mesh->getVertexAroundFaceConstCirculator (*it);
    triangle.vertices [0] = (circ++).getDereferencedIndex ().getIndex ();
    triangle.vertices [1] = (circ++).getDereferencedIndex ().getIndex ();
    triangle.vertices [2] = (circ  ).getDereferencedIndex ().getIndex ();
    triangles.push_back (triangle);
  }

  if (update) visualizer.updatePolygonMesh <PointModel> (vertexes, triangles);
  else        visualizer.addPolygonMesh <PointModel> (vertexes, triangles);

  visualizer.spinOnce (ms);
}

void saveVTK (const MeshConstPtr& mesh, const std::string& filename)
{
  Face          triangle; triangle.vertices.resize (3);
  Faces         triangles;
  CloudModelPtr vertexes (new CloudModel ());

  // Convert to polygon mesh for visualization
  vertexes->reserve (mesh->sizeVertexes ());
  triangles.reserve (3 * mesh->sizeFaces ());

  for (VCI it=mesh->beginVertexes (); it!=mesh->endVertexes (); ++it)
  {
    vertexes->push_back (*it);
  }

  for (FCI it=mesh->beginFaces (); it!=mesh->endFaces (); ++it)
  {
    VAFCCirc circ = mesh->getVertexAroundFaceConstCirculator (*it);
    triangle.vertices [0] = (circ++).getDereferencedIndex ().getIndex ();
    triangle.vertices [1] = (circ++).getDereferencedIndex ().getIndex ();
    triangle.vertices [2] = (circ  ).getDereferencedIndex ().getIndex ();
    triangles.push_back (triangle);
  }

  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg (*vertexes, pc2);

  pcl::PolygonMesh pm;
  pm.cloud    = pc2;
  pm.polygons = triangles;

  pcl::io::saveVTKFile (filename, pm);
}

void
disableNormalWeighting (const CloudProcessedPtr& cloud)
{
  for (CloudProcessed::iterator it=cloud->begin (); it!=cloud->end (); ++it)
  {
    if (pcl::isFinite (*it))
    {
      it->normal_x =  0.f;
      it->normal_y =  0.f;
      it->normal_z = -1.f;
    }
  }
}

// remove outliers (regions with a small boundary)
void
simpleOutlierRemoval (const MeshPtr& mesh, const SizeType n, const bool cleanup=true)
{
  // TODO: maybe put the deleted faces into another vector
  // -> don't delete anything while going through this loop
  for (HalfEdgeIndex ind=HalfEdgeIndex (0); ind<HalfEdgeIndex (mesh->sizeHalfEdges ()); ++ind)
  {
    const HalfEdge& he = mesh->getElement (ind);
    if (he.isBoundary () && !he.getDeleted ())
    {
      HEABCC       circ     = mesh->getHalfEdgeAroundBoundaryConstCirculator (ind);
      const HEABCC circ_end = circ;

      FaceIndexes  deleted_faces; deleted_faces.reserve (n);
      bool         delete_faces = false;

      for (SizeType i=0; i<n; ++i)
      {
        ++circ;
        deleted_faces.push_back (circ->getOppositeFaceIndex (*mesh));
        if (circ == circ_end)
        {
          delete_faces = true;
          break;
        }
      }

      if (delete_faces)
      {
        for (FaceIndexes::const_iterator it = deleted_faces.begin (); it!=deleted_faces.end (); ++it)
        {
          mesh->deleteFace (*it);
        }
      }
    }
  }

  if (cleanup)
  {
    mesh->cleanUp ();
  }
}

int
main (int argc, char** argv)
{
  // Parse the command line arguments for .pcd files
  std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  if (file_indices.size () < 2)
  {
    PCL_ERROR ("Please give at least 2 .pcd files as input.\n");
    return (EXIT_FAILURE);
  }

  pcl::PCDReader reader;
  MeshPtr        mesh_model (new Mesh ());
  Integration    integration;
  Visualizer     visualizer;

  // TODO: add the other settings as well
  float squared_distance_threshold = 0.f;
  if (pcl::console::parse (argc, argv, "-d", squared_distance_threshold) != -1)
  {
    integration.setDistanceThreshold (squared_distance_threshold);
  }

  // merge
  for (int i=0; i<file_indices.size (); ++i)
  {
    const std::string filename = argv[file_indices.at (i)];
    std::cerr << filename << std::endl;

    CloudProcessedPtr cloud_data (new CloudProcessed ());
    reader.read (filename, *cloud_data);
    disableNormalWeighting (cloud_data);

    if (i==0)
    {
      integration.reconstructMesh (cloud_data, mesh_model);
      drawMesh(mesh_model, visualizer, 500, false);
    }
    else
    {
      integration.merge (cloud_data, mesh_model, pcl::ihs::Transformation::Identity ());
      drawMesh(mesh_model, visualizer, 1);
    }
  }

  std::cerr << "simple outlier removal ... ";
  drawMesh(mesh_model, visualizer, 500);
  simpleOutlierRemoval (mesh_model, 10); // TODO: hard coded threshold
  std::cerr << "done\n";
  drawMesh(mesh_model, visualizer, 1);

  std::string filename;
  if (pcl::console::parse (argc, argv, "-f", filename) != -1)
  {
    std::cerr << "saving '" << filename << "' ... ";
    saveVTK (mesh_model, filename);
    std::cerr << "done" << "\n";
  }

  visualizer.spin ();

  return (EXIT_SUCCESS);
}
