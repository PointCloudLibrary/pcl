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
 *
 *  \author Justin Rosen (jmylesrosen@gmail.com)
 * */

// C++
#include <iostream>

// PCL
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

// PCL - visualziation
//#include <pcl/visualization/pcl_visualizer.h>
//#include "vtkVBOPolyDataMapper.h"

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

using namespace pcl;
using namespace pcl::outofcore;

using pcl::console::parse_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

typedef PointXYZRGB PointT;
typedef octree_base<octree_disk_container<PointT> , PointT> octree_disk;
typedef octree_base_node<octree_disk_container<PointT> , PointT> octree_disk_node;
typedef Eigen::aligned_allocator<PointT> AlignedPointT;

// VTK
#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

// Boost
#include <boost/filesystem.hpp>

// Definitions
#define MAX_DEPTH -1

vtkSmartPointer<vtkPolyData>
getCuboid (double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (x_min, x_max, y_min, y_max, z_min, z_max);
  return cube->GetOutput ();
}

vtkSmartPointer<vtkActor>
getOctreeActor (std::vector<PointT, AlignedPointT> &voxel_centers, double voxel_side_length)
{
  vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();

  double s = voxel_side_length / 2;
  for (size_t i = 0; i < voxel_centers.size (); i++)
  {
    double x = voxel_centers[i].x;
    double y = voxel_centers[i].y;
    double z = voxel_centers[i].z;

    treeWireframe->AddInput (getCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
  }

  vtkSmartPointer<vtkActor> treeActor = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (treeWireframe->GetOutput ());
  treeActor->SetMapper (mapper);

  treeActor->GetProperty ()->SetRepresentationToWireframe ();
  treeActor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  treeActor->GetProperty ()->SetLighting (false);

  return treeActor;
}

vtkSmartPointer<vtkActor>
getCloudActor (std::list<PointT> points)
{
  vtkSmartPointer<vtkPoints> cloud_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cloud_vertices = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();

  colors->SetNumberOfComponents (3);
  colors->SetName ("Colors");

  std::list<PointT>::iterator it;
  vtkIdType pid[1];
  for (it = points.begin (); it != points.end (); it++)
  {
    pid[0] = cloud_points->InsertNextPoint (it->x, it->y, it->z);
    cloud_vertices->InsertNextCell (1, pid);
    unsigned char rgb[3] = {it->r, it->g, it->b};
    colors->InsertNextTupleValue (rgb);
  }

  vtkSmartPointer<vtkPolyData> cloud = vtkSmartPointer<vtkPolyData>::New ();
  //set the points and vertices we created as the geometry and topology of the polydata
  cloud->SetPoints (cloud_points);
  cloud->SetVerts (cloud_vertices);

  //cloud->GetCellData()->SetScalars(colors);

  vtkSmartPointer<vtkActor> cloud_actor = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  //vtkSmartPointer<vtkVBOPolyDataMapper> mapper = vtkSmartPointer<vtkVBOPolyDataMapper>::New ();
  mapper->SetInput (cloud);
  cloud_actor->SetMapper (mapper);
  cloud_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  cloud_actor->GetProperty ()->SetPointSize (1);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 4))
  cloud_actor->GetProperty ()->SetLighting (0);
#endif
  cloud_actor->Modified ();

  return cloud_actor;

}

int
outofcoreViewer (boost::filesystem::path tree_root, int depth, bool display_octree)
{
  cout << boost::filesystem::absolute (tree_root) << endl;
  octree_disk octree (tree_root, true);

  double min[3], max[3];
  octree.getBB (min, max);
  cout << " Bounds: (" << min[0] << ", " << min[1] << ", " << min[2] << ") - " << max[0] << ", " << max[1] << ", "
      << max[2] << ")" << endl;

  // Get tree LOD
  cout << " Depth: " << octree.getDepth () << endl;

  // Get point count every LOD
  cout << " LOD Points: [";
  std::vector<boost::uint64_t> lodPoints = octree.getNumPoints ();
  for (boost::uint64_t i = 0; i < lodPoints.size () - 1; i++)
    cout << lodPoints[i] << " ";
  cout << lodPoints[lodPoints.size () - 1] << "]" << endl;

  // Get voxel size and divide by 2 - we +/- from the center for bounding cubes
  double voxel_side_length = octree.getVoxelSideLength ((size_t)depth);
  cout << " Voxel Side Length: " << voxel_side_length << endl;

  // Print bounding box info
  //octree.printBBox();

  // Print voxel count
  std::vector<PointT, AlignedPointT> voxel_centers;
  octree.getVoxelCenters (voxel_centers, (size_t)depth);
  cout << " Voxel Count: " << voxel_centers.size () << " - " << voxel_centers[0] << endl;
  //  cout << " Voxel Bounds: [" << voxel_centers[0].x - voxel_side_length << ", " << voxel_centers[0].y - voxel_side_length << ", " << voxel_centers[0].z - voxel_side_length << "] -" <<
  //          " [" << voxel_centers[0].x + voxel_side_length << ", " << voxel_centers[0].y + voxel_side_length << ", " << voxel_centers[0].z + voxel_side_length << "]" << endl;

  std::list<PointT> points;
  octree.queryBBIncludes (min, max, (size_t)depth, points);
  cout << " Point Count: " << points.size () << endl;

  vtkRenderer *renderer = vtkRenderer::New ();
  vtkRenderWindowInteractor *interactor = vtkRenderWindowInteractor::New ();
  vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New ();

  // Generate voxel boxes
  vtkSmartPointer<vtkActor> octree_actor = getOctreeActor (voxel_centers, voxel_side_length);
  vtkSmartPointer<vtkActor> cloud_actor = getCloudActor (points);

  renderer->AddActor (cloud_actor);
  if (display_octree)
    renderer->AddActor (octree_actor);

  window->AddRenderer (renderer);
  window->SetSize (500, 500);
  interactor->SetRenderWindow (window);

  window->Render ();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New ();
  interactor->SetInteractorStyle (style);

  interactor->Start ();

  return 0;
}

void
print_help (int argc, char **argv)
{
  print_info ("This program is used to visualize outofcore data structure");
  print_info ("%s <options> <input_tree_dir> \n", argv[0]);
  print_info ("\n");
  print_info ("Options:\n");
  print_info ("\t -depth <depth>                \t Octree depth\n");
  print_info ("\t -display_octree               \t Toggles octree display\n");
  print_info ("\t -v                            \t Print more verbosity\n");
  print_info ("\t -h                            \t Display help\n");
  print_info ("\n");

  exit (1);
}

int
main (int argc, char* argv[])
{

  // Check for help (-h) flag
  if (argc > 1)
  {
    if (find_switch (argc, argv, "-h"))
    {
      print_help (argc, argv);
      return (-1);
    }
  }

  // If no arguments specified
  if (argc - 1 < 1)
  {
    print_help (argc, argv);
    return (-1);
  }

  if (find_switch (argc, argv, "-v"))
    console::setVerbosityLevel (console::L_DEBUG);

  // Defaults
  int depth = 4;
  bool display_octree = find_switch (argc, argv, "-display_octree");

  // Parse options
  parse_argument (argc, argv, "-depth", depth);

  // Parse non-option arguments
  boost::filesystem::path tree_root (argv[argc - 1]);

  // Check if a root directory was specified, use directory of pcd file
  if (boost::filesystem::is_directory (tree_root))
  {
    boost::filesystem::directory_iterator diterend;
    for (boost::filesystem::directory_iterator diter (tree_root); diter != diterend; ++diter)
    {
      const boost::filesystem::path& file = *diter;
      if (!boost::filesystem::is_directory (file))
      {
        if (boost::filesystem::extension (file) == octree_disk_node::node_index_extension)
        {
          tree_root = file;
        }
      }
    }
  }

  return outofcoreViewer (tree_root, depth, display_octree);
}
