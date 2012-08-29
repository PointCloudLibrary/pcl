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

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

// PCL - visualziation
//#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/visualization/vtk/vtkVertexBufferObjectMapper.h"

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

// VTK
#include "vtkgl.h"
#include "vtkShaderProgram2.h"
#include "vtkShader2.h"
#include "vtkShader2Collection.h"

using namespace pcl;
using namespace pcl::outofcore;
using namespace sensor_msgs;

using pcl::console::parse_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

typedef PointXYZ PointT;
//typedef PointCloud2 PointT;
typedef OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<PointT> , PointT> octree_disk;
typedef OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer<PointT> , PointT> octree_disk_node;
typedef Eigen::aligned_allocator<PointT> AlignedPointT;

// VTK
#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRectilinearGrid.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUniformVariables.h>

// Boost
#include <boost/filesystem.hpp>

// Definitions
const int MAX_DEPTH (-1);

vtkSmartPointer<vtkPolyData>
getCuboid (double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (x_min, x_max, y_min, y_max, z_min, z_max);
  return cube->GetOutput ();
}

vtkSmartPointer<vtkActor>
getOctreeActor (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, double voxel_side_length)
{
  vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();

  double s = voxel_side_length / 2;
  for (size_t i = 0; i < voxel_centers.size (); i++)
  {
    double x = voxel_centers[i].x ();
    double y = voxel_centers[i].y ();
    double z = voxel_centers[i].z ();

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
getGridActor(int size=10, double spacing=1.0){

// Create a grid
  vtkSmartPointer<vtkRectilinearGrid> grid =
    vtkSmartPointer<vtkRectilinearGrid>::New();

  grid->SetDimensions(size, size, 1);

  vtkSmartPointer<vtkDoubleArray> array = vtkSmartPointer<vtkDoubleArray>::New();
  for (int i=-size/2; i <= size/2; i++){
    array->InsertNextValue (static_cast<double> (i)*spacing);
  }

//  vtkSmartPointer<vtkDoubleArray> yArray =
//    vtkSmartPointer<vtkDoubleArray>::New();
//  yArray->InsertNextValue(0.0);
//  yArray->InsertNextValue(1.0);
//  yArray->InsertNextValue(2.0);
//
  vtkSmartPointer<vtkDoubleArray> zArray =
    vtkSmartPointer<vtkDoubleArray>::New();
  zArray->InsertNextValue(0.0);

  grid->SetXCoordinates(array);
  grid->SetYCoordinates(array);
  grid->SetZCoordinates(zArray);


  std::cout << "There are " << grid->GetNumberOfPoints()
            << " points." << std::endl;
  std::cout << "There are " << grid->GetNumberOfCells()
            << " cells." << std::endl;

  for(vtkIdType id = 0; id < grid->GetNumberOfPoints(); id++)
    {
    double p[3];
    grid->GetPoint(id, p);
    std::cout << "Point " << id
              << " : (" << p[0] << " , " << p[1] << " , " << p[2] << ")" << std::endl;
    }

  // Create a mapper and actor
  vtkSmartPointer<vtkDataSetMapper> mapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
#if VTK_MAJOR_VERSION <= 5
  mapper->SetInputConnection(grid->GetProducerPort());
#else
  mapper->SetInputData(grid);
#endif

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetColor (0.3, 0.3, 0.3);
  actor->GetProperty ()->SetLighting (false);

  return actor;
}

// Shaders
// ----------------------------------------------------------------------------

const vtkgl::GLchar* VertexShader =
  "#version 120\n"

//  "varying vec4 normals;"

  "void main(void){\n"
  "  gl_Position = ftransform();\n"
  "  gl_FrontColor = gl_Color;\n"
  "  gl_BackColor = gl_Color;\n"
  "}\0";

const vtkgl::GLchar* IntensityShader =
  "#version 120\n"

  "void main(void){"
  "  gl_Position = ftransform();"
  "  gl_FrontColor = vec4(gl_Color[0], gl_Color[0], gl_Color[0], gl_Color[3]);"
  "  gl_BackColor = vec4(gl_Color[0], gl_Color[0], gl_Color[0], gl_Color[3]);"
  "}\0";

// Display Intensity data as RGB
const vtkgl::GLchar* NormalColorShader =
  "#version 120\n"

  "void main(void){"
  "  gl_Position = ftransform();"
  "  gl_FrontColor = vec4(abs(gl_Color[0]), abs(gl_Color[1]), abs(gl_Color[2]), gl_Color[3]);"
  "  gl_BackColor = vec4(abs(gl_Color[0]), abs(gl_Color[1]), abs(gl_Color[2]), gl_Color[3]);"
  "}\0";


const vtkgl::GLchar* FragmentShader =
  "#version 120\n"

  "void main(void){\n"
  "   gl_FragColor = gl_Color;\n"
  "}\0";

const vtkgl::GLchar* NormalsVertexShader =
  "#version 120\n"

  "varying vec4 normals;"

  "void main(void){\n"
  "  gl_Position = gl_Vertex;\n"
  "  normals = vec4(gl_Normal, 0);"
  "}\0";

const vtkgl::GLchar* NormalsGeometryShader =
  "#version 120\n"
  "#extension GL_EXT_geometry_shader4 : enable\n"

  "varying in vec4 normals[];"

  "void main(void){"
  "  for(int i = 0; i < gl_VerticesIn; i=i++){"
  "    if (mod(i,1000) == 0){"
  "      gl_Position = gl_ModelViewProjectionMatrix * gl_PositionIn[i];"
  "      gl_FrontColor = vec4(0.0, 0.698, 0.0, 1.0);"
  "      gl_BackColor = vec4(0.0, 0.698, 0.0, 1.0);"
  "      EmitVertex();"

  "      gl_Position = gl_ModelViewProjectionMatrix * (gl_PositionIn[i] + (normals[i] * 0.05));"
  "      gl_FrontColor = vec4(0.0, 0.698, 0.0, 1.0);"
  "      gl_BackColor = vec4(0.0, 0.698, 0.0, 1.0);"
  "      EmitVertex();"
  "    }"
  "    EndPrimitive();"
  "  }"
  "}\0";

vtkSmartPointer<vtkActor>
getCloudActor (const PointCloud2Ptr &cloud)
{

  vtkSmartPointer<vtkPolyData> vtkCloud;
  pcl::io::pointCloudTovtkPolyData(cloud, vtkCloud);

  vtkSmartPointer<vtkActor> cloud_actor = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkVertexBufferObjectMapper> mapper = vtkSmartPointer<vtkVertexBufferObjectMapper>::New ();

  mapper->SetInput (vtkCloud);

  vtkSmartPointer<vtkShaderProgram2> program = vtkSmartPointer<vtkShaderProgram2>::New();

  vtkShader2 *vertexShader = vtkShader2::New();
  vertexShader->SetType(VTK_SHADER_TYPE_VERTEX);
  vertexShader->SetSourceCode(VertexShader);
  //vertexShader->SetSourceCode(IntensityShader);
  //vertexShader->SetSourceCode(NormalColorShader);

  vtkShader2 *fragmentShader = vtkShader2::New();
  fragmentShader->SetType(VTK_SHADER_TYPE_FRAGMENT);
  fragmentShader->SetSourceCode(FragmentShader);

  // Geometry shaders available in 5.8+
//  vtkShader2 *geometryShader = vtkShader2::New();
//  geometryShader->SetType(VTK_SHADER_TYPE_GEOMETRY);
//  geometryShader->SetSourceCode(NormalsGeometryShader);
//
//  program->SetGeometryTypeIn(VTK_GEOMETRY_SHADER_IN_TYPE_POINTS);
//  program->SetGeometryTypeOut(VTK_GEOMETRY_SHADER_OUT_TYPE_LINE_STRIP);

//  int maxVertices;
//  glGetIntegerv(vtkgl::MAX_GEOMETRY_OUTPUT_VERTICES_EXT, &maxVertices);
//  program->SetGeometryVerticesOut(maxVertices);

  program->GetShaders()->AddItem(vertexShader);
  program->GetShaders()->AddItem(fragmentShader);
//  program->GetShaders()->AddItem(geometryShader);

  std::cout << "Created Shaders" << std::endl;

  mapper->SetProgram(program);
  cloud_actor->SetMapper (mapper);
  cloud_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  cloud_actor->GetProperty ()->SetOpacity (1.0);
  cloud_actor->GetProperty ()->SetPointSize (1.0);

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 4))
  cloud_actor->GetProperty ()->SetLighting (0);
#endif
  cloud_actor->Modified ();

  return cloud_actor;

}

int
outofcoreViewer (boost::filesystem::path tree_root, int depth, bool display_octree=true)
{
  cout << boost::filesystem::absolute (tree_root) << endl;
  octree_disk octree (tree_root, true);

  Eigen::Vector3d min, max;
  octree.getBoundingBox (min, max);
  cout << " Bounds: (" << min[0] << ", " << min[1] << ", " << min[2] << ") - " << max[0] << ", " << max[1] << ", "
      << max[2] << ")" << endl;

  // Get tree LOD
  cout << " Depth: " << octree.getDepth () << endl;

  // Get point count every LOD
  cout << " LOD Points: [";
  std::vector<boost::uint64_t> lodPoints = octree.getNumPointsVector ();
  for (boost::uint64_t i = 0; i < lodPoints.size () - 1; i++)
    cout << lodPoints[i] << " ";
  cout << lodPoints[lodPoints.size () - 1] << "]" << endl;

  // Get voxel size and divide by 2 - we +/- from the center for bounding cubes
  double voxel_side_length = octree.getVoxelSideLength (depth);
  cout << " Voxel Side Length: " << voxel_side_length << endl;

  // Print bounding box info
  //octree.printBBox();

  // Print voxel count
  //std::vector<PointT, AlignedPointT> voxel_centers;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > voxel_centers;
  octree.getVoxelCenters (voxel_centers, depth);
  cout << " Voxel Count: " << voxel_centers.size () << endl;

  PointCloud2Ptr cloud(new PointCloud2);
  octree.queryBBIncludes (min, max, depth, cloud);
  cout << " Point Count: " << cloud->width * cloud->height << endl;

  vtkRenderer *renderer = vtkRenderer::New ();
  vtkRenderWindowInteractor *interactor = vtkRenderWindowInteractor::New ();
  vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New ();

  // Generate voxel boxes
  vtkSmartPointer<vtkActor> octree_actor = getOctreeActor (voxel_centers, voxel_side_length);
  vtkSmartPointer<vtkActor> cloud_actor = getCloudActor (cloud);

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
print_help (char **argv)
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
      print_help (argv);
      return (-1);
    }
  }

  // If no arguments specified
  if (argc - 1 < 1)
  {
    print_help (argv);
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

  return outofcoreViewer (tree_root, static_cast<size_t>(depth), display_octree);
}
