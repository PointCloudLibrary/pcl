/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <vtkCubeSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLine.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef PointXYZ PointT;
typedef PointCloud<PointT> CloudT;

float default_leaf_size = 0.01f;

vtkDataSet*
createDataSetFromVTKPoints (vtkPoints *points)
{
  vtkCellArray *verts = vtkCellArray::New ();
  vtkPolyData *data   = vtkPolyData::New  ();
  // Iterate through the points                                                          
                             
  for (vtkIdType i = 0; i < points->GetNumberOfPoints (); i++)
    verts->InsertNextCell ((vtkIdType)1, &i);
  data->SetPoints (points);
  data->SetVerts (verts);
  return data;
}

vtkSmartPointer<vtkPolyData>
getCuboid (double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  vtkSmartPointer < vtkCubeSource > cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (minX, maxX, minY, maxY, minZ, maxZ);
  return cube->GetOutput ();
}

void
getVoxelActors (pcl::PointCloud<pcl::PointXYZ>& voxelCenters,
                 double voxelSideLen, Eigen::Vector3f color,
                 vtkSmartPointer<vtkActorCollection> coll)
{
  vtkSmartPointer < vtkAppendPolyData > treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();
  
  size_t i;
  double s = voxelSideLen/2.0;
  
  for (i = 0; i < voxelCenters.points.size (); i++)
  {
    double x = voxelCenters.points[i].x;
    double y = voxelCenters.points[i].y;
    double z = voxelCenters.points[i].z;
    
#if VTK_MAJOR_VERSION < 6
    treeWireframe->AddInput (getCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
#else
    treeWireframe->AddInputData (getCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
#endif

  }

  vtkSmartPointer < vtkLODActor > treeActor = vtkSmartPointer<vtkLODActor>::New ();
  
  vtkSmartPointer < vtkDataSetMapper > mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
#if VTK_MAJOR_VERSION < 6
  mapper->SetInput (treeWireframe->GetOutput ());
#else
  mapper->SetInputData (treeWireframe->GetOutput ());
#endif

  treeActor->SetMapper (mapper);
  
  treeActor->GetProperty ()->SetRepresentationToWireframe ();
  treeActor->GetProperty ()->SetColor (color[0], color[1], color[2]);
  treeActor->GetProperty ()->SetLineWidth (4);
  coll->AddItem (treeActor);
}

void
displayBoundingBox (Eigen::Vector3f& min_b, Eigen::Vector3f& max_b,
                    vtkSmartPointer<vtkActorCollection> coll)
{
  vtkSmartPointer < vtkAppendPolyData > treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();
#if VTK_MAJOR_VERSION < 6
  treeWireframe->AddInput (getCuboid (min_b[0], max_b[0], min_b[1], max_b[1], min_b[2], max_b[2]));
#else
  treeWireframe->AddInputData (getCuboid (min_b[0], max_b[0], min_b[1], max_b[1], min_b[2], max_b[2]));
#endif

  vtkSmartPointer < vtkActor > treeActor = vtkSmartPointer<vtkActor>::New ();

  vtkSmartPointer < vtkDataSetMapper > mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
#if VTK_MAJOR_VERSION < 6
  mapper->SetInput (treeWireframe->GetOutput ());
#else
  mapper->SetInputData (treeWireframe->GetOutput ());
#endif
  treeActor->SetMapper (mapper);

  treeActor->GetProperty ()->SetRepresentationToWireframe ();
  treeActor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  treeActor->GetProperty ()->SetLineWidth (2);
  coll->AddItem (treeActor);
}

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -leaf x,y,z   = the VoxelGrid leaf size (default: "); 
  print_value ("%f, %f, %f", default_leaf_size, default_leaf_size, default_leaf_size); print_info (")\n");
}

int main (int argc, char** argv)
{
  print_info ("Estimate occlusion using pcl::VoxelGridOcclusionEstimation. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  float leaf_x = default_leaf_size,
        leaf_y = default_leaf_size,
        leaf_z = default_leaf_size;

  std::vector<double> values;
  parse_x_arguments (argc, argv, "-leaf", values);
  if (values.size () == 1)
  {
    leaf_x = static_cast<float> (values[0]);
    leaf_y = static_cast<float> (values[0]);
    leaf_z = static_cast<float> (values[0]);
  }
  else if (values.size () == 3)
  {
    leaf_x = static_cast<float> (values[0]);
    leaf_y = static_cast<float> (values[1]);
    leaf_z = static_cast<float> (values[2]);
  }
  else
  {
    print_error ("Leaf size must be specified with either 1 or 3 numbers (%lu given).\n", values.size ());
  }
  print_info ("Using a leaf size of: "); print_value ("%f, %f, %f\n", leaf_x, leaf_y, leaf_z);

  // input cloud
  CloudT::Ptr input_cloud (new CloudT);
  loadPCDFile (argv[1], *input_cloud);

  // VTK actors
  vtkSmartPointer<vtkActorCollection> coll = vtkActorCollection::New ();

  // voxel grid
  VoxelGridOcclusionEstimation<PointT> vg;
  vg.setInputCloud (input_cloud);
  vg.setLeafSize (leaf_x, leaf_y, leaf_z);
  vg.initializeVoxelGrid ();

  Eigen::Vector3f b_min, b_max;
  b_min = vg.getMinBoundCoordinates ();
  b_max = vg.getMaxBoundCoordinates ();

  TicToc tt;
  print_highlight ("Ray-Traversal ");
  tt.tic ();

  // estimate the occluded space
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > occluded_voxels;
  vg.occlusionEstimationAll (occluded_voxels);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", (int)occluded_voxels.size ()); print_info (" occluded voxels]\n");
  
  CloudT::Ptr occ_centroids (new CloudT);
  occ_centroids->width = static_cast<int> (occluded_voxels.size ());
  occ_centroids->height = 1;
  occ_centroids->is_dense = false;
  occ_centroids->points.resize (occluded_voxels.size ());
  for (size_t i = 0; i < occluded_voxels.size (); ++i)
  {
    Eigen::Vector4f xyz = vg.getCentroidCoordinate (occluded_voxels[i]);
    PointT point;
    point.x = xyz[0];
    point.y = xyz[1];
    point.z = xyz[2];
    occ_centroids->points[i] = point;
  }

  CloudT::Ptr cloud_centroids (new CloudT);
  cloud_centroids->width = static_cast<int> (input_cloud->points.size ());
  cloud_centroids->height = 1;
  cloud_centroids->is_dense = false;
  cloud_centroids->points.resize (input_cloud->points.size ());

  for (size_t i = 0; i < input_cloud->points.size (); ++i)
  {
    float x = input_cloud->points[i].x;
    float y = input_cloud->points[i].y;
    float z = input_cloud->points[i].z;
    Eigen::Vector3i c = vg.getGridCoordinates (x, y, z);
    Eigen::Vector4f xyz = vg.getCentroidCoordinate (c);
    PointT point;
    point.x = xyz[0];
    point.y = xyz[1];
    point.z = xyz[2];
    cloud_centroids->points[i] = point;
  }

  // visualization
  Eigen::Vector3f red (1.0, 0.0, 0.0);  
  Eigen::Vector3f blue (0.0, 0.0, 1.0);
  // draw point cloud voxels
  getVoxelActors (*cloud_centroids, leaf_x, red, coll);
  // draw the bounding box of the voxel grid
  displayBoundingBox (b_min, b_max, coll);
  // draw the occluded voxels
  getVoxelActors (*occ_centroids, leaf_x, blue, coll);

  // Create the RenderWindow, Renderer and RenderWindowInteractor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer (renderer);  
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow (renderWindow);

  // Add the actors to the renderer, set the background and size
  renderer->SetBackground (1.0, 1.0, 1.0);
  renderWindow->SetSize (640, 480);

  vtkActor* a;
  coll->InitTraversal ();
  a = coll->GetNextActor ();
  while(a!=0)
    {
      renderer->AddActor (a);
      a = coll->GetNextActor ();
    }

  renderWindow->Render ();
  renderWindowInteractor->Start ();

  return 0;
}
