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
 *  \author Nico Blodow (blodow@cs.tum.edu), Julius Kammerl (julius@kammerl.de)
 * */

#include "pcl/octree/octree.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <vtkCubeSource.h>

/////////////////////////////////////////////////////////////////////////////
// Create a vtkSmartPointer object containing a cube
vtkSmartPointer<vtkPolyData>
GetCuboid (double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  vtkSmartPointer < vtkCubeSource > cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (minX, maxX, minY, maxY, minZ, maxZ);
  return cube->GetOutput ();
}

/////////////////////////////////////////////////////////////////////////////
// Create vtkActorCollection of vtkSmartPointers describing octree cubes
void
GetOctreeActors (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >& voxelCenters, double voxelSideLen, vtkSmartPointer<vtkActorCollection> coll)
{

  vtkSmartPointer < vtkAppendPolyData > treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();

  size_t i;
  double s = voxelSideLen/2.0;

  for (i = 0; i < voxelCenters.size (); i++)
  {

    double x = voxelCenters[i].x;
    double y = voxelCenters[i].y;
    double z = voxelCenters[i].z;

    treeWireframe->AddInput (GetCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
  }

  vtkSmartPointer < vtkActor > treeActor = vtkSmartPointer<vtkActor>::New ();

  vtkSmartPointer < vtkDataSetMapper > mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (treeWireframe->GetOutput ());
  treeActor->SetMapper (mapper);

  treeActor->GetProperty ()->SetRepresentationToWireframe ();
  treeActor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  treeActor->GetProperty ()->SetLineWidth (2);
  coll->AddItem (treeActor);
}


////////////////////////////////////////////////////////////////////////////////
// Create a vtkPolyData object from a set of vtkPoints
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



/////////////////////////////////////////////////////////////////////////////
// Create vtkActorCollection of vtkSmartPointers describing input points
void
  GetPointActors (pcl::PointCloud<pcl::PointXYZ>& cloud, vtkSmartPointer<vtkActorCollection> coll)
{
  vtkPolyData* points_poly;

  size_t i;

  vtkPoints *octreeLeafPoints = vtkPoints::New ();

  // add all points from octree to vtkPoint object
  for (i=0; i< cloud.points.size(); i++) {
    octreeLeafPoints->InsertNextPoint (cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }
  points_poly = (vtkPolyData*)createDataSetFromVTKPoints(octreeLeafPoints);

  vtkSmartPointer<vtkActor> pointsActor = vtkSmartPointer<vtkActor>::New ();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (points_poly);
  pointsActor->SetMapper (mapper);

  pointsActor->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  pointsActor->GetProperty ()->SetPointSize (4);
  coll->AddItem (pointsActor);

}

void
printHelp (int argc, char **argv)
{
  std::cout << "Syntax is " << argv[0] << " <file_name.pcd> <octree resolution> \n";
  std::cout << "Example: ./octree_viewer ../../test/bunny.pcd 0.02 \n";

  exit(1);
}

/////////////////////////////////////////////////////////////////////////////
// MAIN
/* ---[ */
int
  main (int argc, char** argv)
{

  if (argc!=3) {
    printHelp(argc, argv);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud);

  vtkSmartPointer<vtkActorCollection> coll = vtkActorCollection::New ();

  vtkRenderer* ren = vtkRenderer::New ();



  // create octree from pointcloud
  pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (atof (argv[2]));
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  // get vector of voxel centers from octree
  double voxelSideLen;
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxelCenters;
  octree.getOccupiedVoxelCenters (voxelCenters);
  voxelSideLen = sqrt (octree.getVoxelSquaredSideLen ());

  // delete octree
  octree.deleteTree();

  // generate voxel boxes
  GetOctreeActors ( voxelCenters, voxelSideLen, coll);

  // visualize point input
  GetPointActors ( *cloud, coll);

  vtkActor* a;
  coll->InitTraversal ();
  a = coll->GetNextActor ();
  while(a!=0)
  {
    ren->AddActor (a);
    a = coll->GetNextActor ();
  }

  // Create Renderer and Interractor
  ren->SetBackground (1.0, 1.0, 1.0);

  vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New ();
  vtkSmartPointer<vtkRenderWindow> win = vtkSmartPointer<vtkRenderWindow>::New ();
  win->AddRenderer (ren);

  iren->SetRenderWindow (win);
  win->Render ();
  iren->Start ();
}
