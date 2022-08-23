/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

/*
 * obj_rec_ransac_orr_octree.cpp
 *
 *  Created on: Oct 24, 2012
 *      Author: papazov
 *
 *  Visualizes the specialized octree class used for the ransac-based object
 *  recognition. Use the left/right arrows to select a full octree leaf which
 *  will be used to place a sphere at it and to cut the sphere against the
 *  other full octree leaves which are visualized in yellow.
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/ransac_based/orr_octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkVersion.h>
#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkCubeSource.h>
#include <vtkPointData.h>
#include <vtkRenderWindow.h>
#include <vector>
#include <list>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <thread>

using namespace pcl;
using namespace pcl::visualization;
using namespace pcl::recognition;
using namespace pcl::io;
using namespace std::chrono_literals;

void run (const char *file_name, float voxel_size);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>* pcl_normals);
void show_octree (ORROctree* octree, PCLVisualizer& viz, bool show_full_leaves_only);
void node_to_cube (const ORROctree::Node* node, vtkAppendPolyData* additive_octree);
void updateViewer (ORROctree& octree, PCLVisualizer& viz, std::vector<ORROctree::Node*>::iterator leaf);

#define _SHOW_OCTREE_NORMALS_

class CallbackParameters
{
  public:
    CallbackParameters (ORROctree& octree, PCLVisualizer& viz, std::vector<ORROctree::Node*>::iterator leaf)
    : octree_(octree),
      viz_(viz),
      leaf_(leaf)
    { }

    ORROctree& octree_;
    PCLVisualizer& viz_;
    std::vector<ORROctree::Node*>::iterator leaf_;
};

int main (int argc, char ** argv)
{
  if ( argc != 3 )
  {
    fprintf(stderr, "\nERROR: Syntax is ./pcl_obj_rec_ransac_orr_octree <vtk file> <leaf_size>\n"
                    "EXAMPLE: ./pcl_obj_rec_ransac_orr_octree ../../test/tum_rabbit.vtk 6\n\n");
    return -1;
  }

  // Get the voxel size
  float voxel_size = static_cast<float> (atof (argv[2]));
  if ( voxel_size <= 0.0 )
  {
    fprintf(stderr, "ERROR: leaf_size has to be positive and not %lf\n", voxel_size);
    return -1;
  }

  run(argv[1], voxel_size);
}

//===============================================================================================================================

void keyboardCB (const pcl::visualization::KeyboardEvent &event, void* params_void)
{
  auto* params = static_cast<CallbackParameters*> (params_void);

  if (event.getKeySym () == "Left" && event.keyUp ())
  {
    if (params->leaf_ == params->octree_.getFullLeaves ().begin ())
      params->leaf_ = params->octree_.getFullLeaves ().end ();

    updateViewer(params->octree_, params->viz_, --params->leaf_);
  }
  else if (event.getKeySym () == "Right" && event.keyUp ())
  {
    ++params->leaf_;
    if (params->leaf_ == params->octree_.getFullLeaves ().end ())
      params->leaf_ = params->octree_.getFullLeaves ().begin ();

    updateViewer (params->octree_, params->viz_, params->leaf_);
  }
}

//===============================================================================================================================

void updateViewer (ORROctree& octree, PCLVisualizer& viz, std::vector<ORROctree::Node*>::iterator leaf)
{
  viz.removeAllShapes();

  const float *b = (*leaf)->getBounds (), *center = (*leaf)->getData ()->getPoint ();
  float radius = 0.1f*octree.getRoot ()->getRadius ();

  // Add the main leaf as a cube
  viz.addCube (b[0], b[1], b[2], b[3], b[4], b[5], 0.0, 0.0, 1.0, "main cube");

  // Get all full leaves intersecting a sphere with certain radius
  std::list<ORROctree::Node*> intersected_leaves;
  octree.getFullLeavesIntersectedBySphere(center, radius, intersected_leaves);

  char cube_id[128];
  int i = 0;

  // Show the cubes
  for (const auto &intersected_leaf : intersected_leaves)
  {
    sprintf(cube_id, "cube %i", ++i);
    b = intersected_leaf->getBounds ();
    viz.addCube (b[0], b[1], b[2], b[3], b[4], b[5], 1.0, 1.0, 0.0, cube_id);
  }

  // Get a random full leaf on the sphere defined by 'center' and 'radius'
  ORROctree::Node *rand_leaf = octree.getRandomFullLeafOnSphere (center, radius);
  if ( rand_leaf )
  {
    pcl::ModelCoefficients sphere_coeffs;
    sphere_coeffs.values.resize (4);
    sphere_coeffs.values[0] = rand_leaf->getCenter ()[0];
    sphere_coeffs.values[1] = rand_leaf->getCenter ()[1];
    sphere_coeffs.values[2] = rand_leaf->getCenter ()[2];
    sphere_coeffs.values[3] = 0.5f*(b[1] - b[0]);
    viz.addSphere (sphere_coeffs, "random_full_leaf");
  }
}

//===============================================================================================================================

void run (const char* file_name, float voxel_size)
{
  PointCloud<PointXYZ>::Ptr points_in (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr points_out (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr normals_in (new PointCloud<Normal> ());
  PointCloud<Normal>::Ptr normals_out (new PointCloud<Normal> ());

  // Get the points and normals from the input vtk file
#ifdef _SHOW_OCTREE_NORMALS_
  if ( !vtk_to_pointcloud (file_name, *points_in, &(*normals_in)) )
    return;
#else
  if ( !vtk_to_pointcloud (file_name, *points_in, NULL) )
    return;
#endif

  // Build the octree with the desired resolution
  ORROctree octree;
  if ( !normals_in->empty () )
    octree.build (*points_in, voxel_size, &*normals_in);
  else
    octree.build (*points_in, voxel_size);

  // Get the first full leaf in the octree (arbitrary order)
  auto leaf = octree.getFullLeaves ().begin ();

  // Get the average points in every full octree leaf
  octree.getFullLeavesPoints (*points_out);
  // Get the average normal at the points in each leaf
  if ( !normals_in->empty () )
    octree.getNormalsOfFullLeaves (*normals_out);

  // The visualizer
  PCLVisualizer viz;

  // Register a keyboard callback
  CallbackParameters params(octree, viz, leaf);
  viz.registerKeyboardCallback (keyboardCB, static_cast<void*> (&params));

  // Add the point clouds
  viz.addPointCloud (points_in, "cloud in");
  viz.addPointCloud (points_out, "cloud out");
  if ( !normals_in->empty () )
    viz.addPointCloudNormals<PointXYZ,Normal> (points_out, normals_out, 1, 6.0f, "normals out");

  // Change the appearance
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud in");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud out");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud out");

  // Convert the octree to a VTK poly-data object
//  show_octree(&octree, viz, true/*show full leaves only*/);

  updateViewer (octree, viz, leaf);

  // Enter the main loop
  while (!viz.wasStopped ())
  {
    //main loop of the visualizer
    viz.spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}

//===============================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>* pcl_normals)
{
  std::size_t len = strlen (file_name);
  if ( file_name[len-3] != 'v' || file_name[len-2] != 't' || file_name[len-1] != 'k' )
  {
    fprintf (stderr, "ERROR: we need a .vtk object!\n");
    return false;
  }

  // Load the model
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (file_name);
  reader->Update ();

  // Get the points
  vtkPolyData *vtk_poly = reader->GetOutput ();
  vtkPoints *vtk_points = vtk_poly->GetPoints ();
  vtkIdType num_points = vtk_points->GetNumberOfPoints ();
  double p[3];

  pcl_points.resize (num_points);

  // Copy the points
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_points->GetPoint (i, p);
    pcl_points[i].x = static_cast<float> (p[0]);
    pcl_points[i].y = static_cast<float> (p[1]);
    pcl_points[i].z = static_cast<float> (p[2]);
  }

  // Check if we have normals
  vtkDataArray *vtk_normals = vtk_poly->GetPointData ()->GetNormals ();
  if ( vtk_normals && pcl_normals )
  {
    pcl_normals->resize (num_points);
    // Copy the normals
    for ( vtkIdType i = 0 ; i < num_points ; ++i )
    {
      vtk_normals->GetTuple (i, p);
      (*pcl_normals)[i].normal_x = static_cast<float> (p[0]);
      (*pcl_normals)[i].normal_y = static_cast<float> (p[1]);
      (*pcl_normals)[i].normal_z = static_cast<float> (p[2]);
    }
  }

  return true;
}

//===============================================================================================================================

void node_to_cube (const ORROctree::Node* node, vtkAppendPolyData* additive_octree)
{
  // Define the cube representing the leaf
  const float *b = node->getBounds ();
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (b[0], b[1], b[2], b[3], b[4], b[5]);
  cube->Update ();

  additive_octree->AddInputData (cube->GetOutput ());
}

//===============================================================================================================================

void show_octree (ORROctree* octree, PCLVisualizer& viz, bool show_full_leaves_only)
{
  vtkSmartPointer<vtkPolyData> vtk_octree = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New ();

  std::cout << "There are " << octree->getFullLeaves ().size () << " full leaves.\n";

  if ( show_full_leaves_only )
  {
    std::vector<ORROctree::Node*>& full_leaves = octree->getFullLeaves ();
    for (const auto &full_leaf : full_leaves)
      // Add it to the other cubes
      node_to_cube (full_leaf, append);
  }
  else
  {
    std::list<ORROctree::Node*> nodes;
    nodes.push_back (octree->getRoot ());

    while ( !nodes.empty () )
    {
      ORROctree::Node* node = nodes.front ();
      nodes.pop_front ();

      // Visualize the node if it has children
      if ( node->getChildren () )
      {
        // Add it to the other cubes
        node_to_cube (node, append);
        // Add all the children to the working list
        for ( int i = 0 ; i < 8 ; ++i )
          nodes.push_back (node->getChild (i));
      }
      // If we arrived at a leaf -> check if it's full and visualize it
      else if ( node->getData () )
        node_to_cube (node, append);
    }
  }

  // Just print the leaf size
  auto first_leaf = octree->getFullLeaves ().begin ();
  if ( first_leaf != octree->getFullLeaves ().end () )
	  printf("leaf size = %f\n", (*first_leaf)->getBounds ()[1] - (*first_leaf)->getBounds ()[0]);

  // Save the result
  append->Update();
  vtk_octree->DeepCopy (append->GetOutput ());

  // Add to the visualizer
  vtkRenderer *renderer = viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
  vtkSmartPointer<vtkActor> octree_actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInputData (vtk_octree);
  octree_actor->SetMapper(mapper);

  // Set the appearance & add to the renderer
  octree_actor->GetProperty ()->SetColor (1.0, 1.0, 1.0);
  octree_actor->GetProperty ()->SetLineWidth (1);
  octree_actor->GetProperty ()->SetRepresentationToWireframe ();
  renderer->AddActor(octree_actor);
}

//===============================================================================================================================
