/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

/*
 * visualize_obj_rec_ransac_opp.cpp
 *
 *  Created on: Jan 17, 2013
 *      Author: papazov
 *
 *  Calls recognize() of the ObjRecRANSAC class and visualizes the oriented point pairs (opp) sampled from the scene.
 *  Does NOT perform full recognition.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <vtkPolyDataReader.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
#include <cstdio>
#include <vector>

using namespace std;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

void run (const char* file_name, float voxel_size);
void show_octree (const ORROctree& octree, PCLVisualizer& viz);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals);
void node_to_cube (const ORROctree::Node* node, vtkAppendPolyData* additive_octree);

#define _SHOW_INPUT_POINTS_
#define _SHOW_OCTREE_POINTS_
#define _SHOW_OCTREE_

class CallbackParameters
{
  public:
    CallbackParameters (list<ObjRecRANSAC::OrientedPointPair>& opps, PCLVisualizer& viz)
    : opps_(opps),
      viz_(viz)
    { }

    list<ObjRecRANSAC::OrientedPointPair>& opps_;
    PCLVisualizer& viz_;
};

//===========================================================================================================================================

int
main (int argc, char** argv)
{
  if ( argc != 3 )
  {
    fprintf(stderr, "\nERROR: Syntax is ./pcl_visualize_obj_rec_ransac_opp <vtk file> <leaf_size>\n"
                    "EXAMPLE: ./pcl_visualize_obj_rec_ransac_opp ../../test/TUM_Table_Scene.vtk 6\n\n");
    return (-1);
  }

  // Get the voxel size
  float voxel_size = static_cast<float> (atof (argv[2]));
  if ( voxel_size <= 0.0 )
  {
    fprintf(stderr, "ERROR: leaf_size has to be positive and not %lf\n", voxel_size);
    return (-1);
  }

  run(argv[1], voxel_size);

  return (0);
}

//===============================================================================================================================

void run (const char* file_name, float voxel_size)
{
  PointCloud<PointXYZ>::Ptr points_in (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr points_octree (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr normals_in (new PointCloud<Normal> ());
  PointCloud<Normal>::Ptr normals_out (new PointCloud<Normal> ());

  // Get the points and normals from the input vtk file
  if ( !vtk_to_pointcloud (file_name, *points_in, *normals_in) )
    return;

  // The recognition object
  ObjRecRANSAC objrec (40.0f, voxel_size);
  list<ObjRecRANSAC::Output> dummy_output;
  // Switch to the test mode in which only oriented point pairs from the scene are sampled
  objrec.enterTestModeSampleOPP ();
  // Run the recognition method
  objrec.recognize (&(*points_in), &(*normals_in), dummy_output);

  // The visualizer
  PCLVisualizer viz;

#ifdef _SHOW_OCTREE_
  show_octree(objrec.getSceneOctree (), viz);
#endif

#ifdef _SHOW_INPUT_POINTS_
  viz.addPointCloud (points_in, "cloud in");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud in");
#endif

#ifdef _SHOW_OCTREE_POINTS_
  objrec.getSceneOctree ().getFullLeafPoints (*points_octree);
  viz.addPointCloud (points_octree, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "octree points");
#endif

  // Enter the main loop
  while (!viz.wasStopped ())
  {
    //main loop of the visualizer
    viz.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

//===============================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals)
{
  size_t len = strlen (file_name);
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
  if ( !vtk_normals )
    return false;

  pcl_normals.resize (num_points);
  // Copy the normals
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_normals->GetTuple (i, p);
    pcl_normals[i].normal_x = static_cast<float> (p[0]);
    pcl_normals[i].normal_y = static_cast<float> (p[1]);
    pcl_normals[i].normal_z = static_cast<float> (p[2]);
  }

  return true;
}

//===============================================================================================================================

void show_octree (const ORROctree& octree, PCLVisualizer& viz)
{
  vtkSmartPointer<vtkPolyData> vtk_octree = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New ();

  cout << "There are " << octree.getFullLeaves ().size () << " full leaves.\n";

  const std::vector<ORROctree::Node*>& full_leaves = octree.getFullLeaves ();
  for ( std::vector<ORROctree::Node*>::const_iterator it = full_leaves.begin () ; it != full_leaves.end () ; ++it )
    // Add it to the other cubes
    node_to_cube (*it, append);

  // Just print the leaf size
  std::vector<ORROctree::Node*>::const_iterator first_leaf = octree.getFullLeaves ().begin ();
  if ( first_leaf != octree.getFullLeaves ().end () )
	  printf("leaf size = %f\n", (*first_leaf)->getBounds ()[1] - (*first_leaf)->getBounds ()[0]);

  // Save the result
  append->Update();
  vtk_octree->DeepCopy (append->GetOutput ());

  // Add to the visualizer
  vtkRenderer *renderer = viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
  vtkSmartPointer<vtkActor> octree_actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput(vtk_octree);
  octree_actor->SetMapper(mapper);

  // Set the appearance & add to the renderer
  octree_actor->GetProperty ()->SetColor (1.0, 1.0, 1.0);
  octree_actor->GetProperty ()->SetLineWidth (1);
  octree_actor->GetProperty ()->SetRepresentationToWireframe ();
  renderer->AddActor(octree_actor);
}

//===============================================================================================================================

void node_to_cube (const ORROctree::Node* node, vtkAppendPolyData* additive_octree)
{
  // Define the cube representing the leaf
  const float *b = node->getBounds ();
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (b[0], b[1], b[2], b[3], b[4], b[5]);
  cube->Update ();

  additive_octree->AddInput (cube->GetOutput ());
}

//===============================================================================================================================
