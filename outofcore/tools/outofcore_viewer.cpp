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
#include <string>

// PCL
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

// PCL - visualziation
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

//#include "vtkVBOPolyDataMapper.h"

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include <pcl/outofcore/visualization/axes.h>
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/grid.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/outofcore_cloud.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/viewport.h>

using namespace pcl;
using namespace pcl::outofcore;

using pcl::console::parse_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

//typedef PCLPointCloud2 PointT;
typedef PointXYZ PointT;

typedef OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<PointT>, PointT> octree_disk;
typedef OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer<PointT>, PointT> octree_disk_node;

//typedef octree_base<OutofcoreOctreeDiskContainer<PointT> , PointT> octree_disk;
typedef boost::shared_ptr<octree_disk> OctreeDiskPtr;
//typedef octree_base_node<octree_disk_container<PointT> , PointT> octree_disk_node;
typedef Eigen::aligned_allocator<PointT> AlignedPointT;

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkActor2DCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkAppendFilter.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkHull.h>
#include <vtkInformation.h>
#include <vtkInformationStringKey.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLODActor.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkMutexLock.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkTextActor.h>
#include <vtkRectilinearGrid.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkParallelCoordinatesInteractorStyle.h>

// Boost
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// Globals
vtkSmartPointer<vtkRenderWindow> window;


class KeyboardCallback : public vtkCommand
{
public:
  vtkTypeMacro(KeyboardCallback, vtkCommand)
  ;

  static KeyboardCallback *
  New ()
  {
    return new KeyboardCallback;
  }

  void
  Execute (vtkObject *caller, unsigned long vtkNotUsed(eventId), void* vtkNotUsed(callData))
  {
    vtkRenderWindowInteractor *interactor = vtkRenderWindowInteractor::SafeDownCast (caller);
    vtkRenderer *renderer = interactor->FindPokedRenderer (interactor->GetEventPosition ()[0],
                                                           interactor->GetEventPosition ()[1]);

    std::string key (interactor->GetKeySym ());
    bool shift_down = interactor->GetShiftKey();

    cout << "Key Pressed: " << key << endl;

    Scene *scene = Scene::instance ();
    OutofcoreCloud *cloud = static_cast<OutofcoreCloud*> (scene->getObjectByName ("my_octree"));

    if (key == "Up" || key == "Down")
    {
      if (key == "Up" && cloud)
      {
        if (shift_down)
        {
          cloud->increaseLodPixelThreshold();
        }
        else
        {
          cloud->setDisplayDepth (cloud->getDisplayDepth () + 1);
        }
      }
      else if (key == "Down" && cloud)
      {
        if (shift_down)
        {
          cloud->decreaseLodPixelThreshold();
        }
        else
        {
          cloud->setDisplayDepth (cloud->getDisplayDepth () - 1);
        }
      }
    }

    if (key == "o")
    {
      cloud->setDisplayVoxels(1-static_cast<int> (cloud->getDisplayVoxels()));
    }

    if (key == "Escape")
    {
      Eigen::Vector3d min (cloud->getBoundingBoxMin ());
      Eigen::Vector3d max (cloud->getBoundingBoxMax ());
      renderer->ResetCamera (min.x (), max.x (), min.y (), max.y (), min.z (), max.z ());
    }
  }
};

void
renderTimerCallback(vtkObject* caller, unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData))
{
  vtkRenderWindowInteractor *interactor = vtkRenderWindowInteractor::SafeDownCast (caller);
  interactor->Render ();
}

void
renderStartCallback(vtkObject* vtkNotUsed(caller), unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData))
{
  //std::cout << "Start...";
}

void
renderEndCallback(vtkObject* vtkNotUsed(caller), unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData))
{
  //std::cout << "End" << std::endl;
}

int
outofcoreViewer (boost::filesystem::path tree_root, int depth, bool display_octree=true, unsigned int gpu_cache_size=512)
{
  cout << boost::filesystem::absolute (tree_root) << endl;

  // Create top level scene
  Scene *scene = Scene::instance ();

  // Clouds
  OutofcoreCloud *cloud = new OutofcoreCloud ("my_octree", tree_root);
  cloud->setDisplayDepth (depth);
  cloud->setDisplayVoxels (display_octree);
  OutofcoreCloud::cloud_data_cache.setCapacity(gpu_cache_size*1024);
  scene->addObject (cloud);

//  OutofcoreCloud *cloud2 = new OutofcoreCloud ("my_octree2", tree_root);
//  cloud2->setDisplayDepth (depth);
//  cloud2->setDisplayVoxels (display_octree);
//  scene->addObject (cloud2);

  // Add Scene Renderables
  Grid *grid = new Grid ("origin_grid");
  Axes *axes = new Axes ("origin_axes");
  scene->addObject (grid);
  scene->addObject (axes);

  // Create smart pointer with arguments
//  Grid *grid_raw = new Grid("origin_grid");
//  vtkSmartPointer<Grid> grid;
//  grid.Take(grid_raw);

  // Create window and interactor
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
  window = vtkSmartPointer<vtkRenderWindow>::New ();
  window->SetSize (1000, 500);

  interactor->SetRenderWindow (window);
  interactor->Initialize ();
  interactor->CreateRepeatingTimer (100);

  // Viewports
  Viewport octree_viewport (window, 0.0, 0.0, 0.5, 1.0);
  Viewport persp_viewport (window, 0.5, 0.0, 1.0, 1.0);

  // Cameras
  Camera *persp_camera = new Camera ("persp", persp_viewport.getRenderer ()->GetActiveCamera ());
  Camera *octree_camera = new Camera ("octree", octree_viewport.getRenderer ()->GetActiveCamera ());
  scene->addCamera (persp_camera);
  scene->addCamera (octree_camera);
  octree_camera->setDisplay(true);

  // Set viewport cameras
  persp_viewport.setCamera (persp_camera);
  octree_viewport.setCamera (octree_camera);

  // Render once
  window->Render ();

  // Frame cameras
  Eigen::Vector3d min (cloud->getBoundingBoxMin ());
  Eigen::Vector3d max (cloud->getBoundingBoxMax ());
  octree_viewport.getRenderer ()->ResetCamera (min.x (), max.x (), min.y (), max.y (), min.z (), max.z ());
  persp_viewport.getRenderer ()->ResetCamera (min.x (), max.x (), min.y (), max.y (), min.z (), max.z ());

  cloud->setRenderCamera(octree_camera);

  // Interactor
  // -------------------------------------------------------------------------
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New ();
  style->SetAutoAdjustCameraClippingRange(false);
  interactor->SetInteractorStyle (style);

  // Callbacks
  // -------------------------------------------------------------------------
  vtkSmartPointer<vtkCallbackCommand> render_start_callback = vtkSmartPointer<vtkCallbackCommand>::New();
  render_start_callback->SetCallback(renderStartCallback);
  window->AddObserver(vtkCommand::StartEvent, render_start_callback);

  vtkSmartPointer<vtkCallbackCommand> render_end_callback = vtkSmartPointer<vtkCallbackCommand>::New();
  render_end_callback->SetCallback(renderEndCallback);
  window->AddObserver(vtkCommand::EndEvent, render_end_callback);

  vtkSmartPointer<KeyboardCallback> keyboard_callback = vtkSmartPointer<KeyboardCallback>::New ();
  interactor->AddObserver (vtkCommand::KeyPressEvent, keyboard_callback);

  interactor->CreateRepeatingTimer(1000);
  vtkSmartPointer<vtkCallbackCommand> render_timer_callback = vtkSmartPointer<vtkCallbackCommand>::New();
  render_timer_callback->SetCallback(renderTimerCallback);
  interactor->AddObserver(vtkCommand::TimerEvent, render_timer_callback);

  interactor->Start ();

  return 0;
}

void
print_help (int, char **argv)
{
  print_info ("This program is used to visualize outofcore data structure");
  print_info ("%s <options> <input_tree_dir> \n", argv[0]);
  print_info ("\n");
  print_info ("Options:\n");
  print_info ("\t -depth <depth>                \t Octree depth\n");
  print_info ("\t -display_octree               \t Toggles octree display\n");
//  print_info ("\t -mem_cache_size <size>        \t Size of pointcloud memory cache in MB (Defaults to 1024MB)\n");
  print_info ("\t -gpu_cache_size <size>        \t Size of pointcloud gpu cache in MB (512MB)\n");
  print_info ("\t -lod_threshold <pixels>       \t Bounding box screen projection threshold (10000)\n");
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
//  unsigned int mem_cache_size = 1024;
  unsigned int gpu_cache_size = 512;
  unsigned int lod_threshold = 10000;
  bool display_octree = find_switch (argc, argv, "-display_octree");

  // Parse options
  parse_argument (argc, argv, "-depth", depth);
//  parse_argument (argc, argv, "-mem_cache_size", mem_cache_size);
  parse_argument (argc, argv, "-gpu_cache_size", gpu_cache_size);
  parse_argument (argc, argv, "-lod_threshold", lod_threshold);

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

  return outofcoreViewer (tree_root, depth, display_octree, gpu_cache_size);
}
