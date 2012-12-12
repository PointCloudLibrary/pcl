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

#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/in_hand_scanner/custom_interactor_style.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/integration.h>
#include <pcl/apps/in_hand_scanner/impl/common_functions.hpp>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::InHandScanner (int argc, char** argv)
  : mutex_                 (),
    run_                   (true),
    visualization_fps_     (),
    computation_fps_       (),
    running_mode_          (RM_UNPROCESSED),
    iteration_             (0),

    visualizer_            (),
    interactor_style_      (),
    draw_crop_box_         (false),
    pivot_                 (0.f, 0.f, 0.f, 1.f),
    display_mode_          (DM_POINTS),

    grabber_               (),
    new_data_connection_   (),

    input_data_processing_ (new InputDataProcessing ()),

    icp_                   (new ICP ()),
    transformation_        (Transformation::Identity ()),

    integration_           (new Integration ()),

    cloud_data_draw_       (),
    mesh_vec_draw_         (),
    mesh_model_            (new Mesh ())

{
  std::cerr << "Initializing the grabber ...\n  ";
  try
  {
    grabber_ = GrabberPtr (new Grabber ());
  }
  catch (const pcl::PCLException& e)
  {
    std::cerr << "ERROR in in_hand_scanner.cpp: " << e.what () << std::endl;
    exit (EXIT_FAILURE);
  }
  std::cerr << "DONE\n";

  // Visualizer
  visualizer_ = PCLVisualizerPtr (new PCLVisualizer (argc, argv, "PCL in-hand scanner", pcl::ihs::CustomInteractorStyle::New ()));

  // TODO: Adapt this to the resolution of the grabbed image
  // grabber_->getDevice ()->get??
  visualizer_->getRenderWindow ()->SetSize (640, 480);

  interactor_style_ = dynamic_cast <InteractorStylePtr> (visualizer_->getInteractorStyle ().GetPointer ());
  if (!interactor_style_)
  {
    std::cerr << "ERROR in in_hand_scanner.cpp: Could not get the custom interactor style\n";
    exit (EXIT_FAILURE);
  }
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::~InHandScanner ()
{
  if (grabber_ && grabber_->isRunning ()) grabber_->stop ();
  if (new_data_connection_.connected ())  new_data_connection_.disconnect ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::run ()
{
  // Visualizer callbacks
  visualizer_->registerKeyboardCallback (&pcl::ihs::InHandScanner::keyboardCallback, *this);

  // Grabber callbacks
  boost::function <void (const CloudInputConstPtr&)> new_data_cb = boost::bind (&pcl::ihs::InHandScanner::newDataCallback, this, _1);
  new_data_connection_ = grabber_->registerCallback (new_data_cb);

  grabber_->start ();

  // Visualization loop
  while (run_ && !visualizer_->wasStopped ())
  {
    this->calcFPS (visualization_fps_);

    this->draw ();
    this->drawCropBox ();
    this->drawFPS ();

    if (mutex_.try_lock ())
    {
      interactor_style_->setPivot (pivot_.cast <double> ().head <3> ());
      mutex_.unlock ();

      // TODO: Test if this works
      //  vtkCamera*const cam = visualizer_->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
      //  const Eigen::Vector3d pos = Eigen::Map<Eigen::Vector3d> (cam->GetPosition ());
      //  cam->SetFocalPoint (pivot_.cast <double> ().data ());
      //  cam->SetPosition (pos.cast <double> ().data ());
    }

    visualizer_->spinOnce ();
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::quit ()
{
  boost::mutex::scoped_lock lock (mutex_);

  run_ = false;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setRunningMode (const RunningMode& mode)
{
  boost::mutex::scoped_lock lock (mutex_);

  switch (mode)
  {
    case RM_SHOW_MODEL:
    {
      draw_crop_box_ = false;
      std::cerr << "Show the model\n";
      break;
    }
    case RM_UNPROCESSED:
    {
      draw_crop_box_ = false;
      std::cerr << "Showing the unprocessed input data\n";
      break;
    }
    case RM_PROCESSED:
    {
      draw_crop_box_ = true;
      std::cerr << "Showing the processed input data\n";
      break;
    }
    case RM_REGISTRATION_CONT:
    {
      draw_crop_box_ = true;
      std::cerr << "Continuous registration\n";
      break;
    }
    case RM_REGISTRATION_SINGLE:
    {
      draw_crop_box_ = true;
      std::cerr << "Single registration\n";
      break;
    }
    default:
    {
      std::cerr << "ERROR in in_hand_scanner.cpp: Unknown command!\n";
      return;
    }
  }

  running_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setDisplayMode (const DisplayMode& mode)
{
  boost::mutex::scoped_lock lock (mutex_);

  switch (mode)
  {
    case DM_POINTS: std::cerr << "Displaying the points\n";                          break;
    case DM_EDGES:  std::cerr << "Displaying the edges.\n";                          break;
    case DM_MESH:   std::cerr << "Displaying the mesh\n";                            break;
    default:        std::cerr << "ERROR in in_hand_scanner.cpp: Unknown command!\n"; return;
  }

  display_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::resetRegistration ()
{
  boost::mutex::scoped_lock lock (mutex_);

  running_mode_    = RM_PROCESSED;
  iteration_       = 0;
  transformation_  = Transformation::Identity ();
  mesh_vec_draw_.clear ();
  mesh_model_->clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::resetCamera ()
{
  boost::mutex::scoped_lock lock (mutex_);

  interactor_style_->resetCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::newDataCallback (const CloudInputConstPtr& cloud_in)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (!run_) return;

  this->calcFPS (computation_fps_);

  // Input data processing
  CloudProcessedPtr cloud_data;
  if      (running_mode_ == RM_SHOW_MODEL)  cloud_data = CloudProcessedPtr (new CloudProcessed ());
  else if (running_mode_ == RM_UNPROCESSED) cloud_data = input_data_processing_->calculateNormals (cloud_in);
  else if (running_mode_ >= RM_PROCESSED)   cloud_data = input_data_processing_->process (cloud_in);

  // Registration & integration
  if (running_mode_ >= RM_REGISTRATION_CONT)
  {
    if(running_mode_ == RM_REGISTRATION_SINGLE)
    {
      running_mode_ = RM_PROCESSED;
    }

    if (iteration_ == 0)
    {
      transformation_ = Transformation::Identity ();
      integration_->reconstructMesh (cloud_data, mesh_model_);
      cloud_data = CloudProcessedPtr (new CloudProcessed ());
      ++iteration_;
    }
    else
    {
      Transformation T = Transformation::Identity ();
      if (icp_->findTransformation (mesh_model_, cloud_data, transformation_, T))
      {
        transformation_ = T;
        integration_->merge (cloud_data, mesh_model_, transformation_);
        integration_->age (mesh_model_);
        cloud_data = CloudProcessedPtr (new CloudProcessed ());

        // Does not work here because multiple threads!
        // interactor_style_->transformCamera (InteractorStyle::Quaternion (T.topLeftCorner <3, 3> ().cast <double> ()), T.topRightCorner <3, 1> ().cast <double> ());

        ++iteration_;
      }
    }
  }

  // Visualization
  cloud_data_draw_ = cloud_data;
  MeshPtr mesh_model_draw (new Mesh ());
  if (running_mode_ != RM_UNPROCESSED) *mesh_model_draw = *mesh_model_;
  mesh_vec_draw_.push_back (mesh_model_draw);

  // TODO: put this into the visualization thread.
  Eigen::Vector4f pivot (0.f, 0.f, 0.f, 1.f);
  if (mesh_model_draw->sizeVertexes () && pcl::compute3DCentroid (*mesh_model_draw, pivot)) pivot_ = pivot;
  else if (pcl::compute3DCentroid (*cloud_data_draw_, pivot))                               pivot_ = pivot;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::draw ()
{
  CloudProcessedPtr cloud_temp;
  MeshPtrVec mesh_vec_temp;
  if (mutex_.try_lock ())
  {
    cloud_temp.swap (cloud_data_draw_);

    mesh_vec_temp.reserve (mesh_vec_draw_.size ());
    for (MeshPtrVec::iterator it=mesh_vec_draw_.begin (); it<mesh_vec_draw_.end (); ++it)
    {
      MeshPtr mesh_temp;
      mesh_temp.swap (*it);
      mesh_vec_temp.push_back (mesh_temp);
    }
    mesh_vec_draw_.clear ();

    mutex_.unlock ();
  }

  if (cloud_temp)
  {
    if (display_mode_==DM_POINTS)
    {
      pcl::visualization::PointCloudColorHandlerRGBField <PointProcessed> ch (cloud_temp);
      if (!visualizer_->updatePointCloud <PointProcessed> (cloud_temp, ch))
      {
        visualizer_->addPointCloud <PointProcessed> (cloud_temp, ch);
      }
    }
    else
    {
      MeshPtr mesh_temp (new Mesh ());
      integration_->reconstructMesh (cloud_temp, mesh_temp);
      mesh_vec_temp.push_back (mesh_temp);
    }
  }

  for (MeshPtrVec::const_iterator it_m=mesh_vec_temp.begin (); it_m!=mesh_vec_temp.end (); ++it_m)
  {
    if (!(*it_m))          continue;
    if ((*it_m)->empty ()) continue;

    // Convert to cloud + indices for visualization
    typedef pcl::Vertices                         Face;
    typedef std::vector <Face>                    Faces;
    typedef Mesh::VertexConstIterator             VCI;
    typedef Mesh::FaceConstIterator               FCI;
    typedef Mesh::VertexAroundFaceConstCirculator VAFCCirc;

    Face          triangle; triangle.vertices.resize (3);
    CloudModelPtr vertexes (new CloudModel ());
    Faces         triangles;
    vertexes->reserve ((*it_m)->sizeVertexes ());
    triangles.reserve (3 * (*it_m)->sizeFaces ());

    for (VCI it=(*it_m)->beginVertexes (); it!=(*it_m)->endVertexes (); ++it)
    {
      vertexes->push_back (*it);
    }

    for (FCI it=(*it_m)->beginFaces (); it!=(*it_m)->endFaces (); ++it)
    {
      VAFCCirc circ = (*it_m)->getVertexAroundFaceConstCirculator (*it);
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

    // TODO: There is likely a bug in updatePolygonMesh.
    visualizer_->removePolygonMesh ("mesh_model");

    if (!visualizer_->updatePolygonMesh (pm, "mesh_model"))
    {
      visualizer_->addPolygonMesh (pm, "mesh_model");
    }

    vtkSmartPointer <vtkProperty> prop = visualizer_->getRendererCollection ()->GetFirstRenderer ()->GetActors ()->GetLastActor ()->GetProperty ();
    switch (display_mode_)
    {
      case DM_POINTS: prop->SetRepresentationToPoints ();    break;
      case DM_EDGES:  prop->SetRepresentationToWireframe (); break;
      case DM_MESH:   prop->SetRepresentationToSurface ();   break;
    }

    // Doesn't add the color
    //  if (!visualizer_->updatePolygonMesh <PointModel> (vertexes, triangles, "mesh_model"))
    //  {
    //    visualizer_->addPolygonMesh <PointModel> (vertexes, triangles, "mesh_model");
    //  }
  }


//  if (!mesh_model_temp) return;
//  if (!mesh_model_temp->sizeVertexes ())
//  {
//    // TODO: addPolygonMesh does not remove the mesh if it is empty: "[0;m[1;31m[addPolygonMesh] No vertices given!"
//    visualizer_->removePolygonMesh ("mesh_model");
//    return;
//  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::drawCropBox ()
{
  static bool crop_box_added = false;
  if (draw_crop_box_ && !crop_box_added)
  {
    float x_min, x_max, y_min, y_max, z_min, z_max;
    input_data_processing_->getCropBox (x_min, x_max, y_min, y_max, z_min, z_max);
    visualizer_->addCube (x_min, x_max, y_min, y_max, z_min, z_max, 1., 1., 1., "crop_box");
    crop_box_added = true;
  }
  else if (!draw_crop_box_ && crop_box_added)
  {
    visualizer_->removeShape ("crop_box");
    crop_box_added = false;
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::drawFPS ()
{
  std::string vis_fps ("Visualization: "), comp_fps ("Computation: ");
  bool draw = false;

  if (mutex_.try_lock ())
  {
    vis_fps.append (visualization_fps_.str ()).append (" fps");
    comp_fps.append (computation_fps_.str ()).append (" fps");
    draw = true;
    mutex_.unlock ();
  }

  if (!draw) return;

  if (!visualizer_->updateText (vis_fps, 1, 15, "visualization_fps"))
  {
    visualizer_->addText (vis_fps, 1, 15, "visualization_fps");
  }

  if (!visualizer_->updateText (comp_fps, 1, 30, "computation_fps"))
  {
    visualizer_->addText (comp_fps, 1, 30, "computation_fps");
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
{
  if(!event.keyDown ())
  {
    return;
  }

  switch (event.getKeyCode ())
  {
    case 'h': case 'H':
    {
      std::cerr << "======================================================================\n"
                << "Help:\n"
                << "----------------------------------------------------------------------\n"
                << "q, ESC: Quit the application\n"
                << "----------------------------------------------------------------------\n"
                << "1     : Shows the unprocessed input data\n"
                << "2     : Shows the processed input data\n"
                << "3     : Registers new data to the first acquired data continuously\n"
                << "4     : Registers new data once and returns to '2'\n"
                << "5     : Shows the model shape\n"
                << "0     : Reset the registration\n"
                << "----------------------------------------------------------------------\n"
                << "c     : Reset the camera\n"
                << "d     : Switching the representation between points, edges and a mesh\n"
                << "======================================================================\n";
      break;
    }
    case   0:                                                break; // Special key
    case  27: // ESC
    case 'q': this->quit ();                                 break;
    case '1': this->setRunningMode (RM_UNPROCESSED);         break;
    case '2': this->setRunningMode (RM_PROCESSED);           break;
    case '3': this->setRunningMode (RM_REGISTRATION_CONT);   break;
    case '4': this->setRunningMode (RM_REGISTRATION_SINGLE); break;
    case '5': this->setRunningMode (RM_SHOW_MODEL);          break;
    case '0': this->resetRegistration ();                    break;
    case 'c': this->resetCamera ();                          break;
    case 'd':
    {
      switch (display_mode_)
      {
        case DM_POINTS: this->setDisplayMode (DM_EDGES);     break;
        case DM_EDGES:  this->setDisplayMode (DM_MESH);      break;
        case DM_MESH:   this->setDisplayMode (DM_POINTS);    break;
      }                                                      break;
    }
    default:                                                 break;
  }
}

////////////////////////////////////////////////////////////////////////////////
