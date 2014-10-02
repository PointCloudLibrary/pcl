/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/common/common_headers.h>
#include <pcl/visualization/common/common.h>
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
#include <pcl/visualization/interactor.h>
#else
#include <vtkRenderWindowInteractor.h>
#endif
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/boost.h>

#include <vtkVersion.h>
#include <vtkXYPlotActor.h>
#include <vtkDoubleArray.h>
#include <vtkTextProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkDataObject.h>
#include <vtkProperty2D.h>
#include <vtkFieldData.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLHistogramVisualizer::PCLHistogramVisualizer () : 
  wins_ (),
  exit_main_loop_timer_callback_ (vtkSmartPointer<ExitMainLoopTimerCallback>::New ()), 
  exit_callback_ (vtkSmartPointer<ExitCallback>::New ()), 
  stopped_ ()
{
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  resetStoppedFlag ();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Spin once method. Calls the interactor and updates the screen once. */
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Spin once method. Calls the interactor and updates the screen once. */
void
pcl::visualization::PCLHistogramVisualizer::spinOnce (int time, bool force_redraw)
{
  resetStoppedFlag ();

  if (time <= 0)
    time = 1;
  
  if (force_redraw)
  {
    for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    {
      (*am_it).second.interactor_->Render ();
      exit_main_loop_timer_callback_->right_timer_id = (*am_it).second.interactor_->CreateRepeatingTimer (time);

      exit_main_loop_timer_callback_->interact = (*am_it).second.interactor_;

      (*am_it).second.interactor_->Start ();
      (*am_it).second.interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    }
    return;
  }
  
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    DO_EVERY(1.0/(*am_it).second.interactor_->GetDesiredUpdateRate (),
      (*am_it).second.interactor_->Render ();
      exit_main_loop_timer_callback_->right_timer_id = (*am_it).second.interactor_->CreateRepeatingTimer (time);

      exit_main_loop_timer_callback_->interact = (*am_it).second.interactor_;

      (*am_it).second.interactor_->Start ();
      (*am_it).second.interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
  }
}
#else
void
pcl::visualization::PCLHistogramVisualizer::spinOnce (int time)
{
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    DO_EVERY(1.0/(*am_it).second.interactor_->GetDesiredUpdateRate (),
      (*am_it).second.interactor_->Render ();
      exit_main_loop_timer_callback_->right_timer_id = (*am_it).second.interactor_->CreateRepeatingTimer (time);

      // Set the correct interactor for callbacks
      exit_main_loop_timer_callback_->interact = (*am_it).second.interactor_;
      (*am_it).second.interactor_->Start ();
      (*am_it).second.interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
  }
}
#endif

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizer::spin ()
{
  resetStoppedFlag ();
  do
  {
    spinOnce ();
    for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    {
      if ((*am_it).second.interactor_->stopped)
        return;
    }
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }
  while (true);
}
////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::visualization::PCLHistogramVisualizer::wasStopped ()
{
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    // If any of the interactors was stopped, return true (stop everything else)
    if ((*am_it).second.interactor_->stopped)
      return (true);
  }
  return (false);
}

////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLHistogramVisualizer::resetStoppedFlag () 
{ 
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    (*am_it).second.interactor_->stopped = false; 
}

#else
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizer::spin ()
{
  stopped_ = false;
  do
  {
    spinOnce ();
    if (stopped_)
      break;
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }
  while (true);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLHistogramVisualizer::setBackgroundColor (const double &r, const double &g, const double &b)
{
  /*
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->SetBackground (r, g, b);
      renderer->Render ();
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->SetBackground (r, g, b);
      renderer->Render ();
    }
    ++i;
  }
  */
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    (*am_it).second.ren_->SetBackground (r, g, b);
    (*am_it).second.ren_->Render ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLHistogramVisualizer::setGlobalYRange (float minp, float maxp)
{
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    (*am_it).second.xy_plot_->SetYRange (minp, maxp);
    (*am_it).second.xy_plot_->Modified ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLHistogramVisualizer::updateWindowPositions ()
{
  int posx = 0, posy = 0;
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    // Get the screen size
    int *scr_size = (*am_it).second.win_->GetScreenSize ();
    int *win_size = (*am_it).second.win_->GetActualSize ();

    // Update the position of the current window
    (*am_it).second.win_->SetPosition (posx, posy);
    (*am_it).second.win_->Modified ();
    // If there is space on Y, go on Y first
    if ((posy + win_size[1]) <= scr_size[1]) 
      posy += win_size[1];
    // Go on X
    else
    {
      posy = 0;
      if ((posx + win_size[0]) <= scr_size[0]) 
        posx += win_size[0];
      else
        posx = 0;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLHistogramVisualizer::reCreateActor (
    const vtkSmartPointer<vtkDoubleArray> &xy_array, RenWinInteract* renwinupd, const int hsize)
{
  renwinupd->ren_->RemoveActor2D (renwinupd->xy_plot_);
#if VTK_MAJOR_VERSION < 6
  renwinupd->xy_plot_->RemoveAllInputs ();
#else
  renwinupd->xy_plot_->RemoveAllDataSetInputConnections ();
#endif

  
  double min_max[2];
  xy_array->GetRange (min_max, 1);

  // Create the data structures
  vtkSmartPointer<vtkFieldData> field_values = vtkSmartPointer<vtkFieldData>::New ();
  field_values->AddArray (xy_array);

  vtkSmartPointer<vtkDataObject> field_data = vtkSmartPointer<vtkDataObject>::New ();
  field_data->SetFieldData (field_values);

  renwinupd->xy_plot_->AddDataObjectInput (field_data);
  renwinupd->ren_->AddActor2D (renwinupd->xy_plot_);
  
  renwinupd->xy_plot_->SetYTitle (""); renwinupd->xy_plot_->SetXTitle ("");
  renwinupd->xy_plot_->SetYRange (min_max[0], min_max[1]); 
  renwinupd->xy_plot_->SetXRange (0, hsize - 1);
}
   

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizer::createActor (
    const vtkSmartPointer<vtkDoubleArray> &xy_array, 
    pcl::visualization::RenWinInteract &renwinint,
    const std::string &id, const int win_width, const int win_height)
{
  // Create the actor
  renwinint.xy_plot_->SetDataObjectPlotModeToColumns ();
  renwinint.xy_plot_->SetXValuesToValue ();

  // Create the data structures
  vtkSmartPointer<vtkFieldData> field_values = vtkSmartPointer<vtkFieldData>::New ();
  field_values->AddArray (xy_array);

  vtkSmartPointer<vtkDataObject> field_data = vtkSmartPointer<vtkDataObject>::New ();
  field_data->SetFieldData (field_values);

  renwinint.xy_plot_->AddDataObjectInput (field_data);
  // Set the plot color
  renwinint.xy_plot_->SetPlotColor (0, 1.0, 0.0, 0.0);

  renwinint.xy_plot_->SetDataObjectXComponent (0, 0); renwinint.xy_plot_->SetDataObjectYComponent (0, 1);
  renwinint.xy_plot_->PlotPointsOn ();
  //renwinint.xy_plot_->PlotCurvePointsOn ();
  //renwinint.xy_plot_->PlotLinesOn ();
  renwinint.xy_plot_->PlotCurveLinesOn ();

  // Get min-max range
  double min_max[2];
  xy_array->GetRange (min_max, 1);

  renwinint.xy_plot_->SetYTitle (""); renwinint.xy_plot_->SetXTitle ("");
  renwinint.xy_plot_->SetYRange (min_max[0], min_max[1]); 
  renwinint.xy_plot_->SetXRange (0, static_cast<double> (xy_array->GetNumberOfTuples () - 1));

  //renwinint.xy_plot_->SetTitle (id.c_str ());
  renwinint.xy_plot_->GetProperty ()->SetColor (0, 0, 0);

  // Adjust text properties
  vtkSmartPointer<vtkTextProperty> tprop = renwinint.xy_plot_->GetTitleTextProperty ();
  renwinint.xy_plot_->AdjustTitlePositionOn ();
  tprop->SetFontSize (8);
  tprop->ShadowOff (); tprop->ItalicOff ();
  tprop->SetColor (renwinint.xy_plot_->GetProperty ()->GetColor ());

  renwinint.xy_plot_->SetAxisLabelTextProperty (tprop);
  renwinint.xy_plot_->SetAxisTitleTextProperty (tprop);
  renwinint.xy_plot_->SetNumberOfXLabels (8);
  renwinint.xy_plot_->GetProperty ()->SetPointSize (3);
  renwinint.xy_plot_->GetProperty ()->SetLineWidth (2);

  renwinint.xy_plot_->SetPosition (0, 0);
  renwinint.xy_plot_->SetWidth (1); renwinint.xy_plot_->SetHeight (1);

  // Create the new window with its interactor and renderer
  renwinint.ren_->AddActor2D (renwinint.xy_plot_);
  renwinint.ren_->SetBackground (1, 1, 1);
  renwinint.win_->SetWindowName (id.c_str ());
  renwinint.win_->AddRenderer (renwinint.ren_);
  renwinint.win_->SetSize (win_width, win_height);
  renwinint.win_->SetBorders (1);
  
  // Create the interactor style
  vtkSmartPointer<pcl::visualization::PCLHistogramVisualizerInteractorStyle> style_ = vtkSmartPointer<pcl::visualization::PCLHistogramVisualizerInteractorStyle>::New ();
  style_->Initialize ();
  renwinint.style_ = style_;
  renwinint.style_->UseTimersOn ();
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  renwinint.interactor_ = vtkSmartPointer<PCLVisualizerInteractor>::New ();
#else
  renwinint.interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
#endif
  renwinint.interactor_->SetRenderWindow (renwinint.win_);
  renwinint.interactor_->SetInteractorStyle (renwinint.style_);
  // Initialize and create timer
  renwinint.interactor_->Initialize ();
  renwinint.interactor_->CreateRepeatingTimer (5000L);

  exit_main_loop_timer_callback_->right_timer_id = -1;
  renwinint.interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);
#if !((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  exit_callback_->his = this;
#endif
  renwinint.interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLHistogramVisualizer::addFeatureHistogram (
    const pcl::PCLPointCloud2 &cloud, const std::string &field_name,
    const std::string &id, int win_width, int win_height)
{
  // Get the field
  int field_idx = pcl::getFieldIndex (cloud, field_name);
  if (field_idx == -1)
  {
    PCL_ERROR ("[addFeatureHistogram] Invalid field (%s) given!", field_name.c_str ());
    return (false);
  }

  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    PCL_WARN ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (cloud.fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (unsigned int d = 0; d < cloud.fields[field_idx].count; ++d)
  {
    xy[0] = d;
    float data;
    // TODO: replace float with the real data type
    memcpy (&data, &cloud.data[cloud.fields[field_idx].offset + d * sizeof (float)], sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }
  RenWinInteract renwinint;
  createActor (xy_array, renwinint, id, win_width, win_height);

  // Save the pointer/ID pair to the global window map
  wins_[id] = renwinint;
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  resetStoppedFlag ();
#endif
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLHistogramVisualizer::addFeatureHistogram (
    const pcl::PCLPointCloud2 &cloud,
    const std::string &field_name, 
    const int index,
    const std::string &id, int win_width, int win_height)
{
  if (index < 0 || index >= static_cast<int> (cloud.width * cloud.height))
  {
    PCL_ERROR ("[addFeatureHistogram] Invalid point index (%d) given!\n", index);
    return (false);
  }

  // Get the field
  int field_idx = pcl::getFieldIndex (cloud, field_name);
  if (field_idx == -1)
  {
    PCL_ERROR ("[addFeatureHistogram] The specified field <%s> does not exist!\n", field_name.c_str ());
    return (false);
  }

  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    PCL_ERROR ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (cloud.fields[field_idx].count);

  // Compute the total size of the fields
  unsigned int fsize = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
    fsize += cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (unsigned int d = 0; d < cloud.fields[field_idx].count; ++d)
  {
    xy[0] = d;
    float data;
    // TODO: replace float with the real data type
    memcpy (&data, &cloud.data[index * fsize + cloud.fields[field_idx].offset + d * sizeof (float)], sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }
  RenWinInteract renwinint;
  createActor (xy_array, renwinint, id, win_width, win_height);

  // Save the pointer/ID pair to the global window map
  wins_[id] = renwinint;
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  resetStoppedFlag ();
#endif
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLHistogramVisualizer::updateFeatureHistogram (
    const pcl::PCLPointCloud2 &cloud, const std::string &field_name,
    const std::string &id)
{
  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it == wins_.end ())
  {
    PCL_WARN ("[updateFeatureHistogram] A window with id <%s> does not exists!.\n", id.c_str ());
    return (false);
  }
  RenWinInteract* renwinupd = &wins_[id];
  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);

  // Get the field
  int field_idx = pcl::getFieldIndex (cloud, field_name);
  if (field_idx == -1)
  {
    pcl::console::print_error ("[updateFeatureHistogram] Invalid field (%s) given!", field_name.c_str ());
    return (false);
  }
  xy_array->SetNumberOfTuples (cloud.fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (unsigned int d = 0; d < cloud.fields[field_idx].count; ++d)
  {
    xy[0] = d;
    float data;
    memcpy (&data, &cloud.data[cloud.fields[field_idx].offset + d * sizeof (float)], sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }
  reCreateActor(xy_array, renwinupd, cloud.fields[field_idx].count - 1);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLHistogramVisualizer::updateFeatureHistogram (
    const pcl::PCLPointCloud2 &cloud,
    const std::string &field_name, 
    const int index,
    const std::string &id)
{
  if (index < 0 || index >= static_cast<int> (cloud.width * cloud.height))
  {
    PCL_ERROR ("[updateFeatureHistogram] Invalid point index (%d) given!\n", index);
    return (false);
  }
  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it == wins_.end ())
  {
    PCL_WARN ("[updateFeatureHistogram] A window with id <%s> does not exists!.\n", id.c_str ());
    return (false);
  }
  RenWinInteract* renwinupd = &wins_[id];
  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);

  // Get the field
  int field_idx = pcl::getFieldIndex (cloud, field_name);
  if (field_idx == -1)
  {
    pcl::console::print_error ("[updateFeatureHistogram] Invalid field (%s) given!", field_name.c_str ());
    return (false);
  }
  xy_array->SetNumberOfTuples (cloud.fields[field_idx].count);
  
  // Compute the total size of the fields
  unsigned int fsize = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
    fsize += cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (unsigned int d = 0; d < cloud.fields[field_idx].count; ++d)
  {
    xy[0] = d;
    float data;
    memcpy (&data, &cloud.data[index * fsize + cloud.fields[field_idx].offset + d * sizeof (float)], sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }
  reCreateActor(xy_array, renwinupd, cloud.fields[field_idx].count - 1);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizer::ExitMainLoopTimerCallback::Execute (
    vtkObject*, unsigned long event_id, void* call_data)
{
  if (event_id != vtkCommand::TimerEvent)
    return;
  int timer_id = *(reinterpret_cast<int*> (call_data));

  if (timer_id != right_timer_id)
    return;

  // Stop vtk loop and send notification to app to wake it up
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
  interact->stopLoop ();
#else
  interact->TerminateApp ();
#endif
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizer::ExitCallback::Execute (
    vtkObject*, unsigned long event_id, void*)
{
  if (event_id != vtkCommand::ExitEvent)
    return;
  his->stopped_ = true;
}

