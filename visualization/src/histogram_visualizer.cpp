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
 * $Id$
 *
 */

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/interactor.h>
#include <pcl/visualization/histogram_visualizer.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_visualization::PCLHistogramVisualizer::PCLHistogramVisualizer () : exit_main_loop_timer_callback_ (vtkSmartPointer<ExitMainLoopTimerCallback>::New ()), exit_callback_ (vtkSmartPointer<ExitCallback>::New ())
{
/*  // Create a Renderer
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  // Add it to the list of renderers
  rens_->AddItem (ren);
  
  // Create a RendererWindow
  win_ = vtkSmartPointer<vtkRenderWindow>::New ();
  win_->SetWindowName (name.c_str ());

  // Get screen size
  int *scr_size = win_->GetScreenSize ();
  // Set the window size as 1/2 of the screen size
  win_->SetSize (scr_size[0] / 2, scr_size[1] / 2);
  
  // Add all renderers to the window
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  while ((renderer = rens_->GetNextItem ()) != NULL)
    win_->AddRenderer (renderer);

  // Create the interactor style
  style_->Initialize ();
  style_->setRendererCollection (rens_);
  style_->UseTimersOn ();

  interactor_->SetInteractorStyle (style_);
*/
  resetStoppedFlag ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Spin once method. Calls the interactor and updates the screen once. */
void
pcl_visualization::PCLHistogramVisualizer::spinOnce (int time, bool force_redraw)
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
      // Set the correct interactor to both callbacks
      exit_callback_->interact = (*am_it).second.interactor_;
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

      // Set the correct interactor to both callbacks
      exit_callback_->interact = (*am_it).second.interactor_;
      exit_main_loop_timer_callback_->interact = (*am_it).second.interactor_;

      (*am_it).second.interactor_->Start ();
      (*am_it).second.interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLHistogramVisualizer::spin ()
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
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }
  while (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLHistogramVisualizer::wasStopped ()
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
pcl_visualization::PCLHistogramVisualizer::resetStoppedFlag () 
{ 
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    (*am_it).second.interactor_->stopped = false; 
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLHistogramVisualizer::setBackgroundColor (const double &r, const double &g, const double &b, int viewport)
{
/*  rens_->InitTraversal ();
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
  }*/
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLHistogramVisualizer::setGlobalYRange (float minp, float maxp)
{
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
  {
    (*am_it).second.xy_plot_->SetYRange (minp, maxp);
    (*am_it).second.xy_plot_->Modified ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLHistogramVisualizer::updateWindowPositions ()
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

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLHistogramVisualizer::addFeatureHistogram (
    const sensor_msgs::PointCloud2 &cloud, const std::string &field_name, 
    const std::string &id, int win_width, int win_height)
{
  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    terminal_tools::print_error ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create the actor
  RenWinInteract renwinint;
  renwinint.xy_plot_->SetDataObjectPlotModeToColumns ();
  renwinint.xy_plot_->SetXValuesToValue ();

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);

  // Get the field
  int field_idx = pcl::getFieldIndex (cloud, field_name);
  if (field_idx == -1)
  {
    terminal_tools::print_error ("[addFeatureHistogram] Invalid field (%s) given!", field_name.c_str ());
    return (false);
  }
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
  double min_max[2];
  xy_array->GetRange (min_max, 1);

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

  renwinint.xy_plot_->SetYTitle (""); renwinint.xy_plot_->SetXTitle ("");
  renwinint.xy_plot_->SetYRange (min_max[0], min_max[1]); 
  renwinint.xy_plot_->SetXRange (0, cloud.fields[field_idx].count - 1);

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
  vtkSmartPointer<pcl_visualization::PCLHistogramVisualizerInteractorStyle> style_ = vtkSmartPointer<pcl_visualization::PCLHistogramVisualizerInteractorStyle>::New ();
  style_->Initialize ();
  renwinint.style_ = style_;
  renwinint.style_->UseTimersOn ();
  renwinint.interactor_ = vtkSmartPointer<PCLVisualizerInteractor>::New ();
  renwinint.interactor_->SetRenderWindow (renwinint.win_);
  renwinint.interactor_->SetInteractorStyle (renwinint.style_);
  // Initialize and create timer
  renwinint.interactor_->Initialize ();
  renwinint.interactor_->CreateRepeatingTimer (5000L);

  exit_main_loop_timer_callback_->right_timer_id = -1;
  renwinint.interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

  renwinint.interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);

  // Save the pointer/ID pair to the global window map
  wins_[id] = renwinint;

  resetStoppedFlag ();
  return (true);
}

pcl_visualization::PCLHistogramVisualizer::~PCLHistogramVisualizer ()
{
  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
     {
       (*am_it).second.interactor_->DestroyTimer ( (*am_it).second.interactor_->timer_id_);
     }
  }


