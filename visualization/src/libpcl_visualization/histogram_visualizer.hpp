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
 * $Id: pcl_histogram_visualizer.hpp 31709 2010-08-11 08:11:54Z rusu $
 *
 */

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl_visualization::PCLHistogramVisualizer::addFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, int hsize, 
    const std::string &id, int win_width, int win_height)
{
  RenWinInteractMap::iterator am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    terminal_tools::print_warn ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create the actor
  RenWinInteract renwinint;
  renwinint.xy_plot_->SetDataObjectPlotModeToColumns ();
  renwinint.xy_plot_->SetXValuesToValue ();

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (hsize);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (int d = 0; d < hsize; ++d)
  {
    xy[0] = d;
    xy[1] = cloud.points[0].histogram[d];
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
  renwinint.xy_plot_->SetXRange (0, hsize - 1);

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


