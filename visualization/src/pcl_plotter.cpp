 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
  */

#include <pcl/visualization/pcl_plotter.h>


#define VTK_CREATE(type, name) \
  vtkSmartPointer<type> name = vtkSmartPointer<type>::New()


pcl::visualization::PCLPlotter::PCLPlotter() 
{
  view_=vtkSmartPointer<vtkContextView>::New();
  //chart_=vtkSmartPointer<vtkChartXY>::New();
  view_->GetScene()->AddItem(this);
}

void 
pcl::visualization::PCLPlotter::addPlotData(float const* array_X, float const* array_Y, int size, char const* color)
{
  //transforming data to be fed to the vtkChartXY
  VTK_CREATE(vtkTable, table);  
  
  VTK_CREATE(vtkFloatArray, varray_X);
  varray_X->SetName("X Axis");
  varray_X->SetArray((float *)array_X,size,1);
  table->AddColumn(varray_X);
  
  VTK_CREATE(vtkFloatArray, varray_Y);
  varray_Y->SetName("Y Axis");
  varray_Y->SetArray((float *)array_Y,size,1);
  table->AddColumn(varray_Y);
  
  //adding to chart: default values
  //vtkPlot *line = chart_->AddPlot(vtkChart::LINE);
  vtkPlot *line = this->AddPlot(vtkChart::LINE);
  line->SetInput(table, 0, 1);
  line->SetWidth(1);
  
  if(color==NULL)
    line->SetColor(255,0 , 0, 255);
  else 
    line->SetColor(color[0],color[1],color[2],color[3]);
}

void 
pcl::visualization::PCLPlotter::addPlotData(std::vector<float> const &array_X, std::vector<float> const &array_Y, std::vector<char> const &color)
{
  this->addPlotData(&array_X[0],&array_Y[0],(int)array_X.size(),(color.size()==0) ?NULL : &color[0]);
}

void 
pcl::visualization::PCLPlotter::plot()
{
  view_->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
  view_->GetRenderWindow()->SetSize(400, 300);
  view_->GetInteractor()->Start();
}