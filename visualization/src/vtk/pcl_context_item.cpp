/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#include <pcl/visualization/vtk/pcl_context_item.h>

namespace pcl
{
  namespace visualization
  {
    // Standard VTK macro for *New ()
    vtkStandardNewMacro (PCLContextItem);
    vtkStandardNewMacro (PCLContextImageItem);
    namespace context_items
    {
      vtkStandardNewMacro (Point);
      vtkStandardNewMacro (Line);
      vtkStandardNewMacro (Circle);
      vtkStandardNewMacro (Disk);
      vtkStandardNewMacro (Rectangle);
      vtkStandardNewMacro (FilledRectangle);
      vtkStandardNewMacro (Points);
      vtkStandardNewMacro (Polygon);
    }
  }
}

void
pcl::visualization::PCLContextItem::setColors (unsigned char r, unsigned char g, unsigned char b)
{
  colors[0] = r; colors[1] = g; colors[2] = b;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLContextImageItem::set (float _x, float _y, vtkImageData *_image)
{
  x = _x;
  y = _y;
  image->DeepCopy (_image);
}

bool
pcl::visualization::PCLContextImageItem::Paint (vtkContext2D *painter)
{
  SetOpacity (1.0);
  painter->DrawImage (x, y, image);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Point::set (float x, float y)
{
  params.resize (2);
  params[0] = x; params[1] = y;
}

void
pcl::visualization::context_items::Circle::set (float x, float y, float radius)
{
  params.resize (4);
  params[0] = x; params[1] = y; params[2] = radius; params[3] = radius - 1;
}

void
pcl::visualization::context_items::Rectangle::set (float x, float y, float w, float h)
{
  params.resize (4);
  params[0] = x; params[1] = y; params[2] = w; params[3] = h;
}

void
pcl::visualization::context_items::Line::set (float start_x, float start_y, float end_x, float end_y)
{
  params.resize (4);
  params[0] = start_x; params[1] = start_y; params[2] = end_x; params[3] = end_y;
}

bool
pcl::visualization::context_items::Circle::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors);
  painter->GetBrush ()->SetColor (colors);
  painter->DrawWedge (params[0], params[1], params[2], params[3], 0.0, 360.0);
  return (true);
}

bool
pcl::visualization::context_items::Disk::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors);
  painter->GetPen ()->SetColor (colors);
  painter->DrawEllipse (params[0], params[1], params[2], params[2]);
  return (true);
}

bool
pcl::visualization::context_items::Rectangle::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors);
  float p[] = 
  { 
    params[0], params[1], 
    params[0]+params[2], params[1],
    params[0]+params[2], params[1]+params[3],
    params[0],       params[1]+params[3],
    params[0], params[1]
  };

  painter->DrawPoly (p, 5);
  return (true);
}

bool
pcl::visualization::context_items::FilledRectangle::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors);
  painter->GetPen ()->SetColor (colors);
  painter->DrawRect (params[0], params[1], params[2], params[3]);
  return (true);
}

bool
pcl::visualization::context_items::Line::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors);
  painter->DrawLine (params[0], params[1], params[2], params[3]);
  return (true);
}

bool
pcl::visualization::context_items::Polygon::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors);
  painter->GetPen ()->SetColor (colors);
  painter->DrawPolygon (&params[0], static_cast<int> (params.size () / 2));
  return (true);
}

bool
pcl::visualization::context_items::Point::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors);
  painter->DrawPoint (params[0], params[1]);
  return (true);
}

bool
pcl::visualization::context_items::Points::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors);
  painter->DrawPoints (&params[0], static_cast<int> (params.size () / 2));
  return (true);
}
