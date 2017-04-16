/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkContext2D.h>
#include <vtkImageData.h>
#include <vtkPen.h>
#include <vtkBrush.h>
#include <vtkTextProperty.h>
#if VTK_MAJOR_VERSION >= 6
#include <vtkContextDevice2D.h>
#include <vtkContextScene.h>
#include <vtkTransform2D.h>
#include <vtkViewport.h>
#include <vtkWindow.h>
#endif

#include <pcl/visualization/vtk/pcl_context_item.h>

namespace pcl
{
  namespace visualization
  {
    // Standard VTK macro for *New ()
    vtkStandardNewMacro (PCLContextItem);
    vtkStandardNewMacro (PCLContextImageItem);
#if VTK_MAJOR_VERSION >= 6
    vtkStandardNewMacro (PCLContextActor);
    bool PCLContextActor::initial_viewport_set_ = false;
    int PCLContextActor::initial_viewport_x_size_ = 0;
    int PCLContextActor::initial_viewport_y_size_ = 0;
    double PCLContextActor::initial_viewport_x_aspect_ = 0.0;
    double PCLContextActor::initial_viewport_y_aspect_ = 0.0;
#endif
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
      vtkStandardNewMacro (Text);
      vtkStandardNewMacro (Markers);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Circle::set (float x, float y, float radius)
{
  params.resize (4);
  params[0] = x; params[1] = y; params[2] = radius; params[3] = radius - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Rectangle::set (float x, float y, float w, float h)
{
  params.resize (4);
  params[0] = x; params[1] = y; params[2] = w; params[3] = h;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Line::set (float start_x, float start_y, float end_x, float end_y)
{
  params.resize (4);
  params[0] = start_x; params[1] = start_y; params[2] = end_x; params[3] = end_y;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Text::set (float x, float y, const std::string& _text)
{
  params.resize (2);
  params[0] = x; params[1] = y;
  text = _text;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Circle::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawWedge (params[0], params[1], params[2], params[3], 0.0, 360.0);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Disk::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawEllipse (params[0], params[1], params[2], params[2]);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Rectangle::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  float p[] = 
  { 
    params[0], params[1], 
    params[2], params[1],
    params[2], params[3],
    params[0], params[3],
    params[0], params[1]
  };

  painter->DrawPoly (p, 5);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::FilledRectangle::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawRect (params[0], params[1], params[2], params[3]);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Line::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawLine (params[0], params[1], params[2], params[3]);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Polygon::Paint (vtkContext2D *painter)
{
  painter->GetBrush ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawPolygon (&params[0], static_cast<int> (params.size () / 2));
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Point::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawPoint (params[0], params[1]);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Points::Paint (vtkContext2D *painter)
{
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawPoints (&params[0], static_cast<int> (params.size () / 2));
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Text::Paint (vtkContext2D *painter)
{
  vtkTextProperty *text_property = painter->GetTextProp ();
  text_property->SetColor (255.0 * colors[0], 255.0 * colors[1], 255.0 * colors[2]);
  text_property->SetOpacity (GetOpacity ());
  text_property->SetFontFamilyToArial ();
  text_property->SetFontSize (10);
  text_property->SetJustificationToLeft ();
  text_property->BoldOff ();
  text_property->ShadowOff ();
  painter->DrawString (params[0], params[1], text.c_str ());
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Markers::setPointColors (unsigned char r, unsigned char g, unsigned char b)
{
  point_colors[0] = r; point_colors[1] = g; point_colors[2] = b;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::context_items::Markers::setPointColors (unsigned char rgb[3])
{
  memcpy (point_colors, rgb, 3 * sizeof (unsigned char));
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::context_items::Markers::Paint (vtkContext2D *painter)
{
  int nb_points (params.size () / 2);
  if (size <= 0)
    size = 2.3 * painter->GetPen ()->GetWidth ();

  painter->GetPen ()->SetWidth (size);
  painter->GetPen ()->SetColor (colors[0], colors[1], colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawPointSprites (0, &params[0], nb_points);
  painter->GetPen ()->SetWidth (1);
  painter->GetPen ()->SetColor (point_colors[0], point_colors[1], point_colors[2], static_cast<unsigned char> ((255.0 * GetOpacity ())));
  painter->DrawPointSprites (0, &params[0], nb_points);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLContextImageItem::PCLContextImageItem ()
{
  image = vtkSmartPointer<vtkImageData>::New ();
}

#if VTK_MAJOR_VERSION >= 6
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLContextActor::PCLContextActor ()
{
}

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PCLContextActor::RenderOverlay (vtkViewport *viewport)
{
  vtkDebugMacro(<< "pcl::visualization::PCLContextActor::RenderOverlay");

  if (!this->Context.GetPointer ())
  {
    vtkErrorMacro(<< "pcl::visualization::PCLContextActor::Render - No painter set");
    return 0;
  }

  if (!this->Initialized)
  {
    this->Initialize(viewport);
  }

  const int *viewport_size = viewport->GetVTKWindow()->GetSize();

  // initial viewport size for scaling and positioning all later created context items
  if (!initial_viewport_set_)
  {
    initial_viewport_x_size_ = viewport_size[0];
    initial_viewport_y_size_ = viewport_size[1];
  }

  // scale factors for actual viewport size
  double x_scale = (static_cast<double>(viewport_size[0]) / static_cast<double>(initial_viewport_x_size_));
  double y_scale = (static_cast<double>(viewport_size[1]) / static_cast<double>(initial_viewport_y_size_));

  double aspect[2];
  viewport->GetAspect (aspect);

  // initial viewport aspect ratio
  if (!initial_viewport_set_)
  {
    initial_viewport_x_aspect_ = aspect[0];
    initial_viewport_y_aspect_ = aspect[1];
    initial_viewport_set_ = true;
  }
  double aspect_modification = initial_viewport_x_aspect_ * aspect[1] / (initial_viewport_y_aspect_ * aspect[0]);

  // initialize the drawing device
  this->GetContext ()->GetDevice ()->Begin (viewport);
  vtkTransform2D *transform = this->Scene->GetTransform ();
  transform->Identity();

  // emulate the window resizing referring to 'vtkImageSlice': complex composition of scaling and translating
  transform->Scale (x_scale * aspect_modification, y_scale);
  transform->Translate ((1.0 - aspect_modification) * static_cast<double>(viewport_size[0])/2.0 
    * static_cast<double>(initial_viewport_y_size_)/static_cast<double>(viewport_size[1]), 0.0);

  this->Scene->SetTransform (transform);
  this->Scene->Paint(this->Context.GetPointer());
  this->GetContext ()->GetDevice()->End();

  return 1;
  //return Superclass::RenderOverlay (viewport);
}
#endif
