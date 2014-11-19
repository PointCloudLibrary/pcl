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
 *
 */

#include <pcl/visualization/pcl_painter2D.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLPainter2D::PCLPainter2D(char const * name)
{
  //construct
  view_ = vtkContextView::New ();
  current_pen_ = vtkPen::New ();
  current_brush_ = vtkBrush::New ();
  current_transform_ = vtkTransform2D::New ();
  exit_loop_timer_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New ();
  
  //connect
  view_->GetScene ()->AddItem (this);
  view_->GetRenderWindow ()->SetWindowName (name);
  
  exit_loop_timer_->interactor = view_->GetInteractor ();
  
  //defaulat state
  win_width_ = 640;
  win_height_ = 480;
  bkg_color_[0] = 1; bkg_color_[1] = 1; bkg_color_[2] = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addLine (float x1, float y1, float x2, float y2)
{
  std::vector<float> line (4);
  line[0] = x1;
  line[1] = y1;
  line[2] = x2;
  line[3] = y2;

  FPolyLine2D *pline = new FPolyLine2D(line, current_pen_, current_brush_, current_transform_);
  figures_.push_back (pline);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addLine (std::vector<float> p)
{
  figures_.push_back (new FPolyLine2D(p, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addPoint (float x, float y)
{
  std::vector<float> points(2);
  points[0] = x; points[1] = y;
  
  figures_.push_back (new FPoints2D(points, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addPoints (std::vector<float> p)
{
  figures_.push_back (new FPoints2D(p, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addRect (float x, float y, float width, float height)
{
  float p[] = { x,       y,
                x+width, y,
                x+width, y+height,
                x,       y+height};
  
  std::vector<float> quad (p, p+8);
  
  figures_.push_back (new FQuad2D(quad, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addQuad (std::vector<float> p)
{
  figures_.push_back (new FQuad2D(p, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addPolygon (std::vector<float> p)
{
  figures_.push_back (new FPolygon2D(p, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addEllipse (float x, float y, float rx, float ry)
{ 
  figures_.push_back (new FEllipticArc2D(x, y, rx, ry, 0, 360, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addCircle (float x, float y, float r)
{  
  figures_.push_back (new FEllipticArc2D(x, y, r, r, 0, 360, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addEllipticArc (float x, float y, float rx, float ry, float start_angle, float end_angle)
{ 
  figures_.push_back (new FEllipticArc2D(x, y, rx, ry, start_angle, end_angle, current_pen_, current_brush_, current_transform_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::addArc (float x, float y, float r, float start_angle, float end_angle)
{ 
  figures_.push_back (new FEllipticArc2D(x, y, r, r, start_angle, end_angle, current_pen_, current_brush_, current_transform_));
}


///////////////////////////////////////Pen and Brush functions////////////////////////////////////////////////////////
void pcl::visualization::PCLPainter2D::setPenColor (unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  current_pen_->SetColor (r, g, b, a);
}

void pcl::visualization::PCLPainter2D::setPenWidth (float w)
{
  current_pen_->SetWidth (w);
}

void pcl::visualization::PCLPainter2D::setPenType (int type)
{
  current_pen_->SetLineType (type);
}

unsigned char* pcl::visualization::PCLPainter2D::getPenColor ()
{
  return current_pen_->GetColor ();
}

float pcl::visualization::PCLPainter2D::getPenWidth ()
{
  return current_pen_->GetWidth ();
}

int pcl::visualization::PCLPainter2D::getPenType ()
{
  return current_pen_->GetLineType ();
}
 
void pcl::visualization::PCLPainter2D::setBrushColor (unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  current_brush_->SetColor (r, g, b, a);
}

unsigned char* pcl::visualization::PCLPainter2D::getBrushColor ()
{
  return current_brush_->GetColor ();
}

void pcl::visualization::PCLPainter2D::setPen (vtkPen *pen)
{
  current_pen_->DeepCopy (pen);
}

vtkPen * pcl::visualization::PCLPainter2D::getPen ()
{
  return current_pen_;
}

void pcl::visualization::PCLPainter2D::setBrush (vtkBrush *brush)
{
  current_brush_->DeepCopy (brush);
}

vtkBrush * pcl::visualization::PCLPainter2D::getBrush ()
{
  return current_brush_;
}
//////////////////////////////////////////End of Pen and Brush functions//////////////////////////////////////////////

void 
pcl::visualization::PCLPainter2D::translatePen (double x, double y)
{
  current_transform_->Translate(x, y);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::rotatePen (double angle)
{
  current_transform_->Rotate(angle);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::scalePen (double x, double y)
{
  current_transform_->Scale(x, y);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::setTransform(vtkMatrix3x3 *matrix)
{
  current_transform_->SetMatrix(matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vtkMatrix3x3 * 
pcl::visualization::PCLPainter2D::getTransform()
{
  return current_transform_->GetMatrix ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::clearTransform()
{
  current_transform_->Identity();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::clearFigures()
{
  figures_.clear();
  view_->GetScene()->SetDirty(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLPainter2D::display ()
{
  view_->GetRenderer ()->SetBackground (bkg_color_[0], bkg_color_[1], bkg_color_[2]);
  view_->GetRenderWindow ()->SetSize (win_width_, win_height_);
  
  //vtkOpenGLContextDevice2D::SafeDownCast (view_->GetContext ()->GetDevice ())->SetStringRendererToFreeType ();
  //view_->GetRenderWindow ()->SetMultiSamples (3);
  
  view_->GetInteractor ()->Initialize ();
  view_->GetRenderer ()->Render ();
  view_->GetInteractor ()->Start ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::spinOnce( const int spin_time )
{
  //apply current states
  view_->GetRenderer ()->SetBackground (bkg_color_[0], bkg_color_[1], bkg_color_[2]);
  view_->GetRenderWindow ()->SetSize (win_width_, win_height_);
  
  //start timer to spin
  if (!view_->GetInteractor ()->GetEnabled ())
  {
    view_->GetInteractor ()->Initialize ();
    view_->GetInteractor ()->AddObserver ( vtkCommand::TimerEvent, exit_loop_timer_ );
  }
  exit_loop_timer_->right_timer_id = view_->GetInteractor()->CreateOneShotTimer( spin_time );
  
  //start spinning
  this->Update();
  view_->GetRenderer ()->Render ();
	view_->GetInteractor()->Start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PCLPainter2D::spin()
{
  this->display();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLPainter2D::setBackgroundColor (const double r, const double g, const double b)
{
  bkg_color_[0] = r;
  bkg_color_[1] = g;
  bkg_color_[2] = b;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLPainter2D::setBackgroundColor (const double color[3])
{
  bkg_color_[0] = color[0];
  bkg_color_[1] = color[1];
  bkg_color_[2] = color[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double*
pcl::visualization::PCLPainter2D::getBackgroundColor ()
{
  double *bc = new double[3];
  bc[0] = bkg_color_[0];
  bc[1] = bkg_color_[1];
  bc[2] = bkg_color_[2];
  return (bc);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLPainter2D::setWindowSize (int w, int h)
{
  win_width_ = w;
  win_height_ = h;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int*
pcl::visualization::PCLPainter2D::getWindowSize ()
{
  int *sz = new int[2];
  sz[0] = win_width_;
  sz[1] = win_height_;
  return (sz);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::visualization::PCLPainter2D::Paint (vtkContext2D *painter)
{  
  //draw every figures
  for (size_t i = 0; i < figures_.size (); i++)
  {
    figures_[i]->draw (painter); 	//thats it ;-)
  }
  
  return true;
}


