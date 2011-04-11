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
 */

/**
\author Bastian Steder
**/


#include <pcl/visualization/common/image_widget_wx.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <iostream>
using std::cout;
using std::cerr;

IMPLEMENT_APP_NO_MAIN (wxApp)

void 
  pcl_visualization::ImageWidgetWX::spinOnce ()
{
  if (wxTheApp != NULL)
    wxTheApp->Yield (true);
  
  //StopMainLoopTimer stopMainLoopTimer;
  //stopMainLoopTimer.Start(timeOut);
  //wxTheApp->OnRun(); 
  //stopMainLoopTimer.Stop();
}

void 
  pcl_visualization::ImageWidgetWX::spin ()
{
  wxTheApp->OnRun (); 
}

pcl_visualization::ImageWidgetWX::ImageWidgetWX () : keepAspectRatio (true), visualize_selected_point (false),
                                                     print_selected_point(false),
                                                     image_frame (NULL), image_data (NULL)
{
  //wxInitAllImageHandlers();
  if (wxTheApp == NULL)
  {
    //std::cout << "wxApp does not exist yet -> Calling wxEntryStart()\n";
    int fake_argc = 0;
    char** fake_argv = NULL;
    wxEntryStart(fake_argc, fake_argv);  // Initialize the wx application
  }
  else
  {
    //std::cout << "wxApp exists already.\n";
  }
}

pcl_visualization::ImageWidgetWX::~ImageWidgetWX ()
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  if (image_frame != NULL) 
  {
    image_frame->parentImageWidget = NULL;
    image_frame->Close (true);
    //image_frame->Destroy();
  }
  free (image_data);
}

void 
  pcl_visualization::ImageWidgetWX::reset ()
{
  mouse_click_happened = false;
  last_clicked_point_x = -1.0f;
  last_clicked_point_y = -1.0f;
  if (image_frame==NULL)
    image_frame = new ImageFrame (this);
  image_frame->image_panel->markedPoints.clear ();
  image_frame->image_panel->lines.clear ();
}

//void pcl_visualization::ImageWidgetWX::savePng (std::string filename) const
//{
  //image_frame->image_panel->SaveFile(filename.c_str());
//}

void 
  pcl_visualization::ImageWidgetWX::setSize (int width, int height)
{
  int original_width  = image_frame->image_panel->image->GetWidth (),
      original_height = image_frame->image_panel->image->GetHeight ();
  
  if (width <= 0  &&  height <= 0)
  {
    width = original_width;
    height=original_height;
  }
  else if (width <= 0)
  {
    width = pcl_lrint(float(original_width) * float(height)/float(original_height));
  }
  else if (height <= 0)
  {
    height = pcl_lrint(float(original_height) * float(width)/float(original_width));
  }
  
  image_frame->SetSize (wxSize (width, height));
}

void 
  pcl_visualization::ImageWidgetWX::setRGBImage (const unsigned char* data,
                                                 unsigned int width, unsigned int height, const char* name)
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  reset ();
  
  if (width==0 || height==0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": image has size 0.\n";
    return;
  }
  
  unsigned int arraySize = 3*width*height;
  if (data == NULL) arraySize=0;
  
  image_data = (unsigned char*)realloc(image_data, arraySize);
  memcpy (image_data, data, arraySize);
  
  image_frame->updateImage (image_data, width, height);
  image_frame->SetTitle (wxString (name, wxConvUTF8));
  image_frame->image_panel->resizeImage ();
  image_frame->Refresh ();
  image_frame->Show ();
}

void 
  pcl_visualization::ImageWidgetWX::setName (const std::string& name)
{
  image_frame->SetTitle (wxString (name.c_str(), wxConvUTF8));
  //image_frame->Refresh ();
}

void 
  pcl_visualization::ImageWidgetWX::setFloatImage (const float* float_image, unsigned int width, unsigned int height,
                                                   const char* name, float min_value, float max_value, bool grayscale)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (float_image, width, height,
                                                              min_value, max_value, grayscale);
  setRGBImage (rgb_image, width, height, name);
  delete[] rgb_image;
}

void 
  pcl_visualization::ImageWidgetWX::setAngleImage (const float* angle_image, unsigned int width, unsigned int height,
                                                   const char* name)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualAngleImage (angle_image, width, height);
  setRGBImage (rgb_image, width, height, name);
  delete[] rgb_image;
}

void 
  pcl_visualization::ImageWidgetWX::setHalfAngleImage (const float* angle_image, unsigned int width,
                                                       unsigned int height, const char* name)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualHalfAngleImage (angle_image, width, height);
  setRGBImage (rgb_image, width, height, name);
  delete[] rgb_image;
}

void 
  pcl_visualization::ImageWidgetWX::informAboutImageFrameDestruction () 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  image_frame = NULL;
}

void 
  pcl_visualization::ImageWidgetWX::markPoint (float x, float y, const wxPen* color, const wxBrush* background) 
{
  image_frame->image_panel->markedPoints.push_back (ImagePoint (x,y, color, background));
}

void 
  pcl_visualization::ImageWidgetWX::markLine (float x1, float y1, float x2, float y2, const wxPen* color) 
{
  image_frame->image_panel->lines.push_back (ImageLine (x1, y1, x2, y2, color));
}

bool 
  pcl_visualization::ImageWidgetWX::isShown () const 
{
  if (image_frame==NULL) return false;
  return image_frame->IsShown ();
}

void 
  pcl_visualization::ImageWidgetWX::addPixelClickedHandler (PixelClickedHandler pixel_clicked_handler)
{
  image_frame->image_panel->pixel_clicked_handlers.push_back (pixel_clicked_handler);
}

void 
  pcl_visualization::ImageWidgetWX::show (bool show_widget)
{
  if (image_frame != NULL)
    image_frame->Show (show_widget);
}


//==================================
//   IMAGE FRAME DEFINITION START
//==================================


pcl_visualization::ImageWidgetWX::ImageFrame::ImageFrame (ImageWidgetWX* parentImageWidget) :
  wxFrame (NULL, wxID_ANY, wxString (__func__, wxConvUTF8), wxPoint (50, 50), wxSize (800, 600)),
  parentImageWidget (parentImageWidget)
{
  image_panel = new ImagePanel (this);
  //wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
  //sizer->Add(image_panel, 1, wxEXPAND);
  //this->SetSizer(sizer);
}

pcl_visualization::ImageWidgetWX::ImageFrame::~ImageFrame() 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  if (parentImageWidget != NULL)
    parentImageWidget->informAboutImageFrameDestruction ();
  //delete image_panel;  // This is done by wxApp
}

BEGIN_EVENT_TABLE (pcl_visualization::ImageWidgetWX::ImageFrame, wxFrame)
  //EVT_PAINT(ImageWidgetWX::ImageFrame::paintEvent)
  EVT_SIZE (pcl_visualization::ImageWidgetWX::ImageFrame::OnSize)
  EVT_CLOSE (pcl_visualization::ImageWidgetWX::ImageFrame::OnClose)  // Subscribe for the close event
END_EVENT_TABLE ()

void 
  pcl_visualization::ImageWidgetWX::ImageFrame::OnSize(wxSizeEvent& event) 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  Fit ();
  event.Skip ();
}

void 
  pcl_visualization::ImageWidgetWX::ImageFrame::OnClose (wxCloseEvent& event) 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //this->Destroy();    // If not, destroy the window anyway.
  event.Skip ();
}

void 
  pcl_visualization::ImageWidgetWX::ImageFrame::updateImage (unsigned char* data,
                                                             unsigned int width, unsigned int height) 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //delete image_panel->image;  image_panel->image=NULL;
  
  delete image_panel->image;
  image_panel->image = NULL;
  
  if (data == NULL) return;
  
  bool letWxDeleteIt = false;
  image_panel->image = new wxImage (width, height, data, !letWxDeleteIt);
}


//==================================
//   IMAGE PANEL DEFINITION START
//==================================

BEGIN_EVENT_TABLE (pcl_visualization::ImageWidgetWX::ImagePanel, wxPanel)
/* some useful events
 EVT_MOTION(ImageWidgetWX::ImagePanel::mouseMoved)
 EVT_LEFT_DOWN(ImageWidgetWX::ImagePanel::mouseDown)
 EVT_RIGHT_DOWN(ImageWidgetWX::ImagePanel::rightClick)
 EVT_LEAVE_WINDOW(ImageWidgetWX::ImagePanel::mouseLeftWindow)
 EVT_KEY_DOWN(ImageWidgetWX::ImagePanel::keyPressed)
 EVT_KEY_UP(ImageWidgetWX::ImagePanel::keyReleased)
 EVT_MOUSEWHEEL(ImageWidgetWX::ImagePanel::mouseWheelMoved)
*/
EVT_LEFT_UP (pcl_visualization::ImageWidgetWX::ImagePanel::mouseReleased)
EVT_PAINT (pcl_visualization::ImageWidgetWX::ImagePanel::paintEvent)  // catch paint events
EVT_SIZE (pcl_visualization::ImageWidgetWX::ImagePanel::OnSize)  // Size event
END_EVENT_TABLE ()
/* some useful events
 void ImageWidgetWX::ImagePanel::mouseMoved(wxMouseEvent& event) {}
 void ImageWidgetWX::ImagePanel::mouseDown(wxMouseEvent& event) {}
 void ImageWidgetWX::ImagePanel::mouseWheelMoved(wxMouseEvent& event) {}
 void ImageWidgetWX::ImagePanel::rightClick(wxMouseEvent& event) {}
 void ImageWidgetWX::ImagePanel::mouseLeftWindow(wxMouseEvent& event) {}
 void ImageWidgetWX::ImagePanel::keyPressed(wxKeyEvent& event) {}
 void ImageWidgetWX::ImagePanel::keyReleased(wxKeyEvent& event) {}
*/

pcl_visualization::ImageWidgetWX::ImagePanel::ImagePanel (wxFrame* parent) : wxPanel (parent), image (NULL),
                                                                             scaledWidth (0), scaledHeight (0)
{
}

pcl_visualization::ImageWidgetWX::ImagePanel::~ImagePanel () 
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
}
 
/*
 * Called by the system by wxWidgets when the panel needs
 * to be redrawn. You can also trigger this call by
 * calling Refresh()/Update().
 */
void 
  pcl_visualization::ImageWidgetWX::ImagePanel::paintEvent (wxPaintEvent & evt)
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  // depending on your system you may need to look at double-buffered dcs
  wxPaintDC dc (this);
  render (dc);
}
 
void 
  pcl_visualization::ImageWidgetWX::ImagePanel::mouseReleased (wxMouseEvent& event)
{
  //std::cout << __PRETTY_FUNCTION__<<" called.\n";
  float clicked_pixel_x = (float)event.m_x * (float)image->GetWidth ()/(float)scaledWidth -0.5f,
        clicked_pixel_y = (float)event.m_y * (float)image->GetHeight ()/(float)scaledHeight -0.5f;
  getParentImageWidget()->mouse_click_happened = true;
  getParentImageWidget()->last_clicked_point_x = clicked_pixel_x;
  getParentImageWidget()->last_clicked_point_y = clicked_pixel_y;
  //std::cout << "New pixel was clicked: "<<clicked_pixel_x<<","
                                        //<<clicked_pixel_y<<".\n";
  
  if (getParentImageWidget()->print_selected_point)
    cout << "ImageWidgetWX: Clicked image point is " << clicked_pixel_x<<", "<<clicked_pixel_y<<".\n";
  // Call handler functions
  for (unsigned int i = 0; i < pixel_clicked_handlers.size(); ++i)
    pixel_clicked_handlers[i] (clicked_pixel_x, clicked_pixel_y);
  Refresh ();
}


/*
 * Alternatively, you can use a clientDC to paint on the panel
 * at any time. Using this generally does not free you from
 * catching paint events, since it is possible that e.g. the window
 * manager throws away your drawing when the window comes to the
 * background, and expects you will redraw it when the window comes
 * back (by sending a paint event).
 */
void 
  pcl_visualization::ImageWidgetWX::ImagePanel::paintNow ()
{
  // depending on your system you may need to look at double-buffered dcs
  wxClientDC dc (this);
  render (dc);
}
 
/*
 * Here we do the actual rendering. I put it in a separate
 * method so that it can work no matter what type of DC
 * (e.g. wxPaintDC or wxClientDC) is used.
 */
void 
  pcl_visualization::ImageWidgetWX::ImagePanel::render (wxDC&  dc)
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  if (image==NULL) return;
  
  int newWidth, newHeight;
  dc.GetSize (&newWidth, &newHeight);
  
  if (newWidth != scaledWidth || newHeight != scaledHeight) 
  {
    resizeImage (newWidth, newHeight);
  }
  dc.DrawBitmap (resized_, 0, 0, false);
  
  int circleSize = 4;
  for (unsigned int i = 0; i < markedPoints.size (); ++i) 
  {
    const ImagePoint& point = markedPoints.at (i);
    dc.SetPen (*point.color);
    dc.SetBrush (*point.background);
    dc.DrawEllipse (pcl_lrint ((point.x+0.5f)*scaledWidth / image->GetWidth ())-0.5f*circleSize,
                    pcl_lrint ((point.y+0.5f)*scaledHeight / image->GetHeight ())-0.5f*circleSize, circleSize, circleSize);
  }
  
  for (unsigned int i = 0; i < lines.size (); ++i) 
  {
    const ImageLine& line = lines.at (i);
    wxPoint points_array[2];
    points_array[0].x = pcl_lrint ((line.x1+0.5f)*scaledWidth / image->GetWidth ());
    points_array[0].y = pcl_lrint ((line.y1+0.5f)*scaledHeight / image->GetHeight ());
    points_array[1].x = pcl_lrint((line.x2+0.5f)*scaledWidth / image->GetWidth ());
    points_array[1].y = pcl_lrint((line.y2+0.5f)*scaledHeight / image->GetHeight ());
    wxPen pen(*line.color);
    pen.SetWidth(3);
    dc.SetPen(pen);
    dc.DrawLines (2, points_array);
  }
  
  if (getParentImageWidget()->visualize_selected_point)
  {
    int selected_x = getParentImageWidget()->last_clicked_point_x,
        selected_y = getParentImageWidget()->last_clicked_point_y;
    if (selected_x >= 0 && selected_y >= 0)
    {
      wxPen pen(*wxGREEN, 2);	 
      dc.SetPen (pen);
      dc.CrossHair (pcl_lrint((selected_x+0.5f)*scaledWidth / image->GetWidth ()),
                    pcl_lrint((selected_y+0.5f)*scaledHeight / image->GetHeight ()));
    }
  }
}
 
/*
 * Here we call refresh to tell the panel to draw itself again.
 * So when the user resizes the image panel the image should be resized too.
 */
void 
  pcl_visualization::ImageWidgetWX::ImagePanel::OnSize (wxSizeEvent& event) 
{
  event.Skip ();
  if (getParentImageWidget ()->keepAspectRatio) 
  {
    float aspectRatio = (float)image->GetWidth () / (float)image->GetHeight ();
    SetSize (wxDefaultCoord, wxDefaultCoord, event.GetSize ().GetWidth (),
             pcl_lrint((float)event.GetSize ().GetWidth () / aspectRatio));
  }
  Refresh ();
}

void 
  pcl_visualization::ImageWidgetWX::ImagePanel::resizeImage (int newWidth, int newHeight) 
{
  if (newWidth<=0 || newHeight<=0)  // No size given => Use current widget size
    this->GetSize (&newWidth, &newHeight);
  
  resized_ = wxBitmap (image->Scale (newWidth, newHeight));
  //resized_ = wxBitmap(image->Scale (newWidth, newHeight, wxIMAGE_QUALITY_HIGH));
  scaledWidth = newWidth;
  scaledHeight = newHeight;
  //cout << "Image has new size "<<scaledWidth<<"x"<<scaledHeight<<"\n";
}

