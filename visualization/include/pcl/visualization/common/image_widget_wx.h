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
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_WXWIDGETS

#ifndef PCL_VISUALIZATION_IMAGE_WIDGET_WX_H_
#define PCL_VISUALIZATION_IMAGE_WIDGET_WX_H_

#include <wx/wx.h>
#include <string>
#include <vector>
#include <limits>

namespace pcl
{
  namespace visualization
  {
    /** @b This class encapsulate a wxWidget to visualize an image.
     *  Warning: This header calls IMPLEMENT_APP_NO_MAIN(wxApp). If you do not want this
     *              (because you handle the wxApp yourself) include the header like this:
     *  #define DO_NOT_CALL_WX_IMPLEMENT_APP 0
     *  #include "range_image_visualizer.h"
     *  #undef DO_NOT_CALL_WX_IMPLEMENT_APP 
     *  Please call wxEntryCleanup(); when you do not need the wxApp anymore.
     * \author Bastian Steder
     * \ingroup visualization
     */
    class ImageWidgetWX
    {
      public:
        // =====CONSTRUCTOR & DESTRUCTOR=====
        //! Constructor
        ImageWidgetWX ();
        //! Destructor
        ~ImageWidgetWX ();
        
        // =====STATIC METHODS=====
        // Handle the GUI events and return immediately
        static void
          spinOnce ();
        // Start loop that handles the GUI events
        static void
          spin ();
        
        // =====TYPEDEFS=====
        typedef void (*PixelClickedHandler)(float pixel_x, float pixel_y);
        
        // =====PUBLIC METHODS=====
        //! Set the name (caption) of the widget
        void
          setName (const std::string& name);
        
        //! Visualize a RGB image
        void
          setRGBImage (const unsigned char* data, unsigned int width, unsigned int height, const char* name="RGB image");
        
        //! Visualize a float image
        void 
          setFloatImage (const float* float_image, 
                         unsigned int width, unsigned int height, 
                         const char* name="float image", 
                         float min_value = -std::numeric_limits<float>::infinity (), 
                         float max_value =  std::numeric_limits<float>::infinity (), bool grayscale=false);
        
        //! Visualize an angle image (values in rad!)
        void
          setAngleImage (const float* angle_image, unsigned int width, unsigned int height, const char* name="angle image");
        
        //! Visualize an angle image with a -90,90deg wrap-around (values in rad!)
        void
          setHalfAngleImage (const float* angle_image, unsigned int width, unsigned int height, const char* name="angle image");
        
        //! Marks a point in the image by drawing a small circle around it
        void
          markPoint (float x, float y, const wxPen* color=wxGREEN_PEN, const wxBrush* background=wxTRANSPARENT_BRUSH);
        
        //! Marks a line in the image
        void
          markLine (float x1, float y1, float x2, float y2, const wxPen* color=wxGREEN_PEN);

        
        //! Returns false if the widget is still active and true if it was closed
        bool
          isShown () const;

        //! Show or hide the widget
        void
          show (bool show_widget=true);
        
        /** Subscribe a handler that will be called, when a point in the image is (left) clicked. The
         * E.g.: void pixelClickedHandler(float pixel_x, float pixel_y) { doSomething(); }
         * The pixel is in the original image, not in the scaled one! */
        void
          addPixelClickedHandler (PixelClickedHandler pixel_clicked_handler);
        
        /** Set the size of the window. If you give no values it will resize to the original image size and if you
         *  leave one value -1 it will keep the aspect ratio (The latter will always be the case if keepAspectRatio is true) */
        void
          setSize (int width=-1, int height=-1);

        //! Just ignore this function. For internal use only
        void
          informAboutImageFrameDestruction ();
        
        // =====PUBLIC MEMBER VARIABLES=====
        //! Set this to false if you want to scale your window without keeping the original aspect ratio of the image
        bool keepAspectRatio;
        
        /** This value is set to true every time the image was clicked.
         *  The pixel position (in the original image, not the scaled one!)
         *  is written into last_clicked_point_x, last_clicked_point_y.
         *  You can use this if you don't want to register a handler function by setting it to false manually, after you handled a mouse event. */
        bool mouse_click_happened;
        float last_clicked_point_x, last_clicked_point_y;
        bool visualize_selected_point;
        bool print_selected_point;
        
        // =====EVENTS=====
        //! Do not call this! For internal use only
        void
          OnClose (wxCloseEvent& event);

      protected:
        // =====CLASSES / STRUCTS=====
        struct ImagePoint 
        {
          ImagePoint (float x_, float y_, const wxPen* color_=wxGREEN_PEN, const wxBrush* background_=wxTRANSPARENT_BRUSH) : x(x_), y(y_), color(color_), background(background_) {}
          float x,y;
          const wxPen* color;
          const wxBrush* background;
        };
        struct ImageLine 
        {
          ImageLine (float x1, float y1, float x2, float y2, const wxPen* color=wxGREEN_PEN) : x1(x1), y1(y1), x2(x2), y2(y2), color(color) {}
          float x1, y1, x2, y2;
          const wxPen* color;
        };

        class ImagePanel : public wxPanel
        {
          public:
            // =====CONSTRUCTOR & DESTRUCTOR=====
            ImagePanel (wxFrame* parent);
            ~ImagePanel ();
            
            // =====EVENTS=====
            void
              paintEvent (wxPaintEvent & evt);
            void
              paintNow ();
            void
              OnSize (wxSizeEvent& event);
            void
              mouseReleased (wxMouseEvent& event);
            void
              render (wxDC& dc);
            void
              resizeImage (int newWidth=-1, int newHeight=-1);
            /* some useful events
             void mouseMoved(wxMouseEvent& event);
             void mouseDown(wxMouseEvent& event);
             void mouseWheelMoved(wxMouseEvent& event);
             void mouseReleased(wxMouseEvent& event);
             void rightClick(wxMouseEvent& event);
             void mouseLeftWindow(wxMouseEvent& event);
             void keyPressed(wxKeyEvent& event);
             void keyReleased(wxKeyEvent& event);
             */
            
            // =====PUBLIC MEMBER VARIABLES=====
            wxImage* image;
            int scaledWidth, scaledHeight;
            std::vector<ImagePoint> markedPoints;
            std::vector<ImageLine> lines;
            std::vector<PixelClickedHandler> pixel_clicked_handlers;
          protected:
            // =====PROTECTED MEMBER VARIABLES=====
            wxBitmap resized_;
            // =====PROTECTED METHODS=====
            ImageWidgetWX*
              getParentImageWidget () { return ((ImageFrame*)GetParent ())->parentImageWidget; }
            
          DECLARE_EVENT_TABLE ();
        };
        
        class ImageFrame : public wxFrame
        {
          public:
            // =====TYPEDEFS=====
            typedef wxFrame BaseClass;
            // =====CONSTRUCTOR & DESTRUCTOR=====
            ImageFrame (ImageWidgetWX* parentImageWidget);
            ~ImageFrame ();
            
            // =====EVENTS=====
            void
              OnSize (wxSizeEvent& event);
            
            // =====METHODS=====
            void
              updateImage (unsigned char* data, unsigned int width, unsigned int height);
            
            // =====EVENTS=====
            void
              OnClose (wxCloseEvent& event);
            
            // =====PUBLIC MEMBER VARIABLES=====
            ImagePanel* image_panel;
            ImageWidgetWX* parentImageWidget;
            bool mouse_click_happened;
            float last_clicked_point_x, last_clicked_point_y;
            
          protected:
            // =====PROTECTED MEMBER VARIABLES=====
            
            DECLARE_EVENT_TABLE ();
        };
        
        // =====PROTECTED METHODS=====
        void
          reset ();
        
        // =====PROTECTED MEMBER VARIABLES=====
        ImageFrame* image_frame;
        unsigned char* image_data;
    };
  }  // namespace end
}

#endif  //#ifndef PCL_VISUALIZATION_IMAGE_WIDGET_WX_H_
#endif  //#ifdef HAVE_WXWIDGETS
