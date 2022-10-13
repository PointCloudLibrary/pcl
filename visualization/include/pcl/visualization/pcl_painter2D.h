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

#pragma once

#include <vector>
#include <pcl/pcl_exports.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkContext2D.h>
#include <vtkTransform2D.h>
#include <vtkContextItem.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>
#include <vtkBrush.h>
#include <vtkTextProperty.h>
#include <vtkOpenGLContextDevice2D.h>
#include <vtkPoints2D.h>
#include "vtkCommand.h"

namespace pcl
{
  namespace visualization
  {

   /** \brief Abstract class for storing figure information. All the derived class uses the same method draw() to invoke different drawing function of vtkContext2D
     * \author Kripasindhu Sarkar
     * \ingroup visualization
     */
    struct Figure2D
    {
      std::vector<float> info_;     //information stored in a general form for every object
      vtkPen *pen_;                 //the corresponding pen and brush for the figure
      vtkBrush *brush_;
      vtkTransform2D *transform_;
      
      Figure2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t)
      {
        this->pen_ = vtkPen::New ();
        this->brush_ = vtkBrush::New ();
        this->transform_ = vtkTransform2D::New();

        this->pen_->DeepCopy (p);
        this->brush_->DeepCopy (b);
        this->transform_->SetMatrix (t->GetMatrix());
        this->info_ = info; //note: it copies :-)
      }

      Figure2D (vtkPen *p, vtkBrush * b, vtkTransform2D *t)
      {
        this->pen_ = vtkPen::New ();
        this->brush_ = vtkBrush::New ();
        this->transform_ = vtkTransform2D::New();

        this->pen_->DeepCopy (p);
        this->brush_->DeepCopy (b);
        this->transform_->SetMatrix (t->GetMatrix());
      }

      virtual ~Figure2D()
      {
        pen_->Delete();
        brush_->Delete();
        transform_->Delete();
      }
      
      void applyInternals (vtkContext2D *painter) const
      {
        painter->ApplyPen (pen_);
        painter->ApplyBrush (brush_);
        painter->GetDevice ()->SetMatrix (transform_->GetMatrix());
      }
		  
      virtual void draw (vtkContext2D *) {}
    };
    
   /** \brief Class for PolyLine
     */
    struct FPolyLine2D : public Figure2D
    {

      FPolyLine2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (info, p, b, t){}

      void draw (vtkContext2D * painter) override
      {
        applyInternals(painter);  
        painter->DrawPoly (&info_[0], static_cast<unsigned int> (info_.size ()) / 2);
      }
    };

   /** \brief Class for storing Points
     */
    struct FPoints2D : public Figure2D
    {

      FPoints2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (info, p, b, t) {}

      void draw (vtkContext2D * painter) override
      {
        applyInternals(painter);  
        painter->DrawPoints (&info_[0], static_cast<unsigned int> (info_.size ()) / 2);
      }
    };

   /** \brief Class for storing Quads
     */
    struct FQuad2D : public Figure2D
    {

      FQuad2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (info, p, b, t) {}

      void draw (vtkContext2D * painter) override
      {
        applyInternals(painter);  
        painter->DrawQuad (&info_[0]);
      }
    };
    
    /** \brief Class for Polygon
     */
    struct FPolygon2D : public Figure2D
    {

      FPolygon2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (info, p, b, t){}

      void draw (vtkContext2D * painter) override
      {
        applyInternals(painter);  
        painter->DrawPolygon (&info_[0], static_cast<unsigned int> (info_.size ()) / 2);
      }
    };
    
   /** \brief Class for storing EllipticArc; every ellipse , circle are covered by this
     */
    struct FEllipticArc2D : public Figure2D
    {

      FEllipticArc2D (std::vector<float> info, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (info, p, b, t) {}

      FEllipticArc2D (float x, float y, float rx, float ry, float sa, float ea, vtkPen *p, vtkBrush * b, vtkTransform2D *t) : Figure2D (p, b, t)
      {
        info_.resize (6);
        info_[0] = x;
        info_[1] = y;
        info_[2] = rx;
        info_[3] = ry;
        info_[4] = sa;
        info_[5] = ea;
      }

      void draw (vtkContext2D * painter) override
      {
        applyInternals(painter);  
        painter->DrawEllipticArc (info_[0], info_[1], info_[2], info_[3], info_[4], info_[5]);
      }
    };


    ////////////////////////////////////The Main Painter Class begins here//////////////////////////////////////
    /** \brief PCL Painter2D main class. Class for drawing 2D figures
     * \author Kripasindhu Sarkar
     * \ingroup visualization
     */
    class PCL_EXPORTS PCLPainter2D: public vtkContextItem
    {
    public:

      //static PCLPainter2D *New();
      
      /** \brief Constructor of the class
       */
      PCLPainter2D (char const * name = "PCLPainter2D");
      vtkTypeMacro (PCLPainter2D, vtkContextItem);

      /** \brief Paint event for the chart, called whenever the chart needs to be drawn
       *  \param[in] painter Name of the window
       */
      bool 
      Paint (vtkContext2D *painter) override;

      /** \brief Draw a line between the specified points.
       * \param[in] x1 X coordinate of the starting point of the line
       * \param[in] y1 Y coordinate of the starting point of the line
       * \param[in] x2 X coordinate of the ending point of the line
       * \param[in] y2 Y coordinate of the ending point of the line
       */
      void 
      addLine (float x1, float y1, float x2, float y2);
      
      /** \brief Draw line(s) between the specified points 
       *  \param[in] p a vector of size 2*n and the points are packed x1, y1, x2, y2 etc.
       */
      void 
      addLine (std::vector<float> p);

      
      /** \brief Draw specified point(s).
       * \param[in] x X coordinate of the point
       * \param[in] y Y coordinate of the point
       */      
      void 
      addPoint (float x, float y);
      /** \brief Draw specified point(s).
       * \param[in] points a vector of size 2*n and the points are packed x1, y1, x2, y2 etc.
       */
      
      void 
      addPoints (std::vector<float> points);
      
      
      /** \brief Draw a rectangle based on the given points
       * \param[in] x X coordinate of the origin
       * \param[in] y Y coordinate of the origin
       * \param[in] width width of the rectangle
       * \param[in] height height of the rectangle
       */
      void 
      addRect (float x, float y, float width, float height);
      
      /** \brief Draw a quadrilateral based on the given points
       * \param[in] p a vector of size 8 and the points are packed x1, y1, x2, y2, x3, y3 and x4, y4.
       */
      void 
      addQuad (std::vector<float> p);
      
        /** \brief Draw a polygon between the specified points 
       *  \param[in] p a vector of size 2*n and the points are packed x1, y1, x2, y2 etc.
       */
      void 
      addPolygon (std::vector<float> p);

      
      /** \brief Draw an ellipse based on the inputs
       * \param[in] x X coordinate of the origin
       * \param[in] y Y coordinate of the origin
       * \param[in] rx X radius of the ellipse
       * \param[in] ry Y radius of the ellipse
       */
      void 
      addEllipse (float x, float y, float rx, float ry);
      
      /** \brief Draw a circle based on the inputs
       * \param[in] x X coordinate of the origin
       * \param[in] y Y coordinate of the origin
       * \param[in] r radius of the circle
       */
      void 
      addCircle (float x, float y, float r);
      
      /** \brief Draw an elliptic arc based on the inputs
       * \param[in] x X coordinate of the origin
       * \param[in] y Y coordinate of the origin
       * \param[in] rx X radius of the ellipse
       * \param[in] ry Y radius of the ellipse
       * \param[in] start_angle the starting angle of the arc expressed in degrees
       * \param[in] end_angle the ending angle of the arc expressed in degrees
       */
      void 
      addEllipticArc (float x, float y, float rx, float ry, float start_angle, float end_angle);
      
      /** \brief Draw an arc based on the inputs
       * \param[in] x X coordinate of the origin
       * \param[in] y Y coordinate of the origin
       * \param[in] r radius of the circle
       * \param[in] start_angle the starting angle of the arc expressed in degrees
       * \param[in] end_angle the ending angle of the arc expressed in degrees
       */
      void 
      addArc (float x, float y, float r, float start_angle, float end_angle);


      /** \brief Create a translation matrix and concatenate it with the current transformation.
       * \param[in] x translation along X axis
       * \param[in] y translation along Y axis
       */
      void 
      translatePen (double x, double y);
      
      /** \brief Create a rotation matrix and concatenate it with the current transformation.
       * \param[in] angle angle in degrees
       */
      void 
      rotatePen(double angle);
      
      /** \brief Create a scale matrix and concatenate it with the current transformation.
       * \param[in] x translation along X axis
       * \param[in] y translation along Y axis
       */
      void 
      scalePen(double x, double y);
      
      /** \brief Create a translation matrix and concatenate it with the current transformation.
       * \param[in] matrix the transformation matrix
       */
      void 
      setTransform(vtkMatrix3x3 *matrix);
      
      /** \brief Returns the current transformation matrix.
       */
      vtkMatrix3x3 * 
      getTransform();
      
      /** \brief Clears all the transformation applied. Sets the transformation matrix to Identity
       */
      void 
      clearTransform();
      
      /** \brief remove all the figures from the window
       */
       void
       clearFigures();

      /** \brief set/get methods for current working vtkPen
       */
      void setPenColor (unsigned char r, unsigned char g, unsigned char b, unsigned char a);
      void setPenWidth (float w);
      void setPenType (int type);

      /** \brief set/get methods for current working vtkPen
       */
      unsigned char* getPenColor ();
      float getPenWidth ();
      int getPenType ();
      void setPen (vtkPen *pen);
      vtkPen* getPen ();

      /** \brief set/get methods for current working vtkBrush
       */
      void setBrush (vtkBrush *brush);
      vtkBrush* getBrush ();
      void setBrushColor (unsigned char r, unsigned char g, unsigned char b, unsigned char a);
      unsigned char* getBrushColor ();

      /** \brief set/get method for the viewport's background color.
       * \param[in] r the red component of the RGB color
       * \param[in] g the green component of the RGB color
       * \param[in] b the blue component of the RGB color
       */
      void
      setBackgroundColor (const double r, const double g, const double b);

      /** \brief set/get method for the viewport's background color.
       * \param [in] color the array containing the 3 component of the RGB color
       */
      void
      setBackgroundColor (const double color[3]);

      /** \brief set/get method for the viewport's background color.
       * \return [out] color the array containing the 3 component of the RGB color
       */
      double *
      getBackgroundColor ();


      /** \brief set/get method for the window size.
       * \param[in] w the width of the window
       * \param[in] h the height of the window
       */
      void
      setWindowSize (int w, int h);

      /** \brief set/get method for the window size.
       * \return[in] array containing the width and height of the window
       */
      int *
      getWindowSize () const;

      /** \brief displays all the figures added in a window.
       */    
      void display ();
      
      /** \brief spins (runs the event loop) the interactor for spin_time amount of time. The name is confusing and will be probably obsolete in the future release with a single overloaded spin()/display() function.
        *  \param[in] spin_time - How long (in ms) should the visualization loop be allowed to run.
        */
      void spinOnce ( const int spin_time = 0 );
        
      /** \brief spins (runs the event loop) the interactor indefinitely. Same as display() - added to retain the similarity between other existing visualization classes
       */
      void spin ();

    private:
      //std::map< int, std::vector< std::vector<float> > > figures_; //FIG_TYPE -> std::vector<array>

      //All the figures drawn till now gets stored here
      std::vector<Figure2D *> figures_;
    
      //state variables of the class
      vtkPen *current_pen_;
      vtkBrush *current_brush_;
      vtkTransform2D *current_transform_;
      int win_width_, win_height_;
      double bkg_color_[3];

      vtkContextView *view_;
      
      //####event callback class####
        struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New ()
          {
            return (new ExitMainLoopTimerCallback);
          }
          void 
          Execute (vtkObject* vtkNotUsed (caller), unsigned long event_id, void* call_data) override
          {
            if (event_id != vtkCommand::TimerEvent)
              return;
            int timer_id = *(reinterpret_cast<int*> (call_data));

            if (timer_id != right_timer_id)
              return;

            // Stop vtk loop and send notification to app to wake it up
            interactor->TerminateApp ();
          }
          int right_timer_id;
          vtkRenderWindowInteractor *interactor;
        };
        
        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_loop_timer_;
    };

  }
}
