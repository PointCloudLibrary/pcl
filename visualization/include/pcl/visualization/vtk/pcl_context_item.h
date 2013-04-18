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

#ifndef PCL_VISUALIZATION_PCL_CONTEXT_ITEM_H_
#define	PCL_VISUALIZATION_PCL_CONTEXT_ITEM_H_

#include <pcl/pcl_macros.h>
#include <pcl/visualization/vtk.h>
#include <vtkContextItem.h>
#include <vtkContextActor.h>
#include <vtkContext2D.h>
#include <vtkPen.h>
#include <vtkBrush.h>

namespace pcl
{
  namespace visualization
  {
    /** Struct PCLContextItem represents our own custom version of vtkContextItem, used by
      * the ImageViewer class.
      *
      * \author Nizar Sallem
      */
    struct PCL_EXPORTS PCLContextItem : public vtkContextItem
    {
      vtkTypeMacro (PCLContextItem, vtkContextItem);
      static PCLContextItem *New();
      virtual bool Paint (vtkContext2D *) { return (false); };
      void setColors (unsigned char r, unsigned char g, unsigned char b);
      void setColors (unsigned char rgb[3]) { memcpy (colors, rgb, 3 * sizeof (unsigned char)); }
      void setOpacity (double opacity) { SetOpacity (opacity); };
      unsigned char colors[3];
      double opacity;
      std::vector<float> params;
    };

    /** Struct PCLContextImageItem a specification of vtkContextItem, used to add an image to the
      * scene in the ImageViewer class.
      *
      * \author Nizar Sallem
      */
    struct PCL_EXPORTS PCLContextImageItem : public vtkContextItem
    {
      vtkTypeMacro (PCLContextImageItem, vtkContextItem);
      PCLContextImageItem () { image = vtkSmartPointer<vtkImageData>::New (); }
      static PCLContextImageItem *New ();
      virtual bool Paint (vtkContext2D *painter);
      void set (float _x, float _y, vtkImageData *_image);
      vtkSmartPointer<vtkImageData> image;
      float x, y;
    };

    namespace context_items
    {
      struct PCL_EXPORTS Point : public PCLContextItem
      {
        vtkTypeMacro (Point, PCLContextItem);
        static Point *New();
        virtual bool Paint (vtkContext2D *painter);
        virtual void set (float _x, float _y);
      };

      struct PCL_EXPORTS Line : public PCLContextItem
      {
        vtkTypeMacro (Line, PCLContextItem);
        static Line *New();
        virtual bool Paint (vtkContext2D *painter);
        virtual void set (float _x_1, float _y_1, float _x_2, float _y_2);
      };

      struct PCL_EXPORTS Circle : public PCLContextItem
      {
        vtkTypeMacro (Circle, PCLContextItem);
        static Circle *New();
        virtual bool Paint (vtkContext2D *painter);
        virtual void set (float _x, float _y, float _r);
      };

      struct PCL_EXPORTS Disk : public Circle
      {
        vtkTypeMacro (Disk, Circle);
        static Disk *New();
        virtual bool Paint (vtkContext2D *painter);
      };

      struct PCL_EXPORTS Rectangle : public PCLContextItem
      {
        vtkTypeMacro (Rectangle, Point);
        static Rectangle *New();
        virtual bool Paint (vtkContext2D *painter);
        virtual void set (float _x, float _y, float _w, float _h);
      };

      struct PCL_EXPORTS FilledRectangle : public Rectangle
      {
        vtkTypeMacro (FilledRectangle, Rectangle);
        static FilledRectangle *New();
        virtual bool Paint (vtkContext2D *painter);
      };

      struct PCL_EXPORTS Points : public PCLContextItem
      {
        vtkTypeMacro (Points, PCLContextItem);
        static Points *New();
        virtual bool Paint (vtkContext2D *painter);
        void set (const std::vector<float>& _xy)  { params = _xy; }
      };

      struct PCL_EXPORTS Polygon : public Points
      {
        vtkTypeMacro (Polygon, Points);
        static Polygon *New();
        virtual bool Paint (vtkContext2D *painter);
      };
    }
  }
}

#endif
