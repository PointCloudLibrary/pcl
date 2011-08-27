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
 *
 */

#ifndef PCL_VISUALIZATION_POINT_PICKING_EVENT_H_
#define	PCL_VISUALIZATION_POINT_PICKING_EVENT_H_

#include <pcl/pcl_macros.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>

namespace pcl
{
  namespace visualization
  {
    class PCL_EXPORTS PointPickingCallback : public vtkCommand
    {
      public:
        static PointPickingCallback *New () { return new PointPickingCallback; }

        virtual void
        Execute (vtkObject *caller, unsigned long eventid, void*);

        int
        performSinglePick (vtkRenderWindowInteractor *iren);

        int
        performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z);
     };

    /** /brief Class representing 3D point picking events. */
    class PCL_EXPORTS PointPickingEvent
    {
      public:
        inline PointPickingEvent (int idx)
        {
          idx_ = idx;
        }

        inline PointPickingEvent (int idx, float x, float y, float z)
        {
          idx_ = idx; x_ = x; y_ = y; z_ = z;
        }

        inline int
        getPointIndex () const
        {
          return (idx_);
        }

        inline void
        getPoint (float &x, float &y, float &z) const
        {
          x = x_; y = y_; z = z_;
        }

      private:
        int idx_;

        float x_, y_, z_;
    };
  } //namespace visualization
} //namespace pcl

#endif	/* PCL_VISUALIZATION_POINT_PICKING_EVENT_H_ */

