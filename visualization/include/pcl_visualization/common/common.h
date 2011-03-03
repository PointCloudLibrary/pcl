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
 * $Id: common.h 35278 2011-01-17 01:20:57Z rusu $
 *
 */
#ifndef PCL_PCL_VISUALIZER_COMMON_H_
#define PCL_PCL_VISUALIZER_COMMON_H_
#include <vtkCommand.h>
#include <vtkTextActor.h>

namespace pcl_visualization
{
  /** \brief Get (good) random values for R/G/B.
    * \param r the resultant R color value
    * \param g the resultant G color value
    * \param b the resultant B color value
    */
  void getRandomColors (double &r, double &g, double &b, double min = 0.2, double max = 2.8);

  enum RenderingProperties
  {
    PCL_VISUALIZER_POINT_SIZE,
    PCL_VISUALIZER_OPACITY,
    PCL_VISUALIZER_LINE_WIDTH,
    PCL_VISUALIZER_FONT_SIZE,
    PCL_VISUALIZER_COLOR
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Camera class holds a set of camera parameters together with the window pos/size. */
  class Camera
  {
    public:
      double clip[2];     // clipping range
      double focal[3];    // focal point
      double pos[3];      // position
      double view[3];     // viewup

      double window_size[2];  // window size
      double window_pos[2];   // window position
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  class FPSCallback : public vtkCommand
  {
    public:
      static FPSCallback *New () { return new FPSCallback;}
      inline void setTextActor (vtkTextActor *txt) { this->actor_ = txt; }
      virtual void Execute (vtkObject *, unsigned long, void*);
    protected:
      vtkTextActor *actor_;
  };
}

#endif
