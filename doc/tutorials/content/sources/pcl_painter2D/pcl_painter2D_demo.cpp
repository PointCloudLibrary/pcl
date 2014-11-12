/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


/* \author Kripasindhu Sarkar */

#include <iostream>
#include <map>
#include <vector>
#include <pcl/visualization/pcl_painter2D.h>
//----------------------------------------------------------------------------

int main (int argc, char * argv [])
{
  pcl::visualization::PCLPainter2D *painter = new pcl::visualization::PCLPainter2D();
  
  int winw = 800, winh = 600;
  painter->setWindowSize (winw, winh);
  int xpos = 0;
  int r = winw;
  int R = 50;
  int inc = 5;
  int noc = winw/R;
  
  while (1)
  {
    //draw noc no of circles
    for (int i = 0; i < noc; i++)
    {
      if (i % 2) 
        painter->setBrushColor (0, 0, 0, 200);
      else
        painter->setBrushColor (255, 255, 255, 200);
      
      int rad = r - i*R;
      if (rad < 0) { rad = winw + rad;}
      
      painter->addCircle (winw/2, winh/2, rad);
    }
    
    r -= inc;
    if (r < winw-R) r = winw + R;

    painter->setBrushColor (255,0,0,100);
    painter->addRect ((xpos += inc) % winw, 100, 100, 100);

    //display
    painter->spinOnce ();
    painter->clearFigures ();
  }


  return 0;
}