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
 * $Id: interactor.cpp 34885 2010-12-19 00:23:43Z rusu $
 *
 */

#include <pcl_visualization/interactor.h>

namespace pcl_visualization
{
  // Standard VTK macro for *New () 
  vtkStandardNewMacro (PCLVisualizerInteractor);
  
  void PCLVisualizerInteractor::stopLoop ()
  {
#ifndef _WIN32
    BreakLoopFlagOn ();
    XClientMessageEvent client;
    memset (&client, 0, sizeof (client));
    client.type = ClientMessage;
    client.display = DisplayId;
    client.window = WindowId;
    client.message_type = XInternAtom (client.display, "spinOnce exit", false);
    client.format = 32; // indicates size of data chunks: 8, 16 or 32 bits...
    XSendEvent (client.display, client.window, True, NoEventMask, reinterpret_cast<XEvent *>(&client));
    XFlush (client.display);
#endif
  }
 //void PCLVisualizerInteractor::TerminateApp(void)
 //{
    ////std::cerr<<__PRETTY_FUNCTION__<<" called.\n";
    //stopped = true;
  //}
}

