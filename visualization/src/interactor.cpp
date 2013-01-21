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
 * $Id$
 *
 */

#include <vtkVersion.h>
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))

#include <pcl/visualization/interactor.h>
#include <vtkCommand.h>

namespace pcl
{
  namespace visualization
  {
    // Standard VTK macro for *New () 
    vtkStandardNewMacro (PCLVisualizerInteractor);
    
/*    void
    PCLVisualizerInteractor::TerminateApp ()
    {
      stopped = true;
    }
*/    
    //////////////////////////////////////////////////////////////////////////
    void 
    PCLVisualizerInteractor::stopLoop ()
    {
  #if defined (_WIN32) || defined (VTK_USE_COCOA) || defined (VTK_USE_CARBON)
      BreakLoopFlagOn ();
      // Send a VTK_BreakWin32Loop ClientMessage event to be sure we pop out of the
      // event loop.  This "wakes up" the event loop.  Otherwise, it might sit idle
      // waiting for an event before realizing an exit was requested.
    #if defined (_WIN32)
      SendMessage (this->WindowId, RegisterWindowMessage (TEXT ("VTK_BreakWin32Loop")), 0, 0);
    #endif
  #else
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

  #if defined (_WIN32) || (defined VTK_USE_COCOA) || defined (VTK_USE_CARBON)
    //////////////////////////////////////////////////////////////////////////
    void 
    PCLVisualizerInteractor::Start ()
    {
      // Let the compositing handle the event loop if it wants to.
      if (this->HasObserver(vtkCommand::StartEvent) && !this->HandleEventLoop)
      {
        this->InvokeEvent (vtkCommand::StartEvent, NULL);
        return;
      }

      // No need to do anything if this is a 'mapped' interactor
      if (!this->Enabled || !this->InstallMessageProc)
        return;

      this->StartedMessageLoop = 1;

      MSG msg;
      this->BreakLoopFlag=0;
      
      while (GetMessage (&msg, NULL, 0, 0) && this->BreakLoopFlag == 0)
      {
        TranslateMessage (&msg);
        DispatchMessage (&msg);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void 
    PCLVisualizerInteractor::SetBreakLoopFlag (int f)
    {
      if (f)
        this->BreakLoopFlagOn ();
      else
        this->BreakLoopFlagOff ();
    }

    //////////////////////////////////////////////////////////////////////////
    void 
    PCLVisualizerInteractor::BreakLoopFlagOff ()
    {
      this->BreakLoopFlag = 0;
      this->Modified ();
    }

    //////////////////////////////////////////////////////////////////////////
    void 
    PCLVisualizerInteractor::BreakLoopFlagOn ()
    {
      this->BreakLoopFlag = 1;
      this->Modified ();
    }
  #endif
  }
}

#endif
