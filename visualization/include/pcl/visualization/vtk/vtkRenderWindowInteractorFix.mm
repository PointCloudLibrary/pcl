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
  */

#import <Cocoa/Cocoa.h>
#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>
#include <vtkCocoaRenderWindow.h>
#include <vtkCocoaRenderWindowInteractor.h>
#include <vtkObjectFactory.h>

#if ((VTK_MAJOR_VERSION < 6) || ((VTK_MAJOR_VERSION == 6) && (VTK_MINOR_VERSION < 2)))

//----------------------------------------------------------------------------
@interface vtkCocoaServerFix : NSObject
{
  vtkCocoaRenderWindow* renWin;
}

+ (id)cocoaServerWithRenderWindow:(vtkCocoaRenderWindow*)inRenderWindow;

- (void)start;
- (void)stop;
- (void)breakEventLoop;

@end

//----------------------------------------------------------------------------
@implementation vtkCocoaServerFix

//----------------------------------------------------------------------------
- (id)initWithRenderWindow:(vtkCocoaRenderWindow *)inRenderWindow
{
  self = [super init];
  if (self)
  {
    renWin = inRenderWindow;
  }
  return (self);
}

//----------------------------------------------------------------------------
+ (id)cocoaServerWithRenderWindow:(vtkCocoaRenderWindow *)inRenderWindow
{
  vtkCocoaServerFix *server = [[[vtkCocoaServerFix alloc] 
                            initWithRenderWindow:inRenderWindow]
                            autorelease];
  return (server);
}

//----------------------------------------------------------------------------
- (void)start
{
  // Retrieve the NSWindow.
  NSWindow *win = nil;
  if (renWin != NULL)
  {
    win = reinterpret_cast<NSWindow*> (renWin->GetRootWindow ());
  
    // We don't want to be informed of every window closing, so check for nil.
    if (win != nil)
    {
      // Register for the windowWillClose notification in order to stop
      // the run loop if the window closes.
      NSNotificationCenter *nc = [NSNotificationCenter defaultCenter];
      [nc addObserver:self selector:@selector(windowWillClose:) 
                               name:NSWindowWillCloseNotification 
                             object:win];
    }
  }
  // Start the NSApplication's run loop
  NSApplication* application = [NSApplication sharedApplication];
  [application run];
}

//----------------------------------------------------------------------------
- (void)stop
{
  [self breakEventLoop];
}

//----------------------------------------------------------------------------
- (void)breakEventLoop
{
  NSApplication* application = [NSApplication sharedApplication];
  [application stop:application];

  NSEvent *event = [NSEvent otherEventWithType:NSApplicationDefined
                                      location:NSMakePoint(0.0,0.0)
                                 modifierFlags:0
                                     timestamp:0
                                  windowNumber:-1
                                       context:nil
                                       subtype:0
                                         data1:0
                                         data2:0];
  [application postEvent:event atStart:YES];
}

//----------------------------------------------------------------------------
- (void)windowWillClose:(NSNotification*)aNotification
{
  (void)aNotification;
  
  NSNotificationCenter *nc = [NSNotificationCenter defaultCenter];
  [nc removeObserver:self name:NSWindowWillCloseNotification object:nil];
  
  if (renWin)
  {
    int windowCreated = renWin->GetWindowCreated ();
    if (windowCreated)
    {
      [self breakEventLoop];
      
      // The NSWindow is closing, so prevent anyone from accidently using it
      renWin->SetRootWindow(NULL);
    }
  }
} 

@end


//----------------------------------------------------------------------------
class vtkCocoaRenderWindowInteractorFix : public vtkCocoaRenderWindowInteractor
{
  public:

    static vtkCocoaRenderWindowInteractorFix *New ();
    vtkTypeMacro (vtkCocoaRenderWindowInteractorFix, vtkCocoaRenderWindowInteractor);

    virtual void 
    Start ();

    virtual void 
    TerminateApp ();

  protected:
    vtkCocoaRenderWindowInteractorFix () {}
    ~vtkCocoaRenderWindowInteractorFix () {}

  private:
    vtkCocoaRenderWindowInteractorFix (const vtkCocoaRenderWindowInteractorFix&);  // Not implemented.
    void 
    operator = (const vtkCocoaRenderWindowInteractorFix&);  // Not implemented.
};

//----------------------------------------------------------------------------
vtkStandardNewMacro (vtkCocoaRenderWindowInteractorFix);

//----------------------------------------------------------------------------
void 
vtkCocoaRenderWindowInteractorFix::Start ()
{
  vtkCocoaRenderWindow *renWin = vtkCocoaRenderWindow::SafeDownCast (this->GetRenderWindow ());
  if (renWin != NULL)
  {
    vtkCocoaServerFix *server = reinterpret_cast<vtkCocoaServerFix*> (this->GetCocoaServer ());
    if (!this->GetCocoaServer ())
    {
      server = [vtkCocoaServerFix cocoaServerWithRenderWindow:renWin];
      this->SetCocoaServer (reinterpret_cast<void*> (server));
    }

    [server start];
  }
}

//----------------------------------------------------------------------------
void 
vtkCocoaRenderWindowInteractorFix::TerminateApp ()
{
  vtkCocoaRenderWindow *renWin = vtkCocoaRenderWindow::SafeDownCast (this->RenderWindow);
  if (renWin)
  {
    vtkCocoaServerFix *server = reinterpret_cast<vtkCocoaServerFix*> (this->GetCocoaServer ());
    [server stop]; 
  }
}

//----------------------------------------------------------------------------
vtkRenderWindowInteractor* vtkRenderWindowInteractorFixNew ()
{
  return (vtkCocoaRenderWindowInteractorFix::New ());
}
#else
//----------------------------------------------------------------------------
vtkRenderWindowInteractor* vtkRenderWindowInteractorFixNew ()
{
  return (vtkCocoaRenderWindowInteractor::New ());
}
#endif
