/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 ========================================================================*/

#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <UIKit/UIKit.h>
#import <OpenGLES/EAGL.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>
#import "kiwiAppDelegate.h"

@class ES2Renderer;
struct vesKiwiPCLApp;

// This class wraps the CAEAGLLayer from CoreAnimation into a convenient UIView subclass.
// The view content is basically an EAGL surface you render your OpenGL scene into.
// Note that setting the view non-opaque will only work if the EAGL surface has an alpha channel.
@interface EAGLView : UIView
{
@public

  int builtinDatasetIndex;

@private
  EAGLContext *context;
  // The pixel dimensions of the CAEAGLLayer
  GLint backingWidth;
  GLint backingHeight;

  /* OpenGL names for the renderbuffer, framebuffers used to render to this view */
  GLuint viewRenderbuffer;
  GLuint depthRenderbuffer;
  GLuint viewFramebuffer;

  // animation loop
  BOOL shouldRender;
  CADisplayLink* displayLink;
  NSMutableArray* recentRenderFPS;
  NSRecursiveLock* renderDataMutex;

  ES2Renderer* renderer;
  NSString *filePath;
}

@property (nonatomic, retain) EAGLContext *context;

// animation loop
- (void)drawView:(id)sender;
- (void)scheduleRender;
- (void)forceRender;
- (void)updateRefreshRate:(float) lastRenderFPS;
- (int)currentRefreshRate;

-(struct vesKiwiPCLApp*) getApp;

- (void)resetView;

// resource protection
-(void)disableRendering;
-(void)enableRendering;

// Touch handling
- (void) createGestureRecognizers;
- (IBAction)handleSingleFingerPanGesture:(UIPanGestureRecognizer *)sender;
- (IBAction)handleDoubleFingerPanGesture:(UIPanGestureRecognizer *)sender;
- (IBAction)handlePinchGesture:(UIPinchGestureRecognizer *)sender;
- (IBAction)handle2DRotationGesture:(UIRotationGestureRecognizer *)sender;
- (IBAction)handleTapGesture:(UITapGestureRecognizer *)sender;

// model information
- (int)getNumberOfFacetsForCurrentModel;
- (int)getNumberOfLinesForCurrentModel;
- (int)getNumberOfVerticesForCurrentModel;

@end
