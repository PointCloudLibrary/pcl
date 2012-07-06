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

#import "EAGLView.h"
#import "ES2Renderer.h"

#include "vesKiwiPCLApp.h"
#include "vesKiwiCameraSpinner.h"

@interface kwGestureDelegate : NSObject <UIGestureRecognizerDelegate>{

}
@end

@implementation kwGestureDelegate

- (BOOL)gestureRecognizerShouldBegin:(UIGestureRecognizer *)gestureRecognizer
{
  return YES;
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer *)otherGestureRecognizer
{
  BOOL rotating2D =
  [gestureRecognizer isMemberOfClass:[UIRotationGestureRecognizer class]] ||
  [otherGestureRecognizer isMemberOfClass:[UIRotationGestureRecognizer class]];

  BOOL pinching =
  [gestureRecognizer isMemberOfClass:[UIPinchGestureRecognizer class]] ||
  [otherGestureRecognizer isMemberOfClass:[UIPinchGestureRecognizer class]];

  BOOL panning =
  [gestureRecognizer numberOfTouches] == 2 &&
  ([gestureRecognizer isMemberOfClass:[UIPanGestureRecognizer class]] ||
   [otherGestureRecognizer isMemberOfClass:[UIPanGestureRecognizer class]]);

  if ((pinching && panning) ||
      (pinching && rotating2D) ||
      (panning && rotating2D))
    {
    return YES;
    }
  return NO;
}
- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer shouldReceiveTouch:(UITouch *)touch
{
  if ([touch.view isKindOfClass:[UIButton class]]
      || [touch.view isKindOfClass:[UISlider class]]) {
    return NO;
  }
  return YES;
}
@end

@interface EAGLView ()

- (BOOL) createFramebuffer;
- (void) destroyFramebuffer;

@end

@implementation EAGLView

@synthesize context;

// You must implement this method
+ (Class)layerClass
{
  return [CAEAGLLayer class];
}

//The EAGL view is stored in the nib file. When it's unarchived it's sent -initWithCoder:
- (id)initWithCoder:(NSCoder*)coder
{
  self = [super initWithCoder:coder];
  if (self)
    {
    // Get the layer
    CAEAGLLayer *eaglLayer = (CAEAGLLayer *)self.layer;

    eaglLayer.opaque = TRUE;
    eaglLayer.drawableProperties = [NSDictionary dictionaryWithObjectsAndKeys:
                                    [NSNumber numberWithBool:FALSE], kEAGLDrawablePropertyRetainedBacking, kEAGLColorFormatRGBA8, kEAGLDrawablePropertyColorFormat, nil];
    context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    if (!context || ![EAGLContext setCurrentContext:context])
      {
      [self release];
      return nil;
      }

    builtinDatasetIndex = -1;
    renderer = [[ES2Renderer alloc] init];

    if (!renderer)
      {
      [self release];
      return nil;
      }

    [self createGestureRecognizers];
    self.multipleTouchEnabled = YES;

    self->renderDataMutex = [[NSRecursiveLock alloc] init];
    [self->renderDataMutex setName:@"RenderDataMutex"];
    self->shouldRender = NO;
    self->recentRenderFPS = [NSMutableArray new];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(scheduleRender) name:@"scheduleRender" object:nil];
    }

  return self;
}

-(struct vesKiwiPCLApp*) getApp
{
  return self->renderer.app;
}

- (void)layoutSubviews
{
  [EAGLContext setCurrentContext:context];

  if (self->displayLink)
  {
    [self->displayLink invalidate];
  }

  [self destroyFramebuffer];
  [self createFramebuffer];
  [renderer resizeFromLayer:backingWidth height:backingHeight];

  // Reposition buttons on the iPhone
  if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone)
    {
    UIButton* resetButton = [[self subviews] objectAtIndex:0];
    resetButton.frame = CGRectMake((backingWidth / 2) - (resetButton.frame.size.width / 2),
                                   backingHeight - 50,
                                   resetButton.frame.size.width, resetButton.frame.size.height);

    UIButton* openDataButton = [[self subviews] objectAtIndex:1];
    openDataButton.frame = CGRectMake(backingWidth - 90, backingHeight - 45,
                                      openDataButton.frame.size.width, openDataButton.frame.size.height);

    UIButton* infoButton = [[self subviews] objectAtIndex:2];
    infoButton.frame = CGRectMake(backingWidth - 36, backingHeight - 38,
                                  infoButton.frame.size.width, infoButton.frame.size.height);
    }
  //
  // set up animation loop
  self->displayLink = [self.window.screen displayLinkWithTarget:self selector:@selector(drawView:)];
  [self->displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
  [self scheduleRender];
}

- (BOOL)createFramebuffer
{
  glGenFramebuffers(1, &viewFramebuffer);
  glGenRenderbuffers(1, &viewRenderbuffer);

  glBindFramebuffer(GL_FRAMEBUFFER, viewFramebuffer);
  glBindRenderbuffer(GL_RENDERBUFFER, viewRenderbuffer);
  [context renderbufferStorage:GL_RENDERBUFFER fromDrawable:(CAEAGLLayer*)self.layer];
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, viewRenderbuffer);

  glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &backingWidth);
  glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &backingHeight);

  glGenRenderbuffers(1, &depthRenderbuffer);
  glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, backingWidth, backingHeight);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer);

  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
    NSLog(@"failed to make complete framebuffer object %x", glCheckFramebufferStatus(GL_FRAMEBUFFER));
    return NO;
    }

  return YES;
}

- (void)destroyFramebuffer
{
  glDeleteFramebuffers(1, &viewFramebuffer);
  viewFramebuffer = 0;
  glDeleteRenderbuffers(1, &viewRenderbuffer);
  viewRenderbuffer = 0;

  if(depthRenderbuffer)
    {
    glDeleteRenderbuffers(1, &depthRenderbuffer);
    depthRenderbuffer = 0;
    }
}

-(void) disableRendering
{
  [self->renderDataMutex lock];
}

-(void) enableRendering
{
  [self->renderDataMutex unlock];
}

- (void) updateRefreshRate:(float)lastRenderFPS
{
  //
  // ignore the call if there is no animation loop
  if (!self->displayLink)
    {
    return;
    }

  //
  // keep track of the last few rendering speeds
  const unsigned int maxWindowSize = 20;
  [self->recentRenderFPS addObject:[NSNumber numberWithFloat:lastRenderFPS]];
  if ([self->recentRenderFPS count] > maxWindowSize)
    {
    [self->recentRenderFPS removeObjectAtIndex:0];
    }
  float sumFPS = 0.0;
  for (NSNumber* n in self->recentRenderFPS)
    {
    sumFPS += n.floatValue;
    }
  float meanFPS = sumFPS / maxWindowSize;

  //
  // set forward refresh rate to match current rendering speed
  // (round up to be conservative)
  int desiredFrameInterval = static_cast<int>(60.0 / meanFPS) + 1;

  //
  // clamp to 10Hz or higher
  desiredFrameInterval = desiredFrameInterval > 6 ? 6 : desiredFrameInterval;

  if (desiredFrameInterval != self->displayLink.frameInterval)
    {
    //NSLog(@"Changing frame interval to %d", desiredFrameInterval);
    [self->displayLink setFrameInterval:desiredFrameInterval];
    }
}

- (int)currentRefreshRate
{
  if (!self->displayLink)
    {
    return 0;
    }
  return 60 / self->displayLink.frameInterval;
}

- (void) scheduleRender
{
  self->shouldRender = YES;
}

- (void) forceRender
{
  [self scheduleRender];
  [self drawView:nil];
}

- (void)drawView:(id) sender
{
  if (![self->renderDataMutex tryLock])
    {
    return;
    }

  if (self->shouldRender || self->renderer.app->cameraSpinner()->isEnabled() || self->renderer.app->isAnimating())
    {
    NSDate* startRenderDate = [NSDate date];
    [EAGLContext setCurrentContext:context];
    glBindFramebuffer(GL_FRAMEBUFFER, viewFramebuffer);
    [renderer render];
    glBindRenderbuffer(GL_RENDERBUFFER, viewRenderbuffer);
    [context presentRenderbuffer:GL_RENDERBUFFER];
    self->shouldRender = NO;
    float currentFPS = 1.0 / [[NSDate date] timeIntervalSinceDate:startRenderDate];
    //NSLog(@"Render @ %4.1f fps", currentFPS);
    [self updateRefreshRate:currentFPS];
    }

  [self->renderDataMutex unlock];
}

- (void)dealloc
{
  if ([EAGLContext currentContext] == context)
    {
    [EAGLContext setCurrentContext:nil];
    }
  [context release];
  context = nil;
  [renderer release];
  [renderDataMutex release];
  [super dealloc];
}

- (void)resetView
{
  if (builtinDatasetIndex < 0) {
    renderer.app->resetView();
  }
  else {
    renderer.app->applyBuiltinDatasetCameraParameters(builtinDatasetIndex);
  }
  [self scheduleRender];
}


#pragma mark -
#pragma mark Touch handling

- (void) createGestureRecognizers
{
  UIPanGestureRecognizer *singleFingerPanGesture = [[UIPanGestureRecognizer alloc]
                                                    initWithTarget:self action:@selector(handleSingleFingerPanGesture:)];
  [singleFingerPanGesture setMinimumNumberOfTouches:1];
  [singleFingerPanGesture setMaximumNumberOfTouches:1];
  [self addGestureRecognizer:singleFingerPanGesture];
  [singleFingerPanGesture release];

  UIPanGestureRecognizer *doubleFingerPanGesture = [[UIPanGestureRecognizer alloc]
                                                    initWithTarget:self action:@selector(handleDoubleFingerPanGesture:)];
  [doubleFingerPanGesture setMinimumNumberOfTouches:2];
  [doubleFingerPanGesture setMaximumNumberOfTouches:2];
  [self addGestureRecognizer:doubleFingerPanGesture];
  [doubleFingerPanGesture release];

  UIPinchGestureRecognizer *pinchGesture = [[UIPinchGestureRecognizer alloc]
                                            initWithTarget:self action:@selector(handlePinchGesture:)];
  [self addGestureRecognizer:pinchGesture];
  [pinchGesture release];

  UIRotationGestureRecognizer *rotate2DGesture = [[UIRotationGestureRecognizer alloc]
                                                  initWithTarget:self action:@selector(handle2DRotationGesture:)];
  [self addGestureRecognizer:rotate2DGesture];
  [rotate2DGesture release];

  UITapGestureRecognizer *tapGesture = [[UITapGestureRecognizer alloc]
                                             initWithTarget:self action:@selector(handleTapGesture:)];
  // this is needed so that the buttons on top of the render view will
  // work since this is the first responder---is this the best way to
  // fix this problem?
  //tapGesture.cancelsTouchesInView = NO;
  tapGesture.numberOfTapsRequired = 1;
  [self addGestureRecognizer:tapGesture];
  [tapGesture release];

  UITapGestureRecognizer *doubleTapGesture = [[UITapGestureRecognizer alloc]
                                             initWithTarget:self action:@selector(handleDoubleTapGesture:)];
  //doubleTapGesture.cancelsTouchesInView = NO;
  doubleTapGesture.numberOfTapsRequired = 2;
  [self addGestureRecognizer:doubleTapGesture];
  [doubleTapGesture release];

  [tapGesture requireGestureRecognizerToFail:doubleTapGesture];


  UILongPressGestureRecognizer* longPress = [[UILongPressGestureRecognizer alloc]
                                              initWithTarget:self action:@selector(handleLongPress:)];
  [self addGestureRecognizer:longPress];
  [longPress release];

  //
  // allow two-finger gestures to work simultaneously
  kwGestureDelegate* gestureDelegate = [[kwGestureDelegate alloc] init];
  [rotate2DGesture setDelegate:gestureDelegate];
  [pinchGesture setDelegate:gestureDelegate];
  [doubleFingerPanGesture setDelegate:gestureDelegate];
  [singleFingerPanGesture setDelegate:gestureDelegate];
  [tapGesture setDelegate:gestureDelegate];
  [doubleTapGesture setDelegate:gestureDelegate];
}

- (IBAction)handleDoubleFingerPanGesture:(UIPanGestureRecognizer *)sender
{
  if (sender.state == UIGestureRecognizerStateEnded ||
      sender.state == UIGestureRecognizerStateCancelled)
    {
    return;
    }

  //
  // get current translation and (then zero it out so it won't accumulate)
  CGPoint currentLocation = [sender locationInView:self];
  CGPoint currentTranslation = [sender translationInView:self];
  [sender setTranslation:CGPointZero inView:self];

  //
  // compute the previous location (have to flip y)
  CGPoint previousLocation;
  previousLocation.x = currentLocation.x - currentTranslation.x;
  previousLocation.y = currentLocation.y + currentTranslation.y;

  self->renderer.app->handleTwoTouchPanGesture(previousLocation.x, previousLocation.y, currentLocation.x, currentLocation.y);

  [self scheduleRender];
}

- (IBAction)handleSingleFingerPanGesture:(UIPanGestureRecognizer *)sender
{
  if (sender.state == UIGestureRecognizerStateEnded ||
      sender.state == UIGestureRecognizerStateCancelled)
    {
    self->renderer.app->handleSingleTouchUp();
    [self scheduleRender];
    return;
    }

  //
  // get current translation and (then zero it out so it won't accumulate)
  CGPoint currentTranslation = [sender translationInView:self];
  CGPoint currentLocation = [sender locationInView:self];
  [sender setTranslation:CGPointZero inView:self];

  if (sender.state == UIGestureRecognizerStateBegan)
    {
    self->renderer.app->handleSingleTouchDown(currentLocation.x, currentLocation.y);
    [self scheduleRender];
    return;
    }

  self->renderer.app->handleSingleTouchPanGesture(currentTranslation.x, currentTranslation.y);
  [self scheduleRender];
}

- (IBAction)handlePinchGesture:(UIPinchGestureRecognizer *)sender
{
  if (sender.state == UIGestureRecognizerStateEnded ||
      sender.state == UIGestureRecognizerStateCancelled)
    {
    return;
    }

  self->renderer.app->handleTwoTouchPinchGesture(sender.scale);

  //
  // reset scale so it won't accumulate
  sender.scale = 1.0;

  [self scheduleRender];
}

- (IBAction)handle2DRotationGesture:(UIRotationGestureRecognizer *)sender
{
  if (sender.state == UIGestureRecognizerStateEnded ||
      sender.state == UIGestureRecognizerStateCancelled)
    {
    return;
    }

  self->renderer.app->handleTwoTouchRotationGesture(sender.rotation);

  //
  // reset rotation so it won't accumulate
  [sender setRotation:0.0];

  [self scheduleRender];
}

- (IBAction)handleDoubleTapGesture:(UITapGestureRecognizer *)sender
{
  CGPoint currentLocation = [sender locationInView:self];
  //self->renderer.app->handleSingleTouchTap(currentLocation.x, currentLocation.y);
  self->renderer.app->handleDoubleTap(currentLocation.x, currentLocation.y);
  [self scheduleRender];
}

- (IBAction)handleTapGesture:(UITapGestureRecognizer *)sender
{
  CGPoint currentLocation = [sender locationInView:self];
  self->renderer.app->handleSingleTouchTap(currentLocation.x, currentLocation.y);
  [self scheduleRender];
}

- (IBAction)handleLongPress:(UITapGestureRecognizer *)sender
{
  CGPoint currentLocation = [sender locationInView:self];
  self->renderer.app->handleLongPress(currentLocation.x, currentLocation.y);
  [self scheduleRender];
}


#pragma mark -
#pragma mark Model information


-(int) getNumberOfFacetsForCurrentModel
{
  if (renderer)
    {
    return [renderer getNumberOfFacetsForCurrentModel];
    }
  return 0;
}

-(int) getNumberOfLinesForCurrentModel
{
  if (renderer)
    {
    return [renderer getNumberOfLinesForCurrentModel];
    }
  return 0;
}


-(int) getNumberOfVerticesForCurrentModel
{
  if (renderer)
    {
    return [renderer getNumberOfVerticesForCurrentModel];
    }
  return 0;
}



@end
