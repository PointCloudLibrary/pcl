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

#import "ES2Renderer.h"

#include "vesKiwiPCLApp.h"

@implementation ES2Renderer

@synthesize app = mApp;


-(NSString*) stringFromFileContents:(NSString*)filename
{
    NSString* absoluteFilename = [[NSBundle mainBundle] pathForResource:filename ofType:nil];
    return [NSString stringWithContentsOfFile:absoluteFilename
              encoding:NSUTF8StringEncoding error:nil];
}

- (id)init
{
  self = [super init];
  if (self)
  {
    self->mApp = new vesKiwiPCLApp;
    self->mApp->initGL();
  }

  return self;
}

- (vesKiwiPCLApp*) getApp
{
  return self->mApp;
}

- (void) render
{
  printf("render()\n");
  self->mApp->render();
}

- (BOOL) resizeFromLayer:(int) w height:(int) h
{
  printf("resize(%d, %d)\n", w, h);
  self->mApp->resizeView(w, h);
  return YES;
}

- (void)dealloc
{
  delete self->mApp;
  [super dealloc];
}

- (void)resetView
{
  printf("resetView()\n");
  self->mApp->resetView();
  [[NSNotificationCenter defaultCenter] postNotificationName:@"scheduleRender" object:nil];
}

-(int) getNumberOfFacetsForCurrentModel
{
  return mApp->numberOfModelFacets();
}

-(int) getNumberOfLinesForCurrentModel
{
  return mApp->numberOfModelLines();
}

-(int) getNumberOfVerticesForCurrentModel
{
  return mApp->numberOfModelVertices();
}
@end
