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

#import "GLViewController.h"


@implementation GLViewController

@synthesize infoPopover;
@synthesize loadPopover;
@synthesize infoButton;
@synthesize loadButton;

- (void)dealloc
{
  [self->infoButton release];
  [self->loadButton release];
  [self->infoPopover release];
  [self->loadPopover release];
  [super dealloc];
}

- (void)didReceiveMemoryWarning
{
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];

    // Release any cached data, images, etc that aren't in use.
}

#pragma mark - View lifecycle

/*
// Implement loadView to create a view hierarchy programmatically, without using a nib.
- (void)loadView
{
}
*/


// Implement viewDidLoad to do additional setup after loading the view, typically from a nib.
- (void)viewDidLoad
{
    [super viewDidLoad];
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    // Release any retained subviews of the main view.
    // e.g. self.myOutlet = nil;
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
  return YES;
}


- (void)willRotateToInterfaceOrientation:(UIInterfaceOrientation)toInterfaceOrientation duration:(NSTimeInterval)duration
{
  //[self->infoPopover dismissPopoverAnimated:NO];
  //[self->loadPopover dismissPopoverAnimated:NO];
  [super willRotateToInterfaceOrientation:toInterfaceOrientation duration:duration];
}

- (void)didRotateFromInterfaceOrientation:(UIInterfaceOrientation)fromInterfaceOrientation
{
  if (self->infoPopover && self->infoPopover.popoverVisible)
    {
    [self->infoPopover presentPopoverFromRect:[self.infoButton frame]
      inView:self.view permittedArrowDirections:UIPopoverArrowDirectionDown animated:YES];
    }

  if (self->loadPopover && self->loadPopover.popoverVisible)
    {
    [self->loadPopover presentPopoverFromRect:[self.loadButton frame]
      inView:self.view permittedArrowDirections:UIPopoverArrowDirectionDown animated:YES];
    }

  [super didRotateFromInterfaceOrientation:fromInterfaceOrientation];
}


@end
