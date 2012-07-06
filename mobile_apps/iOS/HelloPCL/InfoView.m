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

#import "InfoView.h"


@implementation InfoView

@synthesize modelInfoLabel;
@synthesize contentView;

- (id)initWithFrame: (CGRect)inFrame;
{
    if ( (self = [super initWithFrame: inFrame]) ) {
        [[NSBundle mainBundle] loadNibNamed: @"InfoView"
                                      owner: self
                                    options: nil];
        self.autoresizesSubviews = YES;
        self.autoresizingMask = UIViewAutoresizingFlexibleRightMargin | UIViewAutoresizingFlexibleLeftMargin;
        [self addSubview:contentView];
    }
    return self;
}

- (void)dealloc;
{
    [contentView release];
    [modelInfoLabel release];
    [super dealloc];
}


-(IBAction)kitwareDotCom:(UIButton*)sender
{
  [[UIApplication sharedApplication] openURL:[NSURL URLWithString:@"http://www.kitware.com"]];

}

- (void)updateModelInfoLabelWithNumFacets:(int)numFacets withNumLines:(int)numLines withNumVertices:(int)numVertices withCurrentRefreshRate:(int)refreshRate
{
  modelInfoLabel.text = [NSString stringWithFormat:@"Current Mesh:\n   Triangles: %i\n   Lines: %i\n   Vertices: %i\n   Drawing @ %dHz", numFacets, numLines, numVertices, refreshRate];
}


@end
