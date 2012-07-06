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

#import "kiwiAppDelegate.h"
#import "GLViewController.h"
#import "EAGLView.h"
#import "InfoView.h"
#import "TitleBarViewContainer.h"

#include "vesKiwiPCLApp.h"
#include "vesKiwiPCLDemo.h"
#include "vesKiwiPolyDataRepresentation.h"
#include "vesActor.h"

@implementation kiwiAppDelegate

@synthesize window;
@synthesize glView;
@synthesize viewController;
@synthesize dataLoader = _dataLoader;
@synthesize loadDataPopover = _loadDataPopover;

@synthesize opacityLabel;
@synthesize opacitySlider;
@synthesize voxelLabel;
@synthesize voxelButtons;
@synthesize ransacLabel;
@synthesize ransacButtons;

@synthesize voxelIndicator;
@synthesize ransacIndicator;

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
  self->waitDialog = nil;
  self->myQueue = dispatch_queue_create("com.kitware.KiwiViewer.myqueue", 0);

  self.window.rootViewController = self.viewController;
  NSURL *url = (NSURL *)[launchOptions valueForKey:UIApplicationLaunchOptionsURLKey];
  [self handleUrl:url];
  return YES;
}

- (void)dealloc
{
  self.loadDataPopover = nil;

  [window release];
  [glView release];
  [viewController release];
  [_dataLoader release];
  [super dealloc];
}

- (void)applicationWillResignActive:(UIApplication *)application
{
}

- (void)applicationDidBecomeActive:(UIApplication *)application
{
}

- (void)applicationWillTerminate:(UIApplication *)application
{
}

-(IBAction)reset:(UIButton*)sender
{
  [glView resetView];
}

-(IBAction)information:(UIButton*)sender
{
  InfoView *infoView = [[[InfoView alloc] initWithFrame:CGRectMake(0,0,320,260)] autorelease];

  if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone)
    {
    TitleBarViewContainer* container = [TitleBarViewContainer new];
    infoView.contentView.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.0];
    [container addViewToContainer:infoView];
    container.previousViewController = self.window.rootViewController;
    [self.viewController presentModalViewController:container animated:YES];
    }
  else
    {
    UIViewController* newController = [UIViewController new];
    newController.view = infoView;
    UIPopoverController *popover = [[UIPopoverController alloc] initWithContentViewController:newController];
    [popover setPopoverContentSize:CGSizeMake(320,260) animated:NO];
    [popover presentPopoverFromRect:[(UIButton*)sender frame] inView:self.glView
           permittedArrowDirections:(UIPopoverArrowDirectionDown)
                           animated:NO];

    self.viewController.infoPopover = popover;
    }

  [infoView updateModelInfoLabelWithNumFacets:[self.glView getNumberOfFacetsForCurrentModel]
                                 withNumLines:[self.glView getNumberOfLinesForCurrentModel]
                              withNumVertices:[self.glView getNumberOfVerticesForCurrentModel]
                       withCurrentRefreshRate:[self.glView currentRefreshRate]];
}

- (BOOL)application:(UIApplication *)application handleOpenURL:(NSURL *)url
{
  return [self handleUrl:url];
}

-(NSString*) documentsDirectory
{
  NSArray* documentDirectories = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
  return [documentDirectories objectAtIndex:0];
}

-(NSString*) pathInDocumentsDirectoryForFileName:(NSString*) fileName
{
  return [[self documentsDirectory] stringByAppendingPathComponent:fileName];
}

-(void)beginCloudMode
{
  self.opacitySlider.hidden = NO;
  self.opacityLabel.hidden = NO;
  
  self.voxelButtons.hidden = NO;
  self.voxelLabel.hidden = NO;
  
  self.ransacButtons.hidden = NO;
  self.ransacLabel.hidden = NO;
  
  self.opacitySlider.value = 1.0;
  self.voxelButtons.selectedSegmentIndex = 2;
  self.ransacButtons.selectedSegmentIndex = 0;
}

-(void)endCloudMode
{
  self.opacitySlider.hidden = YES;
  self.opacityLabel.hidden = YES;
  self.voxelButtons.hidden = YES;
  self.voxelLabel.hidden = YES;
  self.ransacButtons.hidden = YES;
  self.ransacLabel.hidden = YES;
  self.voxelIndicator.hidden = YES;
  self.ransacIndicator.hidden = YES;
}

-(void)showHeadImageDialog
{
  NSString* title = @"CT Image";
  NSString* message = @"About the CT image";
  [self showAlertDialogWithTitle:title message:message];
}

-(void) showErrorDialog
{
  vesKiwiViewerApp* app = [glView getApp];
  NSString* errorTitle = [NSString stringWithUTF8String:app->loadDatasetErrorTitle().c_str()];
  NSString* errorMessage = [NSString stringWithUTF8String:app->loadDatasetErrorMessage().c_str()];
  [self showAlertDialogWithTitle:errorTitle message:errorMessage];
}

-(void) showWaitDialogWithMessage:(NSString*) message
{
  if (self->waitDialog != nil) {
    self->waitDialog.title = message;
    return;
  }

  self->waitDialog = [[UIAlertView alloc]
    initWithTitle:message message:nil
    delegate:self cancelButtonTitle:nil otherButtonTitles: nil];
  [self->waitDialog show];
}

-(void) showWaitDialog
{
  [self showWaitDialogWithMessage:@"Opening data..."];
}

-(void) dismissWaitDialog
{
  if (self->waitDialog == nil) {
    return;
  }
  [self->waitDialog dismissWithClickedButtonIndex:0 animated:YES];
  [self->waitDialog release];
  self->waitDialog = nil;
}

-(void) postLoadDataset:(NSString*)filename result:(BOOL)result
{
  [self dismissWaitDialog];
  if (!result) {
    [self showErrorDialog];
    }
}

-(BOOL) loadDatasetWithPath:(NSString*)path builtinIndex:(int) index
{
  NSLog(@"load dataset: %@", path);


  // Hide or show the UI for the PCL demo
  if ([path hasSuffix:@".pcd"])
  {
    [self beginCloudMode];
  }
  else
  {
    [self endCloudMode];  
  }


  self.glView->builtinDatasetIndex = index;

  [self showWaitDialog];

  dispatch_async(self->myQueue, ^{

    [self->glView disableRendering];
    bool result = [glView getApp]->loadDataset([path UTF8String]);
    [self->glView resetView];
    [self->glView enableRendering];

    dispatch_async(dispatch_get_main_queue(), ^{
      [self postLoadDataset:path result:result];
    });
  });

  return YES;
}

-(BOOL) loadDatasetWithPath:(NSString*) path
{
  return [self loadDatasetWithPath:path builtinIndex:-1];
}

-(void) loadBuiltinDatasetWithIndex:(int)index
{
  vesKiwiViewerApp* app = [self.glView getApp];
  NSString* datasetName = [NSString stringWithUTF8String:app->builtinDatasetFilename(index).c_str()];

  NSString* absolutePath = [[NSBundle mainBundle] pathForResource:datasetName ofType:nil];
  if (absolutePath == nil)
    {
    absolutePath = [self pathInDocumentsDirectoryForFileName:datasetName];
    }

  [self loadDatasetWithPath:absolutePath builtinIndex:index];
}

- (void)willPresentAlertView:(UIAlertView *)alertView
{
  if (alertView == self->waitDialog) {
   UIActivityIndicatorView *indicator = [[[UIActivityIndicatorView alloc]
    initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge] autorelease];
  indicator.center = CGPointMake(alertView.bounds.size.width / 2, alertView.bounds.size.height - 50);
  [alertView addSubview:indicator];
  [indicator startAnimating];
  }
}


-(IBAction) opacitySliderValueChanged:(UISlider*)sender
{
  double value = [sender value];

  vesKiwiPCLApp* app = [self.glView getApp];
  vesKiwiPCLDemo::Ptr rep = app->getPCLDemo();

  if (rep)
  {
  rep->cloudRepresentation()->setColor(1,1,1,value);
  [self.glView scheduleRender];
  }
}


-(void) startVoxelSpinIndicator
{
  self.voxelIndicator.hidden = NO;
  [self.voxelIndicator performSelectorOnMainThread:@selector(startAnimating) withObject:nil waitUntilDone:NO];
}

-(void) startRansacSpinIndicator
{  
  self.ransacIndicator.hidden = NO;
  [self.ransacIndicator performSelectorOnMainThread:@selector(startAnimating) withObject:nil waitUntilDone:NO];
}

-(void) stopSpinIndicator
{
  self.voxelIndicator.hidden = YES;  
  self.ransacIndicator.hidden = YES;
  [self.voxelIndicator performSelectorOnMainThread:@selector(stopAnimating) withObject:nil waitUntilDone:NO];
  [self.ransacIndicator performSelectorOnMainThread:@selector(stopAnimating) withObject:nil waitUntilDone:NO];
}


-(IBAction) voxelGridIndexChanged
{
  int selectedIndex = self.voxelButtons.selectedSegmentIndex;

  double leafSize = 0.0;
  if (selectedIndex == 1) leafSize = 0.01;
  if (selectedIndex == 2) leafSize = 0.05;
  if (selectedIndex == 3) leafSize = 0.10;

  vesKiwiPCLApp* app = [self.glView getApp];
  vesKiwiPCLDemo::Ptr rep = app->getPCLDemo();
  
  if (rep)
  {
    [self startVoxelSpinIndicator];

    dispatch_async(self->myQueue, ^{

      [self->glView disableRendering];
      rep->setLeafSize(leafSize);
      [self->glView enableRendering];

      dispatch_async(dispatch_get_main_queue(), ^{
        [self stopSpinIndicator];
        [self.glView scheduleRender];
      });
    });
    
    [self.glView scheduleRender];
  }
}

-(IBAction) ransacIndexChanged
{
  int selectedIndex = self.ransacButtons.selectedSegmentIndex;

  double distanceThreshold = 0.0;
  if (selectedIndex == 1) distanceThreshold = 0.01;
  if (selectedIndex == 2) distanceThreshold = 0.05;
  if (selectedIndex == 3) distanceThreshold = 0.10;

  vesKiwiPCLApp* app = [self.glView getApp];
  vesKiwiPCLDemo::Ptr rep = app->getPCLDemo();
  
  if (rep)
  {
    [self startRansacSpinIndicator];

    dispatch_async(self->myQueue, ^{

      [self->glView disableRendering];
      rep->setPlaneDistanceThreshold(distanceThreshold);    
      [self->glView enableRendering];

      dispatch_async(dispatch_get_main_queue(), ^{
        [self stopSpinIndicator];
        [self.glView scheduleRender];
      });
    });
  

  }
}

- (void)alertView:(UIAlertView *)alertView clickedButtonAtIndex:(NSInteger)buttonIndex
{

}

- (BOOL)handleUrl:(NSURL *)url;
{
  // no url; go with the default dataset
  if (!url)
    {    
    NSString* defaultFile = [[NSBundle mainBundle] pathForResource:@"pointcloud.pcd" ofType:nil];
    [self loadDatasetWithPath:defaultFile];
    return YES;
    }

  if ([url isFileURL])
    {
    return [self loadDatasetWithPath:[url path]];
    }

  return NO;
}

- (void)showAlertDialogWithTitle:(NSString *)alertTitle message:(NSString *)alertMessage;
{

  UIAlertView *alert = [[UIAlertView alloc]
                        initWithTitle:alertTitle
                        message:alertMessage
                        delegate:self
                        cancelButtonTitle:@"Ok"
                        otherButtonTitles: nil, nil];
  [alert show];
  [alert release];
}

-(void)dismissLoadDataView
{
  if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone)
    {
    [self.viewController dismissModalViewControllerAnimated:NO];
    }
  else
    {
    [self.loadDataPopover dismissPopoverAnimated:YES];
    }
}

-(void)dataSelected:(int)index
{
  [self dismissLoadDataView];
  [self loadBuiltinDatasetWithIndex:index];
}

-(IBAction)setLoadDataButtonTapped:(id)sender
{
  if (_dataLoader == nil)
    {
    _dataLoader = [[LoadDataController alloc]
                    initWithStyle:UITableViewStyleGrouped];
    _dataLoader.delegate = self;

    vesKiwiViewerApp* app = [self.glView getApp];
    NSMutableArray* exampleData = [NSMutableArray array];
    for (int i = 0; i < app->numberOfBuiltinDatasets(); ++i)
    {
      NSString* datasetName = [NSString stringWithUTF8String:app->builtinDatasetName(i).c_str()];
      [exampleData addObject:datasetName];
    }
    _dataLoader.exampleData = exampleData;


    if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone)
      {
      _dataLoader.modalPresentationStyle = UIModalPresentationFormSheet;
      }
    else
      {
      self.loadDataPopover = [[[UIPopoverController alloc]
                               initWithContentViewController:_dataLoader] autorelease];
      }
    }

  if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone)
    {
    TitleBarViewContainer* container = [TitleBarViewContainer new];
    [container addViewToContainer:_dataLoader.view];
    [container setTitle:@"Load Data"];
    container.previousViewController = self.window.rootViewController;
    [self.viewController presentModalViewController:container animated:YES];
    }
  else
    {
    [self.loadDataPopover presentPopoverFromRect:[(UIButton*)sender frame]
                                          inView:self.glView permittedArrowDirections:UIPopoverArrowDirectionDown animated:YES];
    self.viewController.loadPopover = self.loadDataPopover;
    }
}

@end
