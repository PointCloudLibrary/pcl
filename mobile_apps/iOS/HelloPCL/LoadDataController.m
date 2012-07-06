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

#import "LoadDataController.h"


@implementation LoadDataController

@synthesize exampleData = _exampleData;
@synthesize helpStrings = _helpStrings;
@synthesize delegate = _delegate;

- (id)initWithStyle:(UITableViewStyle)style
{
    self = [super initWithStyle:style];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)dealloc
{
  self.exampleData = nil;
  self.helpStrings = nil;
  self.delegate = nil;
    [super dealloc];
}

- (void)didReceiveMemoryWarning
{
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];

    // Release any cached data, images, etc that aren't in use.
}

#pragma mark - View lifecycle

- (void)viewDidLoad
{
  [super viewDidLoad];
  self.clearsSelectionOnViewWillAppear = YES;
  self.contentSizeForViewInPopover = CGSizeMake(275.0, 570.0);

  self.helpStrings = [NSMutableArray array];


  [_helpStrings addObject:@"from DropBox."];
  [_helpStrings addObject:@"from email."];

}

- (void)viewDidUnload
{
    [super viewDidUnload];
    // Release any retained subviews of the main view.
    // e.g. self.myOutlet = nil;
}

- (void)viewWillAppear:(BOOL)animated
{
    [super viewWillAppear:animated];
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    // Return YES for supported orientations
  return YES;
}

#pragma mark - Table view data source

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    // Return the number of sections.
    return 2;
}

- (NSString *)tableView:(UITableView *)tableView titleForHeaderInSection:(NSInteger)section {
  if (section == 0) {
    return @"Example data";
  }
  else {
    return @"How to load data...";
  }
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    // Return the number of rows in the section.
  if (section == 0) {
    return [_exampleData count];
  } else if (section == 1) {
    return [_helpStrings count];
  }
  return 0;
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    static NSString *CellIdentifier = @"Cell";

    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifier];
    if (cell == nil) {
        cell = [[[UITableViewCell alloc] initWithStyle:UITableViewCellStyleDefault reuseIdentifier:CellIdentifier] autorelease];
    }

    // Configure the cell...
  if (indexPath.section == 0) {
    NSString *label = [_exampleData objectAtIndex:indexPath.row];
    cell.textLabel.text = label;
  }

  if (indexPath.section == 1) {
    NSString *label = [_helpStrings objectAtIndex:indexPath.row];
    cell.textLabel.text = label;
    //cell.accessoryType = UITableViewCellAccessoryDetailDisclosureButton;
  }
  return cell;
}

#pragma mark - Table view delegate

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
  // deselect the row
  // on iphone, this view is packed in a modal view controller and when it is dismissed
  // the tableview does not get the message to deselect the row automatically
  [self.tableView deselectRowAtIndexPath:indexPath animated:YES];

  if (_delegate != nil) {
    if (indexPath.section == 0) {
      [_delegate dataSelected:indexPath.row];
    }
    else if (indexPath.section == 1) {
      NSString *label = [_helpStrings objectAtIndex:indexPath.row];
      if (label == @"from the web.") {
        [[UIApplication sharedApplication] openURL:[NSURL URLWithString:@"http://kitware.com/kiwiviewer/help/documentation.html"]];
      }
      else if (label == @"from DropBox.") {
        [[UIApplication sharedApplication] openURL:[NSURL URLWithString:@"http://kitware.com/kiwiviewer/help/documentation.html#dropbox"]];

      }
      else if (label == @"from email.") {
        [[UIApplication sharedApplication] openURL:[NSURL URLWithString:@"http://kitware.com/kiwiviewer/help/documentation.html#email"]];
      }
    }
  }
}

@end
