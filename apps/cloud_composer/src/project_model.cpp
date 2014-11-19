#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/work_queue.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/merge_selection.h>
#include <pcl/apps/cloud_composer/transform_clouds.h>


pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
  
  last_directory_ = QDir (".");
    
  selection_model_ = new QItemSelectionModel (this);
  
  undo_stack_ = new QUndoStack (this);
  undo_stack_->setUndoLimit (10);
  
  work_thread_ = new QThread();
  work_queue_ = new WorkQueue ();
  work_queue_->moveToThread (work_thread_);
  
  connect (this, SIGNAL (enqueueNewAction (AbstractTool*, ConstItemList)),
           work_queue_, SLOT (enqueueNewAction (AbstractTool*, ConstItemList)));
  connect (work_queue_, SIGNAL (commandComplete (CloudCommand*)),
           this, SLOT (commandCompleted (CloudCommand*)));
  work_thread_->start ();
  
  connect (this, SIGNAL (rowsInserted ( const QModelIndex, int, int)),
           this, SIGNAL (modelChanged ()));
  connect (this, SIGNAL (rowsRemoved  ( const QModelIndex, int, int)),
           this, SIGNAL (modelChanged ()));
  
  connect (this, SIGNAL (rowsAboutToBeRemoved  ( const QModelIndex, int, int)),
           this, SLOT (clearSelection()));
  connect (selection_model_, SIGNAL (selectionChanged(QItemSelection,QItemSelection)),
           this, SLOT (emitAllStateSignals ()));
  connect (selection_model_, SIGNAL (selectionChanged(QItemSelection,QItemSelection)),
           this, SLOT (itemSelectionChanged (QItemSelection,QItemSelection)));
  
  selected_style_map_.insert (interactor_styles::PCL_VISUALIZER, true);
  selected_style_map_.insert (interactor_styles::CLICK_TRACKBALL, false);
  selected_style_map_.insert (interactor_styles::RECTANGULAR_FRUSTUM, false);
  selected_style_map_.insert (interactor_styles::SELECTED_TRACKBALL, false);
}

pcl::cloud_composer::ProjectModel::ProjectModel (const ProjectModel&)
  : QStandardItemModel ()
{
}

pcl::cloud_composer::ProjectModel::~ProjectModel ()
{
  work_thread_->quit ();
  work_thread_->deleteLater ();
  work_queue_->deleteLater ();
}

pcl::cloud_composer::ProjectModel::ProjectModel (QString project_name, QObject* parent)
: QStandardItemModel (parent)
{
  selection_model_ = new QItemSelectionModel(this);
  setName (project_name);
}

void 
pcl::cloud_composer::ProjectModel::setName (QString new_name)
{ 
  //If it hasn't been set yet
  if (!horizontalHeaderItem (0))
    setHorizontalHeaderItem (0, new QStandardItem (new_name));
  else
  {
    QStandardItem* header = horizontalHeaderItem (0);  
    header->setText(new_name);
  }
}

void
pcl::cloud_composer::ProjectModel::setCloudView (CloudView* view)
{
  cloud_view_ = view;
  // Initialize status variables tied to the view
  setAxisVisibility (true);
   
}

void
pcl::cloud_composer::ProjectModel::setPointSelection (boost::shared_ptr<SelectionEvent> selected_event)
{
  selection_event_ = selected_event;
  //Get all the items in this project that are clouds
  QList <CloudItem*> project_clouds;
  for (int i = 0; i < this->rowCount (); ++i)
  {
    CloudItem* cloud_item = dynamic_cast <CloudItem*> (this->item (i));
    if ( cloud_item )
      project_clouds.append ( cloud_item );
  }
  
  selected_item_index_map_.clear ();
  // Find all indices in the selected points which are present in the clouds
  foreach (CloudItem* cloud_item, project_clouds)
  {
    pcl::PointIndices::Ptr found_indices = boost::make_shared<pcl::PointIndices>();
    selected_event->findIndicesInItem (cloud_item, found_indices);
    if (found_indices->indices.size () > 0)
    {
      qDebug () << "Found "<<found_indices->indices.size ()<<" points in "<<cloud_item->text ();
      selected_item_index_map_. insert (cloud_item, found_indices);
      cloud_item->setForeground (QBrush (Qt::green));
    }
  }
  setSelectedStyle (interactor_styles::PCL_VISUALIZER);
  emit mouseStyleState (interactor_styles::PCL_VISUALIZER);
}

void
pcl::cloud_composer::ProjectModel::manipulateClouds (boost::shared_ptr<ManipulationEvent> manip_event)
{
  
  //Get all the items in this project that are clouds
  QList <CloudItem*> project_clouds;
  for (int i = 0; i < this->rowCount (); ++i)
  {
    CloudItem* cloud_item = dynamic_cast <CloudItem*> (this->item (i));
    if ( cloud_item )
      project_clouds.append ( cloud_item );
  }
  
  QMap <QString, vtkSmartPointer<vtkMatrix4x4> > transform_map = manip_event->getEndMap ();
  QList <QString> ids = transform_map.keys ();
  ConstItemList input_data;
  
  TransformClouds* transform_tool = new TransformClouds (transform_map);
  foreach (CloudItem* cloud_item, project_clouds)
  {
    if (ids.contains (cloud_item->getId ()))
    {
      qDebug () << "Found matching item for actor "<<cloud_item->getId ();
      input_data.append (cloud_item);
    }
  }
  
  //Move the tool object to the work queue thread
  transform_tool->moveToThread (work_thread_);
  //Emit signal which tells work queue to enqueue this new action
  emit enqueueNewAction (transform_tool, input_data);
  

}

void
pcl::cloud_composer::ProjectModel::insertNewCloudFromFile ()
{
  qDebug () << "Inserting cloud from file...";
  QString filename = QFileDialog::getOpenFileName (0,tr ("Select cloud to open"), last_directory_.absolutePath (), tr ("PointCloud(*.pcd)"));
  if ( filename.isNull ())
  {
    qWarning () << "No file selected, no cloud loaded";
    return;
  }
  else
  {
    QFileInfo file_info (filename);
    last_directory_ = file_info.absoluteDir ();
  }
    
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;
  
  pcl::PCDReader pcd;
  if (pcd.read (filename.toStdString (), *cloud_blob, origin, orientation, version) < 0)
  {
    qDebug () << "Failed to read cloud from file";
    return;
  }
  if (cloud_blob->width * cloud_blob->height == 0)
  {
    qDebug () << "Cloud read has zero size!";
    return;
  }
  
  QFileInfo file_info (filename);
  QString short_filename = file_info.baseName ();
  //Check if this name already exists in the project - if so, append digit
  QList <QStandardItem*> items = findItems (short_filename);
  if (items.size () > 0)
  {
    int k = 2;
    items = findItems (short_filename+ tr ("-%1").arg (k));
    while (items.size () > 0)
    {  
      ++k;
      items = findItems (short_filename+ tr ("-%1").arg (k));
    }
    short_filename = short_filename+ tr ("-%1").arg (k);
  }
  CloudItem* new_item = new CloudItem (short_filename, cloud_blob, origin, orientation, true);
   
  insertNewCloudComposerItem (new_item, invisibleRootItem());
  
}

void
pcl::cloud_composer::ProjectModel::insertNewCloudFromRGBandDepth ()
{
  qDebug () << "Inserting cloud from RGB and Depth files...";
  QString rgb_filename = QFileDialog::getOpenFileName (0,tr ("Select rgb image file to open"), last_directory_.absolutePath (), tr ("Images(*.png *.bmp *.tif *.ppm)"));
  QString depth_filename;
  if ( rgb_filename.isNull ())
  {
    qWarning () << "No file selected, no cloud loaded";
    return;
  }
  else
  {
    QFileInfo file_info (rgb_filename);
    last_directory_ = file_info.absoluteDir ();
    QString base_name = file_info.baseName ();
    QStringList depth_filter;
    depth_filter << base_name.split("_").at(0) + "_depth.*";
    last_directory_.setNameFilters (depth_filter);
    QFileInfoList depth_info_list = last_directory_.entryInfoList ();
    if (depth_info_list.size () == 0)
    {
      qCritical () << "Could not find depth file in format (rgb file base name)_depth.*";
      return;
    }
    else if (depth_info_list.size () > 1)
    {
      qWarning () << "Found more than one file which matches depth naming format, using first one!";
    }
    depth_filename = depth_info_list.at (0).absoluteFilePath ();
  }
  
  //Read the images
  vtkSmartPointer<vtkImageReader2Factory> reader_factory = vtkSmartPointer<vtkImageReader2Factory>::New ();
  vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2 (rgb_filename.toStdString ().c_str ());
  qDebug () << "RGB File="<<rgb_filename;
  if ( ! rgb_reader->CanReadFile (rgb_filename.toStdString ().c_str ()))
  {
    qCritical () << "Cannot read rgb image file!";
    return;
  }
  rgb_reader->SetFileName (rgb_filename.toStdString ().c_str ());
  rgb_reader->Update ();
  qDebug () << "Depth File="<<depth_filename;
  vtkImageReader2* depth_reader = reader_factory->CreateImageReader2 (depth_filename.toStdString ().c_str ());
  if ( ! depth_reader->CanReadFile (depth_filename.toStdString ().c_str ()))
  {
    qCritical () << "Cannot read depth image file!";
    return;
  }
  depth_reader->SetFileName (depth_filename.toStdString ().c_str ());
  depth_reader->Update ();

  vtkSmartPointer<vtkImageData> rgb_image = rgb_reader->GetOutput ();
  int *rgb_dims = rgb_image->GetDimensions ();
  vtkSmartPointer<vtkImageData> depth_image = depth_reader->GetOutput ();
  int *depth_dims = depth_image->GetDimensions ();
  
  if (rgb_dims[0] != depth_dims[0] || rgb_dims[1] != depth_dims[1])
  {
    qCritical () << "Depth and RGB dimensions to not match!";
    qDebug () << "RGB Image is of size "<<rgb_dims[0] << " by "<<rgb_dims[1];
    qDebug () << "Depth Image is of size "<<depth_dims[0] << " by "<<depth_dims[1];
    return;
  }
  qDebug () << "Images loaded, making cloud";
  PointCloud<PointXYZRGB>::Ptr cloud = boost::make_shared <PointCloud<PointXYZRGB> >();
  cloud->points.reserve (depth_dims[0] * depth_dims[1]);
  cloud->width = depth_dims[0];
  cloud->height = depth_dims[1];
  cloud->is_dense = false;
 

  // Fill in image data
  int centerX = static_cast<int>(cloud->width / 2.0);
  int centerY = static_cast<int>(cloud->height / 2.0);
  unsigned short* depth_pixel;
  unsigned char* color_pixel;
  float scale = 1.0f/1000.0f;
  float focal_length = 525.0f;
  float fl_const = 1.0f / focal_length;
  depth_pixel = static_cast<unsigned short*>(depth_image->GetScalarPointer (depth_dims[0]-1,depth_dims[1]-1,0));
  color_pixel = static_cast<unsigned char*> (rgb_image->GetScalarPointer (depth_dims[0]-1,depth_dims[1]-1,0));
  
  for (int y=0; y<cloud->height; ++y)
  {
    for (int x=0; x<cloud->width; ++x, --depth_pixel, color_pixel-=3)
    {
      PointXYZRGB new_point;
      //  uint8_t* p_i = &(cloud_blob->data[y * cloud_blob->row_step + x * cloud_blob->point_step]);
      float depth = (float)(*depth_pixel) * scale;
    //  qDebug () << "Depth = "<<depth;
      if (depth == 0.0f)
      {
        new_point.x = new_point.y = new_point.z = std::numeric_limits<float>::quiet_NaN ();
      }
      else
      {
        new_point.x = ((float)(x - centerX)) * depth * fl_const;
        new_point.y = ((float)(centerY - y)) * depth * fl_const; // vtk seems to start at the bottom left image corner
        new_point.z = depth;
      }
      
      uint32_t rgb = (uint32_t)color_pixel[0] << 16 | (uint32_t)color_pixel[1] << 8 | (uint32_t)color_pixel[2];
      new_point.rgb = *reinterpret_cast<float*> (&rgb);
      cloud->points.push_back (new_point);
      //   qDebug () << "depth = "<<depth << "x,y,z="<<data[0]<<","<<data[1]<<","<<data[2];
      //qDebug() << "r ="<<color_pixel[0]<<" g="<<color_pixel[1]<<" b="<<color_pixel[2];
      
    }
  }
  qDebug () << "Done making cloud!";
  QFileInfo file_info (rgb_filename);
  QString short_filename = file_info.baseName ();
  //Check if this name already exists in the project - if so, append digit
  QList <QStandardItem*> items = findItems (short_filename);
  if (items.size () > 0)
  {
    int k = 2;
    items = findItems (short_filename+ tr ("-%1").arg (k));
    while (items.size () > 0)
    {  
      ++k;
      items = findItems (short_filename+ tr ("-%1").arg (k));
    }
    short_filename = short_filename+ tr ("-%1").arg (k);
  }

  CloudItem* new_item = CloudItem::createCloudItemFromTemplate<PointXYZRGB> (short_filename,cloud);
  
  insertNewCloudComposerItem (new_item, invisibleRootItem());
  
}
void
pcl::cloud_composer::ProjectModel::saveSelectedCloudToFile ()
{
  qDebug () << "Saving cloud to file...";
  QModelIndexList selected_indexes = selection_model_->selectedIndexes ();
  if (selected_indexes.size () == 0)
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "No Cloud Selected", "Cannot save, no cloud is selected in the browser or cloud view");
    return;
  }
  else if (selected_indexes.size () > 1)
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "Too many clouds Selected", "Cannot save, currently only support saving one cloud at a time");
    return;
  }
  
  QStandardItem* item = this->itemFromIndex (selected_indexes.value (0));
  CloudItem* cloud_to_save = dynamic_cast <CloudItem*> (item); 
  if (!cloud_to_save )
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "Not a Cloud!", "Selected item is not a cloud, not saving!");
    return;
  }
  
  QString filename = QFileDialog::getSaveFileName (0,tr ("Save Cloud"), last_directory_.absolutePath (), tr ("PointCloud(*.pcd)"));
  if ( filename.isNull ())
  {
    qWarning () << "No file selected, not saving";
    return;
  }
  else
  {
    QFileInfo file_info (filename);
    last_directory_ = file_info.absoluteDir ();
  }
  
  pcl::PCLPointCloud2::ConstPtr cloud = cloud_to_save->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
  Eigen::Vector4f origin = cloud_to_save->data (ItemDataRole::ORIGIN).value <Eigen::Vector4f> ();
  Eigen::Quaternionf orientation = cloud_to_save->data (ItemDataRole::ORIENTATION).value <Eigen::Quaternionf> ();
  pcl::io::savePCDFile (filename.toStdString (), *cloud, origin, orientation );
  
}

void 
pcl::cloud_composer::ProjectModel::enqueueToolAction (AbstractTool* tool)
{
  qDebug () << "Enqueuing tool action "<<tool->getToolName ()<<" in project model "<<this->getName ();
  //Get the currently selected item(s), put them in a list, and create the command
  ConstItemList input_data;
  QModelIndexList selected_indexes = selection_model_->selectedIndexes ();
  if (selected_indexes.size () == 0)
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "No Items Selected", "Cannot use tool, no item is selected in the browser or cloud view");
    return;
  }
  foreach (QModelIndex index, selected_indexes)
  {
    QStandardItem* item = this->itemFromIndex (index);
    if ( dynamic_cast <CloudComposerItem*> (item))
      input_data.append (dynamic_cast <CloudComposerItem*> (item));
  }
  qDebug () << "Input for tool is "<<input_data.size () << " element(s)";
 
  //Move the tool object to the work queue thread
  tool->moveToThread (work_thread_);
  //Emit signal which tells work queue to enqueue this new action
  emit enqueueNewAction (tool, input_data);
}


void
pcl::cloud_composer::ProjectModel::commandCompleted (CloudCommand* command)
{
  //We set the project model here - this wasn't done earlier so model is never exposed to plugins
  command->setProjectModel (this);
  qDebug () << "Applying command changes to model and pushing onto undo stack";
  //command->redo ();
  undo_stack_->push (command);
  
}

void
pcl::cloud_composer::ProjectModel::insertNewCloudComposerItem (CloudComposerItem* new_item, QStandardItem* parent_item)
{
  parent_item->appendRow (new_item);  
}

///////////////////////////////////////////////////////////////
//Slots for commands arriving from GUI
///////////////////////////////////////////////////////////////
void
pcl::cloud_composer::ProjectModel::clearSelection ()
{
  getSelectionModel ()->clearSelection ();
  
  //Clear the point selector as well if it has an active selection
  if (selection_event_)
    selection_event_.reset ();
  
  foreach (CloudItem* selected_item, selected_item_index_map_.keys())
  {
    qDebug () << "Setting item color back to black";
    selected_item->setForeground (QBrush (Qt::black));;
  }
  
  selected_item_index_map_.clear ();
}

void
pcl::cloud_composer::ProjectModel::deleteSelectedItems ()
{
  
  QModelIndexList selected_indexes = selection_model_->selectedIndexes ();
  if (selected_indexes.size () == 0)
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "No Items Selected", "Cannot execute delete command, no item is selected in the browser or cloud view");
    return;
  }    
  
  ConstItemList input_data;
  foreach (QModelIndex index, selected_indexes)
  {
    QStandardItem* item = this->itemFromIndex (index);
    //qDebug () << item->text () << " selected!";
    if ( dynamic_cast <CloudComposerItem*> (item))
      input_data.append (dynamic_cast <CloudComposerItem*> (item));
  }
 // qDebug () << "Input for command is "<<input_data.size () << " element(s)";
  DeleteItemCommand* delete_command = new DeleteItemCommand (ConstItemList ());
  delete_command->setInputData (input_data);
  if (delete_command->runCommand (0))
    commandCompleted(delete_command);
  else
    qCritical () << "Execution of delete command failed!";
}

void
pcl::cloud_composer::ProjectModel::setAxisVisibility (bool visible)
{
  //qDebug () << "Setting axis visibility to "<<visible;
  axis_visible_ = visible;
  cloud_view_->setAxisVisibility (axis_visible_);
}

void
pcl::cloud_composer::ProjectModel::mouseStyleChanged (QAction* new_style_action)
{
  interactor_styles::INTERACTOR_STYLES selected_style = new_style_action->data ().value<interactor_styles::INTERACTOR_STYLES> ();
  qDebug () << "Selected style ="<<selected_style;
  setSelectedStyle (selected_style);  
  
  // Now set the correct interactor
  if (cloud_view_)
    cloud_view_->setInteractorStyle (selected_style);
  else
    qWarning () << "No Cloud View active, can't change interactor style!";

    
}

void
pcl::cloud_composer::ProjectModel::createNewCloudFromSelection ()
{
  // We need only clouds to be selected
  if (!onlyCloudItemsSelected ())
  {
    qCritical () << "Only Clouds Selected = False -- Cannot create a new cloud from non-cloud selection";
    return;
  }
  //Add selected items into input data list
  QModelIndexList selected_indexes = selection_model_->selectedIndexes ();
  ConstItemList input_data;
  foreach (QModelIndex index, selected_indexes)
  {
    QStandardItem* item = this->itemFromIndex (index);
    //qDebug () << item->text () << " selected!";
    if ( dynamic_cast <CloudComposerItem*> (item))
      input_data.append (dynamic_cast <CloudComposerItem*> (item));
  }
 
  QMap <const CloudItem*, pcl::PointIndices::ConstPtr> selected_const_map;
  foreach ( CloudItem* item, selected_item_index_map_.keys ())
    selected_const_map.insert (item, selected_item_index_map_.value (item));
  MergeSelection* merge_tool = new MergeSelection (selected_const_map);
  
  //We don't call the enqueueToolAction function since that would abort if we only have a green selection
  //Move the tool object to the work queue thread
  merge_tool->moveToThread (work_thread_);
  //Emit signal which tells work queue to enqueue this new action
  emit enqueueNewAction (merge_tool, input_data);
  
  
}

void
pcl::cloud_composer::ProjectModel::selectAllItems (QStandardItem* item)
{
 
  if (!item)
    item = this->invisibleRootItem ();
  else
   selection_model_->select (item->index (), QItemSelectionModel::Select);
  //qDebug () << "Select all!"<< item->rowCount();
  for (int i = 0; i < item->rowCount (); ++i)
  {
    if (item->child (i))
      selectAllItems(item->child (i));
  }
  
}

/////////////////////////////////////////////////////////
//Slots for Model State
////////////////////////////////////////////////////////
void
pcl::cloud_composer::ProjectModel::emitAllStateSignals ()
{
  emit axisVisible (axis_visible_);
  emit deleteAvailable (selection_model_->hasSelection ());
  emit newCloudFromSelectionAvailable (onlyCloudItemsSelected ());  
  
  //Find out which style is active, emit the signal 
  foreach (interactor_styles::INTERACTOR_STYLES style, selected_style_map_.keys())
  {
    if (selected_style_map_.value (style))
    {
      emit mouseStyleState (style);
      break;
    }
  }
  
  
}

void
pcl::cloud_composer::ProjectModel::itemSelectionChanged ( const QItemSelection &, const QItemSelection &)
{
  //qDebug () << "Item selection changed!";
  //Set all point selected cloud items back to green text, since if they are selected they get changed to white
  foreach (CloudItem* selected_item, selected_item_index_map_.keys())
  {
    selected_item->setForeground (QBrush (Qt::green));;
  }
}


//////////////////////////////////////////////////////
//  Private Support Functions
//////////////////////////////////////////////////////


bool
pcl::cloud_composer::ProjectModel::onlyCloudItemsSelected ()
{
  QModelIndexList selected_indexes = selection_model_->selectedIndexes();
  foreach (QModelIndex model_index, selected_indexes)
  {
    if (this->itemFromIndex (model_index)->type () != CloudComposerItem::CLOUD_ITEM )
    {
      return false;
    }
  }
  return true;
}

void 
pcl::cloud_composer::ProjectModel::setSelectedStyle (interactor_styles::INTERACTOR_STYLES style)
{
  QMap<interactor_styles::INTERACTOR_STYLES, bool>::iterator itr = selected_style_map_.begin();
  while (itr != selected_style_map_.end ())
  {
    itr.value() = false;
    ++itr;
  }
  selected_style_map_[style] = true;
  
}


