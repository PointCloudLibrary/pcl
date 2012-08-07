#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/work_queue.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>


pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
  
  last_directory_ = QDir (".");
    
  selection_model_ = new QItemSelectionModel (this);
  undo_stack_ = new QUndoStack (this);
  
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
  
  item_index_map_.clear ();
  // Find all indices in the selected points which are present in the clouds
  foreach (CloudItem* cloud_item, project_clouds)
  {
    pcl::PointIndices::Ptr found_indices = boost::make_shared<pcl::PointIndices>();
    selected_event->findIndicesInItem (cloud_item, found_indices);
    if (found_indices->indices.size () > 0)
    {
      qDebug () << "Found "<<found_indices->indices.size ()<<" points in "<<cloud_item->text ();
      item_index_map_. insert (cloud_item, found_indices);
      cloud_item->setForeground (QBrush (Qt::green));
    }
  }
    
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
    
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2);
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
  CloudItem* new_item = new CloudItem (short_filename, cloud_blob, origin, orientation);
   
  insertNewCloudComposerItem (new_item, invisibleRootItem());
  
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
pcl::cloud_composer::ProjectModel::doCommand (CloudCommand* command)
{
  ConstItemList input_data;
  QModelIndexList selected_indexes = selection_model_->selectedIndexes ();
  if (selected_indexes.size () == 0)
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "No Items Selected", "Cannot execute command, no item is selected in the browser or cloud view");
    return;
  }    
  foreach (QModelIndex index, selected_indexes)
  {
    QStandardItem* item = this->itemFromIndex (index);
    qDebug () << item->text () << " selected!";
    if ( dynamic_cast <CloudComposerItem*> (item))
      input_data.append (dynamic_cast <CloudComposerItem*> (item));
  }
  qDebug () << "Input for command is "<<input_data.size () << " element(s)";
  command->setInputData (input_data);
  if (command->runCommand (0))
    commandCompleted(command);
  else
    qCritical () << "Execution of command failed!";
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
  
  foreach (CloudItem* selected_item, item_index_map_.keys())
  {
    qDebug () << "Setting item color back to black";
    selected_item->setForeground (QBrush (Qt::black));;
  }
  
  item_index_map_.clear ();
}

void
pcl::cloud_composer::ProjectModel::deleteSelectedItems ()
{
  DeleteItemCommand* delete_command = new DeleteItemCommand (ConstItemList ());
  doCommand (delete_command);
}

void
pcl::cloud_composer::ProjectModel::setAxisVisibility (bool visible)
{
  //qDebug () << "Setting axis visibility to "<<visible;
  axis_visible_ = visible;
  cloud_view_->setAxisVisibility (axis_visible_);
}

void
pcl::cloud_composer::ProjectModel::selectRectangularFrustum ()
{
  qDebug () << "Rectangule Frustum select!";
  if (cloud_view_)
    cloud_view_->setInteractorStyle (RECTANGULAR_FRUSTUM);
  else
    qWarning () << "No Cloud View active, can't change interactor style!";
      
}

/////////////////////////////////////////////////////////
//Slots for Model State
////////////////////////////////////////////////////////
void
pcl::cloud_composer::ProjectModel::emitAllStateSignals ()
{
  emit axisVisible (axis_visible_);
  emit deleteAvailable (selection_model_->hasSelection ());
    
}

void
pcl::cloud_composer::ProjectModel::itemSelectionChanged ( const QItemSelection & selected, const QItemSelection & deselected )
{
  qDebug () << "Item selection changed!";
  //Set all point selected cloud items back to green text, since if they are selected they get changed to white
  foreach (CloudItem* selected_item, item_index_map_.keys())
  {
    selected_item->setForeground (QBrush (Qt::green));;
  }
}



