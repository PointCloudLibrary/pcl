#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/work_queue.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
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
pcl::cloud_composer::ProjectModel::insertNewCloudFromFile (QString filename)
{
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




