#include <QtGui>

#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_item.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/work_queue.h>

pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
  selection_model_ = new QItemSelectionModel (this);
  undo_stack_ = new QUndoStack (this);
  
  work_queue_ = new WorkQueue (this);
  connect (this, SIGNAL (enqueueNewAction (AbstractTool*, QList <const CloudComposerItem*>)),
           work_queue_, SLOT (enqueueNewAction (AbstractTool*, QList <const CloudComposerItem*>)));
}

pcl::cloud_composer::ProjectModel::ProjectModel (const ProjectModel& to_copy)
{
}

pcl::cloud_composer::ProjectModel::~ProjectModel ()
{
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
  QStandardItem* new_item = new CloudItem (short_filename, cloud_blob, origin, orientation);
   
  invisibleRootItem ()->appendRow (new_item);
}

void 
pcl::cloud_composer::ProjectModel::enqueueToolAction (AbstractTool* tool)
{
  //Get the currently selected item(s), put them in a list, and create the command
  QList <const CloudComposerItem*> input_data;
  foreach (QModelIndex index, selection_model_->selectedIndexes ())
  {
    QStandardItem* item = this->itemFromIndex (index);
    if ( dynamic_cast <CloudComposerItem*> (item))
      input_data.append (dynamic_cast <CloudComposerItem*> (item));
  }
  //Move the tool object to the work queue thread
  tool->moveToThread (work_queue_->thread ());
  //Emit signal which tells work queue to enqueue this new action
  emit enqueueNewAction (tool, input_data);
}

void
pcl::cloud_composer::ProjectModel::commandCompleted (CloudCommand* command)
{
 // TODO: THIS IS WHERE COMMAND GETS PUSHED ON UNDO STACK 
  
}

