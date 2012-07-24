#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::CloudCommand::CloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : QUndoCommand (parent)
  , original_data_ (input_data)
{
  
}

void
pcl::cloud_composer::CloudCommand::setProjectModel (ProjectModel* model)
{
  project_model_ = model;
}

//////////// MODIFY CLOUD COMMAND

pcl::cloud_composer::ModifyCloudCommand::ModifyCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{  
}

bool
pcl::cloud_composer::ModifyCloudCommand::runCommand (AbstractTool*)
{
  return (true);
}

void
pcl::cloud_composer::ModifyCloudCommand::undo ()
{
  
}

void
pcl::cloud_composer::ModifyCloudCommand::redo ()
{
  
}

//////////// New Item CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::NewItemCloudCommand::NewItemCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::NewItemCloudCommand::runCommand (AbstractTool* tool)
{
  this->setText (tool->getToolName ());
  //For new item cloud command, each selected item should be processed separately
  //e.g. calculate normals for every selected cloud
  int num_new_items = 0;
  foreach (const CloudComposerItem *item, original_data_)
  {
    //Check to see if this is a cloud
    QList <const CloudComposerItem*> working_copy;
    working_copy.append (item);
    QList <CloudComposerItem*> output = tool->performAction (working_copy);
    if (output.size () == 0)
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a NewItemCloudCommand";
    else 
    {
      OutputPair output_pair = {item, output};
      output_data_.append (output_pair);
      num_new_items += output.size ();
    }
    if (output.size () >= 1)
    {
      qWarning () << "Tool " << tool->getToolName () << "returned multiple items in a NewItemCloudCommand";
    }
    
  }
  if (num_new_items > 0)
  {
    qDebug () << "New Item command generated "<<num_new_items<<" new items";
    return true;
  }
  else
  {
    qWarning () << "New Item command generated no new items!";
    return false;
  }
   
}

void
pcl::cloud_composer::NewItemCloudCommand::undo ()
{
  qDebug () << "Undo in NewItemCloudCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    //Find the input_item index in the project_model_
    QModelIndex input_index = project_model_->indexFromItem (input_item);
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    QStandardItem* item_to_change = project_model_->itemFromIndex (input_index);
    foreach (CloudComposerItem* output_item, output_items)
    {
      QModelIndex output_index = project_model_->indexFromItem (output_item);
      //item_to_change->takeChild (output_index.row (),output_index.column ());
      item_to_change->takeRow (output_index.row ());
    }
    
  }
}

void
pcl::cloud_composer::NewItemCloudCommand::redo ()
{
  qDebug () << "Redo in NewItemCloudCommand - output data size="<<output_data_.size ();
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    foreach (CloudComposerItem* output_item, output_items)
    {
      (project_model_->itemFromIndex (input_index))->appendRow (output_item);
      QPersistentModelIndex output_index = QPersistentModelIndex(project_model_->indexFromItem (output_item));
      if(!output_index.isValid())
        qDebug () << "OUTPUT INDEX NOT VALID";
      input_index = project_model_->indexFromItem (input_item);
    }
  }
}


//////////// Split CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::SplitCloudCommand::SplitCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::SplitCloudCommand::runCommand (AbstractTool* tool)
{
  this->setText (tool->getToolName ());
  //For new item cloud command, each selected item should be processed separately
  //e.g. calculate normals for every selected cloud
  int num_new_items = 0;
  foreach (const CloudComposerItem *item, original_data_)
  {
    //Check to see if this is a cloud
    QList <const CloudComposerItem*> working_copy;
    working_copy.append (item);
    QList <CloudComposerItem*> output = tool->performAction (working_copy);
    if (output.size () == 0)
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a SplitCloudCommand";
    else 
    {
      qDebug () << "Split command returned "<<output.size ()<<" items";
      OutputPair output_pair = {item, output};
      output_data_.append (output_pair);
      num_new_items += output.size ();
    }
  }
  if (num_new_items > 0)
  {
    qDebug () << "Split Item command generated "<<num_new_items<<" new items";
    return true;
  }
  else
  {
    qWarning () << "Split Item command generated no new items!";
    return false;
  }

}

void
pcl::cloud_composer::SplitCloudCommand::undo ()
{
  qDebug () << "Undo in SplitItemCloudCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    //Remove the items we added
    foreach (CloudComposerItem* output_item, output_items)
    {
      QModelIndex output_index = project_model_->indexFromItem (output_item);
      project_model_->takeRow (output_index.row ());
    }
    //Add the original item back into the model
    if (removed_items_.size() > 0)
    {
      foreach (QStandardItem* removed_item, removed_items_)
        project_model_->appendRow (removed_item);
    }
    else
      qCritical () << "Original item is empty, can't reinsert it back into project in SplitCloudCommand::undo!!";
  }
}

void
pcl::cloud_composer::SplitCloudCommand::redo ()
{
  qDebug () << "Redo in SplitItemCloudCommand - output data size="<<output_data_.size ();
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    //Remove the original item from the model, store it
    removed_items_ = project_model_->takeRow (input_index.row ());
    foreach (CloudComposerItem* output_item, output_items)
    {
      project_model_->appendRow (output_item);
    }
  }
}