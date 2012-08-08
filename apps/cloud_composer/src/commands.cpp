#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/merge_selection.h>

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

pcl::cloud_composer::ModifyItemCommand::ModifyItemCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{ 
  
}

bool
pcl::cloud_composer::ModifyItemCommand::runCommand (AbstractTool* tool)
{
  this->setText (tool->getToolName ());
  //For modify item cloud command, each selected item should be processed separately
  int num_items_returned = 0;
  foreach (const CloudComposerItem *item, original_data_)
  {
    //Check to see if this is a cloud
    QList <const CloudComposerItem*> working_copy;
    working_copy.append (item);
    QList <CloudComposerItem*> output = tool->performAction (working_copy);
    if (output.size () == 0)
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a ModifyItemCommand";
    else 
    {
      OutputPair output_pair = {item, output};
      output_data_.append (output_pair);
      num_items_returned++;
    }
    if (output.size () > 1)
    {
      qCritical () << "Tool " << tool->getToolName () << "returned multiple items in a ModifyCloudCommand";
    }
    
  }
  if (num_items_returned != original_data_.size ())
  {
    qDebug () << "Modify Item command generated "<<num_items_returned<<" which does not match input of "<<original_data_.size () <<" items";
    return true;
  }
  else
  {
    return true;
  }
}

void
pcl::cloud_composer::ModifyItemCommand::undo ()
{
  qDebug () << "Undo in ModifyItemCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    //Remove the items we added
    foreach (CloudComposerItem* output_item, output_items)
    {
      QModelIndex output_index = project_model_->indexFromItem (output_item);
      project_model_->takeRow (output_index.row ());
    }  
  }
  //Add the original items back into the model
  if (original_item_parent_pairs_.size() > 0)
  {
    foreach (RemovedPair removed_pair, original_item_parent_pairs_)
    {
      QModelIndex parent_index = removed_pair.second;
      QStandardItem* parent_item;
      if ( !parent_index.isValid ())
        parent_item = project_model_->invisibleRootItem ();
      else
        parent_item = project_model_->itemFromIndex (parent_index);
      //Add the item back into the model
      qDebug () << "Parent item = "<<parent_item;
      parent_item->appendRow (removed_pair.first);
    }
  }
  else
    qCritical () << "Original item is empty, can't reinsert it back into project in ModifyCloudCommand::undo";
}

void
pcl::cloud_composer::ModifyItemCommand::redo ()
{
  //Clear the removed_item_parent_pairs_ storage, as this might not be the first time we call redo
  original_item_parent_pairs_.clear ();
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    if (!input_index.isValid ())
    {
      qCritical () << "Index of item to delete is not valid!";
      return;
    }
    //Find the input item's parent
    // If parent returns 0, it's parent is invisiblerootitem (That's how Qt defines it... boo!)
    QStandardItem* parent_item = input_item->parent ();
    if (parent_item == 0)
      parent_item = project_model_->invisibleRootItem ();
    QPersistentModelIndex parent_index;
    parent_index = QPersistentModelIndex (parent_item->index ());
    //Insert the modified item in the position of the original
    parent_item->insertRow (input_index.row (), output_pair.output_list_.at (0));   
    //Remove the item from the model, we store it to keep its parent
    QList <QStandardItem*> original_items = parent_item->takeRow (input_index.row ());
    
    if (original_items.size () > 1)
      qCritical () << "Modified row with more than one column! This isn't supported!!";
    else
      original_item_parent_pairs_.append (qMakePair (original_items.at (0), parent_index));
    
  }

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
  //For split cloud command, each selected item should be processed separately
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
    
  }
  //Add the original items back into the model
  if (removed_item_parent_pairs_.size() > 0)
  {
    foreach (RemovedPair removed_pair, removed_item_parent_pairs_)
    {
      QModelIndex parent_index = removed_pair.second;
      QStandardItem* parent_item;
      if ( !parent_index.isValid ())
        parent_item = project_model_->invisibleRootItem ();
      else
        parent_item = project_model_->itemFromIndex (parent_index);
      //Add the item back into the model
      qDebug () << "Parent item = "<<parent_item;
      parent_item->appendRow (removed_pair.first);
    }
  }
  else
    qCritical () << "Original item is empty, can't reinsert it back into project in SplitCloudCommand::undo!!";
}

void
pcl::cloud_composer::SplitCloudCommand::redo ()
{
  qDebug () << "Redo in SplitItemCloudCommand - output data size="<<output_data_.size ();
  //Clear the removed_item_parent_pairs_ storage, as this might not be the first time we call redo
  removed_item_parent_pairs_.clear ();
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    
    //Find the original item's parent
    // If parent returns 0, it's parent is invisiblerootitem (That's how Qt defines it... boo!)
    QStandardItem* parent_item = input_item->parent ();
    if (parent_item == 0)
      parent_item = project_model_->invisibleRootItem ();
    QPersistentModelIndex parent_index;
    parent_index = QPersistentModelIndex (parent_item->index ());
    //Remove the item from the model, store it
    QList <QStandardItem*> removed_items = parent_item->takeRow (input_index.row ());
    if (removed_items.size () > 1)
      qCritical () << "Deleted row with more than one column! This isn't supported!!";
    else
      removed_item_parent_pairs_.append (qMakePair (removed_items.at (0), parent_index));
    
    //Add the output items into the model
    QList <CloudComposerItem*> output_items = output_pair.output_list_;
    foreach (CloudComposerItem* output_item, output_items)
    {
      project_model_->appendRow (output_item);
    }
  }
}


//////////// Delete CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::DeleteItemCommand::DeleteItemCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::DeleteItemCommand::runCommand (AbstractTool* tool)
{
  
  //For delete item command, each selected item should be processed separately
  //e.g. delete every selected item
  int num_new_items = 0;
  foreach (const CloudComposerItem *item, original_data_)
  {
    QList <CloudComposerItem*> output;
    OutputPair output_pair = {item, output};
    output_data_.append (output_pair);
    this->setText ("Delete "+item->text ());
  }
  return true;
}

void
pcl::cloud_composer::DeleteItemCommand::undo ()
{
  //Add the original items back into the model
  if (removed_item_parent_pairs_.size() > 0)
  {
    foreach (RemovedPair removed_pair, removed_item_parent_pairs_)
    {
      QModelIndex parent_index = removed_pair.second;
      QStandardItem* parent_item;
      if ( !parent_index.isValid ())
        parent_item = project_model_->invisibleRootItem ();
      else
        parent_item = project_model_->itemFromIndex (parent_index);
      //Add the item back into the model
      qDebug () << "Parent item = "<<parent_item;
      parent_item->appendRow (removed_pair.first);
    }
  }
}

void
pcl::cloud_composer::DeleteItemCommand::redo ()
{
  qDebug () << "Redo in DeleteItemCommand - num items to delete="<<output_data_.size ();
  //Clear the removed_item_parent_pairs_ storage, as this might not be the first time we call redo
  removed_item_parent_pairs_.clear ();
  foreach (OutputPair output_pair, output_data_)
  {
    const CloudComposerItem* input_item = output_pair.input_item_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    if (!input_index.isValid ())
    {
      qCritical () << "Index of item to delete is not valid!";
      return;
    }
    //Find the deleted item's parent
    // If parent returns 0, it's parent is invisiblerootitem (That's how Qt defines it... boo!)
    QStandardItem* parent_item = input_item->parent ();
    if (parent_item == 0)
      parent_item = project_model_->invisibleRootItem ();
    QPersistentModelIndex parent_index;
    parent_index = QPersistentModelIndex (parent_item->index ());
    //Remove the item from the model, store it
    QList <QStandardItem*> removed_items = parent_item->takeRow (input_index.row ());
    
    if (removed_items.size () > 1)
      qCritical () << "Deleted row with more than one column! This isn't supported!!";
    else
      removed_item_parent_pairs_.append (qMakePair (removed_items.at (0), parent_index));  
    

  }
}


//////////// MERGE CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::MergeCloudCommand::MergeCloudCommand (ConstItemList input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::MergeCloudCommand::runCommand (AbstractTool* tool)
{
 
  output_items_ = tool->performAction (original_data_);
  MergeSelection* merge_selection = dynamic_cast <MergeSelection*> (tool);
  // If this is a merge selection we need to put the original items into the list too!
  if (merge_selection)
  {
    QList <const CloudItem*> selected_items = merge_selection->getSelectedItems();
    foreach (const CloudItem* item, selected_items)
      original_data_.append (item);
  }
  if (output_items_.size () == 0)
  {
    qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a MergeCloudCommand";
    return false;
  }
  this->setText ("Merge Clouds");
  return true;
}

void
pcl::cloud_composer::MergeCloudCommand::undo ()
{
  foreach (CloudComposerItem* output_item, output_items_)
  {
    QModelIndex output_index = project_model_->indexFromItem (output_item);
    project_model_->takeRow (output_index.row ());
  }
    
  //Add the original items back into the model
  foreach ( QStandardItem* removed_item, removed_items_)
  {
    project_model_->appendRow (removed_item);
  }
}

void
pcl::cloud_composer::MergeCloudCommand::redo ()
{
  qDebug () << "Redo in MergeCloudCommand ";
  removed_items_.clear ();
  foreach ( const CloudComposerItem* input_item, original_data_)
  {
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (input_item));
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    
    QList <QStandardItem*> removed_items = project_model_->takeRow (input_index.row ());
    if (removed_items.size () > 1)
      qCritical () << "Deleted row with more than one column! This isn't supported!!";
    removed_items_.append (removed_items.value (0));
  }
  
  foreach (CloudComposerItem* output_item, output_items_)
  {
    project_model_->appendRow (output_item);
  }
}