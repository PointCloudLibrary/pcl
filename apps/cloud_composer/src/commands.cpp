#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/merge_selection.h>

pcl::cloud_composer::CloudCommand::CloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : QUndoCommand (parent)
  , original_data_ (std::move(input_data))
  , can_use_templates_(false)
  , template_type_ (-1)
{

}

pcl::cloud_composer::CloudCommand::~CloudCommand ()
{
  qDebug () << "Command Destructor";
  //If we have removed items, we delete them
  if (!last_was_undo_)
  {
    qDebug () << "Last was redo, removing original items ";
    QList <QStandardItem*> items_to_remove = removed_to_parent_map_.keys ();
    foreach (QStandardItem* to_remove, items_to_remove)
    {
        delete to_remove;
    }
  }
  else
  {
    qDebug () << "Last was undo, removing new items";
    foreach (OutputPair output_pair, output_data_)
    {
      QList <CloudComposerItem*> new_items = output_pair.output_items_;
      foreach (CloudComposerItem* item, new_items)
      {
        delete item;
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::CloudCommand::setProjectModel (ProjectModel* model)
{
  project_model_ = model;
}

bool
pcl::cloud_composer::CloudCommand::canUseTemplates (ConstItemList &input_data)
{
  //Make sure the input list isn't empty
  if (input_data.empty ())
  {
    qCritical () << "Cannot call a templated tool on an empty input in CloudCommand::executeToolOnTemplateCloud!";
    template_type_ = -2;
    return false;
  }
  //Make sure all input items are clouds
  QList <const CloudItem*> cloud_items;
  foreach (const CloudComposerItem* item, input_data)
  {
    const CloudItem* cloud_item = dynamic_cast<const CloudItem*> (item);
    if (cloud_item)
      cloud_items.append (cloud_item);
  }
  if (cloud_items.size () != input_data.size ())
  {
    qCritical () << "All input items are not clouds in CloudCommand::executeToolOnTemplateCloud!";
    template_type_ = -3;
    return false;
  }
  
  // Now make sure all input clouds have the same templated type
  int type = cloud_items.value (0)->getPointType ();
  foreach (const CloudItem* cloud_item, cloud_items)
  {
    if (cloud_item->getPointType () != type)
    {
      qCritical () << "All input point cloud template types in CloudCommand::executeToolOnTemplateCloud are not the same!";
      qCritical () << cloud_item->text () << "'s type does not match "<<cloud_items.value (0)->type ();
      template_type_ = -3;
      return false;
    }
  }
  template_type_ = type;
  can_use_templates_ = true;
  return true;
}

/*
QList <pcl::cloud_composer::CloudComposerItem*> 
pcl::cloud_composer::CloudCommand::executeToolOnTemplateCloud (AbstractTool* tool, ConstItemList &input_data)
{
  QList <CloudComposerItem*> output;
  // If can_use_templates_ is false and type is -1 we haven't checked if we can yet
  if (!can_use_templates_ && template_type_ == -1)
    this->canUseTemplates (input_data);
  
  
  //If this is true now, we can go ahead and run it
  if (can_use_templates_ && template_type_ >= 0)
  {
    output = tool->performAction( input_data, static_cast<PointTypeFlags::PointType> (template_type_));
  }
  else
  {
    qDebug () << "Tried CloudCommand::executeToolOnTemplateCloud but input data was not templated clouds!";
  }
  return output;
  
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
pcl::cloud_composer::CloudCommand::replaceOriginalWithNew (const QList <const CloudComposerItem*>& originals, const QList <CloudComposerItem*>& new_items)
{ 
  //Find the input item's parent
  if (originals.empty ())
  {
    qCritical () << "No items to replace specified!";
    return false;
  }
    
  QStandardItem* parent_item = originals.value(0)->parent ();
  //Verify that all items have same parent
  foreach (const CloudComposerItem* item, originals)
  {
    if (item->parent () != parent_item)
    {
      qCritical () << "All original items must have same parent!";
      return false;
    }
  }
  // If parent is 0, it's parent is invisiblerootitem (That's how Qt defines it... boo!)
  if (parent_item == nullptr)
    parent_item = project_model_->invisibleRootItem ();
  
  //Now remove all the originals
  foreach (const CloudComposerItem* item, originals)
  {
    QPersistentModelIndex original_index = QPersistentModelIndex(project_model_->indexFromItem (item));
    if (!original_index.isValid ())
    {
      qCritical () << "Index of item to replace is not valid!";
      return false;
    }
    QList <QStandardItem*> removed_items = parent_item->takeRow (original_index.row ());
    removed_to_parent_map_.insert (removed_items.value(0),parent_item);
  }
  //Insert the new items below the parent item'
  foreach (CloudComposerItem* item, new_items)
  {
    parent_item->appendRow (item);   
  }

  return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
pcl::cloud_composer::CloudCommand::restoreOriginalRemoveNew (const QList <const CloudComposerItem*>& originals, const QList <CloudComposerItem*>& new_items)
{ 
  
  //Now remove all the new items
  foreach (CloudComposerItem* item, new_items)
  {
    QStandardItem* parent_item = item->parent ();
    // If parent is 0, it's parent is invisiblerootitem (That's how Qt defines it... boo!)
    if (parent_item == nullptr)
      parent_item = project_model_->invisibleRootItem ();
    QPersistentModelIndex to_remove_index = QPersistentModelIndex(project_model_->indexFromItem (item));
    if (!to_remove_index.isValid ())
    {
      qCritical () << "Index of item to remove while restoring originals not valid";
      return false;
    }
    //Take them, they're still stored so we don't worry about them
    QList <QStandardItem*> removed = parent_item->takeRow (to_remove_index.row ());
  }
  //Restore the original items
  foreach (const CloudComposerItem* item, originals)
  {
    //Point iterator to the correct spot
    // Find doesn't modify parameter so it should accept a const pointer, but it can't be because it is templated to the map type
    // So we hack to get around this with a const cast
    const auto& itr = removed_to_parent_map_.find (const_cast<CloudComposerItem*> (item));
    QStandardItem* parent = itr.value ();
    QStandardItem* original = itr.key ();
    parent->appendRow (original);
    int num = removed_to_parent_map_.remove (original);
    if (num > 1)
      qCritical () << "More than one item with same pointer in removed_to_parent_map_, this is undefined behavior";
    else if (num == 0) 
      qCritical () << "Could not find key in map to remove, not good!";
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// MODIFY CLOUD COMMAND

pcl::cloud_composer::ModifyItemCommand::ModifyItemCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (std::move(input_data), parent)
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
    QList <const CloudComposerItem*> input_list;
    input_list.append (item);
    QList <CloudComposerItem*> output;
    if (canUseTemplates(input_list))
      output = tool->performAction (input_list, static_cast<PointTypeFlags::PointType> (template_type_));
    else
      output = tool->performAction (input_list);
    if (output.empty ())
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a ModifyItemCommand";
    else 
    {
      OutputPair output_pair = {input_list, output};
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
  return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::ModifyItemCommand::undo ()
{
  last_was_undo_ = true;
  qDebug () << "Undo in ModifyItemCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    if (!restoreOriginalRemoveNew (output_pair.input_items_, output_pair.output_items_))
      qCritical() << "Failed to restore original items in ModifyItemCommand::undo!";
  }
  
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::ModifyItemCommand::redo ()
{
  last_was_undo_ = false;
  foreach (OutputPair output_pair, output_data_)
  {
    //Replace the input with the output for this pair
    if (! replaceOriginalWithNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Replacement of old items with new failed in ModifyItemCommand!";
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// New Item CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::NewItemCloudCommand::NewItemCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (std::move(input_data), parent)
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
    QList <const CloudComposerItem*> input_list;
    input_list.append (item);
    QList <CloudComposerItem*> output;
    if (canUseTemplates(input_list))
      output = tool->performAction (input_list, static_cast<PointTypeFlags::PointType> (template_type_));
    else
      output = tool->performAction (input_list);
    if (output.empty ())
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a NewItemCloudCommand";
    else 
    {
      OutputPair output_pair = {input_list, output};
      output_data_.append (output_pair);
      num_new_items += output.size ();
    }
    
  }
  if (num_new_items > 0)
  {
    qDebug () << "New Item command generated "<<num_new_items<<" new items";
    return true;
  }
  qWarning () << "New Item command generated no new items!";
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::NewItemCloudCommand::undo ()
{
  last_was_undo_ = true;
  qDebug () << "Undo in NewItemCloudCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    //Each pair can only have one input item, so get it
    const CloudComposerItem* const_input_item = output_pair.input_items_.value (0);
    QList <CloudComposerItem*> output_items = output_pair.output_items_;
    //Find the input_item index in the project_model_
    QModelIndex input_index = project_model_->indexFromItem (const_input_item);
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid, cannot undo NewItemCloudCommand";
      return;
    }
    //Get a copy of the input item we can modify
    QStandardItem* item_to_change = project_model_->itemFromIndex (input_index);
    //Remove the items we added 
    foreach (CloudComposerItem* output_item, output_items)
    {
      QModelIndex output_index = project_model_->indexFromItem (output_item);
      item_to_change->takeRow (output_index.row ());
    }
    
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::NewItemCloudCommand::redo ()
{
  last_was_undo_ = false;
  qDebug () << "Redo in NewItemCloudCommand - output data size="<<output_data_.size ();
  foreach (OutputPair output_pair, output_data_)
  {
    //Each pair can only have one input item, so get it
    const CloudComposerItem* const_input_item = output_pair.input_items_.value (0);
    QList <CloudComposerItem*> output_items = output_pair.output_items_;
    //Find the input_item index in the project_model_
    QPersistentModelIndex input_index = QPersistentModelIndex(project_model_->indexFromItem (const_input_item));
    if (!input_index.isValid ())
    {
      qCritical () << "Index of input cloud item is no longer valid upon command completion!";
      return;
    }
    //Get a copy of the input item we can modify
    QStandardItem* input_item = (project_model_->itemFromIndex (input_index));
    //Append the output items to the input item
    foreach (CloudComposerItem* output_item, output_items)
    {
      input_item->appendRow (output_item);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// Split CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::SplitCloudCommand::SplitCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (std::move(input_data), parent)
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
    QList <const CloudComposerItem*> input_list;
    input_list.append (item);
    QList <CloudComposerItem*> output;
    if (canUseTemplates(input_list))
      output = tool->performAction (input_list, static_cast<PointTypeFlags::PointType> (template_type_));
    else
      output = tool->performAction (input_list);
    if (output.empty ())
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a SplitCloudCommand";
    else 
    {
      qDebug () << "Split command returned "<<output.size ()<<" items";
      OutputPair output_pair = {input_list, output};
      output_data_.append (output_pair);
      num_new_items += output.size ();
    }
  }
  if (num_new_items > 0)
  {
    qDebug () << "Split Item command generated "<<num_new_items<<" new items";
    return true;
  }
  qWarning () << "Split Item command generated no new items!";
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::SplitCloudCommand::undo ()
{
  last_was_undo_ = true;
  qDebug () << "Undo in SplitItemCloudCommand";
  foreach (OutputPair output_pair, output_data_)
  {
    if (!restoreOriginalRemoveNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Failed to restore old cloud in SplitCloudCommand::undo!";
  }
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::SplitCloudCommand::redo ()
{
  last_was_undo_ = false;
  qDebug () << "Redo in SplitItemCloudCommand - output data size="<<output_data_.size ();
  foreach (OutputPair output_pair, output_data_)
  {
    //Replace the input with the output for this pair
    if (! replaceOriginalWithNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Replacement of old items with new failed in ModifyItemCommand!";
  
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// Delete CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::DeleteItemCommand::DeleteItemCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (std::move(input_data), parent)
{
  
}

bool
pcl::cloud_composer::DeleteItemCommand::runCommand (AbstractTool*)
{
  
  //For delete item command, each selected item should be processed separately
  //e.g. delete every selected item
  foreach (const CloudComposerItem *item, original_data_)
  {
    QList <CloudComposerItem*> output;
    QList <const CloudComposerItem*> to_delete;
    to_delete.append (item);
    OutputPair output_pair = {to_delete, output};
    output_data_.append (output_pair);
    this->setText ("Delete "+item->text ());
  }
  if (!original_data_.empty ())
    this->setText ("Delete multiple items");
  return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::DeleteItemCommand::undo ()
{
  last_was_undo_ = true;
  //Add the original items back into the model
  foreach (OutputPair output_pair, output_data_)
  {
    if (!restoreOriginalRemoveNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Failed to restore items in DeleteItemCommand::undo!";
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::DeleteItemCommand::redo ()
{
  last_was_undo_ = false;
  qDebug () << "Redo in DeleteItemCommand - num items to delete="<<output_data_.size ();
  foreach (OutputPair output_pair, output_data_)
  {
    //Replace the input with the output for this pair
    if (! replaceOriginalWithNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Removal of items failed in DeleteItemCommand::redo";
  
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// MERGE CLOUD COMMAND ///////////////////////////////////////////////////////////////////////////

pcl::cloud_composer::MergeCloudCommand::MergeCloudCommand (ConstItemList input_data, QUndoCommand* parent)
  : CloudCommand (std::move(input_data), parent)
{
  
}

bool
pcl::cloud_composer::MergeCloudCommand::runCommand (AbstractTool* tool)
{
  //In merge command, input clouds will be combined, so send them to tool together
  QList <CloudComposerItem*> output_items;
  if (canUseTemplates(original_data_))
    output_items = tool->performAction (original_data_, static_cast<PointTypeFlags::PointType> (template_type_));
  else
    output_items = tool->performAction (original_data_);
  MergeSelection* merge_selection = dynamic_cast <MergeSelection*> (tool);
  // If this is a merge selection we need to put the partially selected items into the original data list too!
  // We didn't send them before because merge selection already knows about them (and needs to tree input list differently from selected items)
  if (merge_selection)
  {
    QList <const CloudItem*> selected_items = merge_selection->getSelectedItems();
    foreach (const CloudItem* item, selected_items)
      original_data_.append (item);
  }
  OutputPair output_pair = {original_data_, output_items};
  output_data_.append (output_pair);
  
  if (output_items.empty ())
  {
    qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a MergeCloudCommand";
    return false;
  }
  
  this->setText ("Merge Clouds");
  return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::MergeCloudCommand::undo ()
{
  last_was_undo_ = true;
  //Add the original items back into the model
  foreach (OutputPair output_pair, output_data_)
  {
    if (!restoreOriginalRemoveNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Failed to restore original clouds in MergeCloudCommand::undo!";
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::cloud_composer::MergeCloudCommand::redo ()
{
  qDebug () << "Redo in MergeCloudCommand ";
  last_was_undo_ = false;
  //There is only one output_pair, but that's ok
  foreach (OutputPair output_pair, output_data_)
  {
    //Replace the input with the output for this pair
    // This works because all input clouds must have same parent, the root item (clouds must be on top level)
    if (! replaceOriginalWithNew (output_pair.input_items_, output_pair.output_items_))
      qCritical () << "Removal of items failed in MergeCloudCommand::redo";
  
  }
}

