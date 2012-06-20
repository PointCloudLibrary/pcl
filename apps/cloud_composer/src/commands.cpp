
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <QDebug>

pcl::cloud_composer::CloudCommand::CloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : QUndoCommand (parent)
  , original_data_(input_data)
{
  
}


//////////// MODIFY CLOUD COMMAND

pcl::cloud_composer::ModifyCloudCommand::ModifyCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::ModifyCloudCommand::runCommand (AbstractTool* tool)
{
  
  
}

void
pcl::cloud_composer::ModifyCloudCommand::undo ()
{
  
}

void
pcl::cloud_composer::ModifyCloudCommand::redo ()
{
  
}

//////////// Create CLOUD COMMAND

pcl::cloud_composer::NewItemCloudCommand::NewItemCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (input_data, parent)
{
  
}

bool
pcl::cloud_composer::NewItemCloudCommand::runCommand (AbstractTool* tool)
{
  
  //For new item cloud command, each selected item should be processed separately
  //e.g. calculate normals for every selected cloud
  foreach (const CloudComposerItem *item, original_data_)
  {
    //Check to see if this is a cloud
    QList <const CloudComposerItem*> working_copy;
    working_copy.append (item);
    QList <CloudComposerItem*> output = tool->performAction (working_copy);
    if (output.size () == 0)
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned no item in a NewItemCloudCommand";
    else if (output.size () == 1)
    {
      //Create a copy of the original item
      CloudComposerItem* copy = item->clone ();
      ParentChildPair output_pair = {copy, output};
      output_data_.append (output_pair);
    }
    else
    {
      qWarning () << "Warning: Tool " << tool->getToolName () << "returned multiple items in a NewItemCloudCommand";
    }
    
  }
  
}

void
pcl::cloud_composer::NewItemCloudCommand::undo ()
{
  
}

void
pcl::cloud_composer::NewItemCloudCommand::redo ()
{
  
}