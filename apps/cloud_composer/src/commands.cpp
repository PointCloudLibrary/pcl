
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <QDebug>

pcl::cloud_composer::CloudCommand::CloudCommand (QUndoCommand* parent)
  : QUndoCommand (parent)
{
  
}


//////////// MODIFY CLOUD COMMAND

pcl::cloud_composer::ModifyCloudCommand::ModifyCloudCommand (QList <const CloudComposerItem*> input_data, QUndoCommand* parent)
  : CloudCommand (parent)
  , original_data_ (input_data)
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
  : CloudCommand (parent)
  , original_data_ (input_data)
{
  
}

bool
pcl::cloud_composer::NewItemCloudCommand::runCommand (AbstractTool* tool)
{
  
  
}

void
pcl::cloud_composer::NewItemCloudCommand::undo ()
{
  
}

void
pcl::cloud_composer::NewItemCloudCommand::redo ()
{
  
}