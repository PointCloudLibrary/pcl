
#include <pcl/apps/cloud_composer/commands.h>

pcl::cloud_composer::ModifyCloudCommand::ModifyCloudCommand (AbstractTool* tool, CloudItem* cloud_item, QUndoCommand* parent)
  : QUndoCommand (parent)
  , tool_ (tool)
  , cloud_item_ (cloud_item)
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