#include <QtGui>

#include <pcl/apps/cloud_composer/toolbox_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>

pcl::cloud_composer::ToolBoxModel::ToolBoxModel (QObject* parent)
: QStandardItemModel (parent)
{


  
}

pcl::cloud_composer::ToolBoxModel::ToolBoxModel (const ToolBoxModel& to_copy)
{
}

pcl::cloud_composer::ToolBoxModel::~ToolBoxModel ()
{
}

void
pcl::cloud_composer::ToolBoxModel::addTool (ToolFactory* tool_factory)
{
  qDebug () << "Icon name:"<< tool_factory->getIconName ();
  QIcon new_tool_icon = QIcon (tool_factory->getIconName ());
  QStandardItem* new_tool_item = new QStandardItem (new_tool_icon, tool_factory->getPluginName ());
  new_tool_item->isCheckable ();
  
  QStandardItem* group_item = addToolGroup (tool_factory->getToolGroupName ());
  group_item->appendRow (new_tool_item); 
  
  
}

QStandardItem*
pcl::cloud_composer::ToolBoxModel::addToolGroup (QString tool_group_name)
{
  QList <QStandardItem*> matches_name = findItems (tool_group_name);
  if (matches_name.size () == 0)
  {
    QStandardItem* new_group_item = new QStandardItem (tool_group_name);
    appendRow (new_group_item);
    new_group_item->setSelectable (false);
    new_group_item->setEditable (false);
    return new_group_item;
  }
  else if (matches_name.size () > 1)
  {
    qWarning () << "Multiple tool groups with same name in ToolBoxModel!!";
  }
  
  return matches_name.value (0);
  
}