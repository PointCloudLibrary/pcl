#include <QTreeView>
#include <QMessageBox>

#include <pcl/apps/cloud_composer/toolbox_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>


pcl::cloud_composer::ToolBoxModel::ToolBoxModel (QTreeView* tool_view, QTreeView* parameter_view_, QObject* parent)
: QStandardItemModel (parent)
, tool_view_ (tool_view)
, parameter_view_ (parameter_view_)
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
  //qDebug () << "Icon name:"<< tool_factory->getIconName ();
  QIcon new_tool_icon = QIcon (tool_factory->getIconName ());
  QStandardItem* new_tool_item = new QStandardItem (new_tool_icon, tool_factory->getPluginName ());
  new_tool_item->setEditable (false);
  
  new_tool_item->setData (QVariant::fromValue (tool_factory), FACTORY);
  QStandardItemModel* new_tool_parameters= tool_factory->createToolParameterModel (this);  
  new_tool_item->setData (QVariant::fromValue (new_tool_parameters), PARAMETER_MODEL);
  
  QStandardItem* group_item = addToolGroup (tool_factory->getToolGroupName ());
  group_item->appendRow (new_tool_item); 
  //Expand the view for this tool group
  QModelIndex group_index = this->indexFromItem(group_item);
  tool_view_->setExpanded (group_index, true);
  
}

void
pcl::cloud_composer::ToolBoxModel::setSelectionModel (QItemSelectionModel* selection_model)
{
  selection_model_ =selection_model;
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

void
pcl::cloud_composer::ToolBoxModel::selectedToolChanged (const QModelIndex & current, const QModelIndex & previous)
{
  //qDebug() << "Selected Tool changed";
  if (!parameter_view_)
  {
    qCritical () << "Toolbox parameter view not set!!!";
    return;
  }  
  QVariant parameter_model = current.data (PARAMETER_MODEL);
  parameter_view_->setModel ( parameter_model.value <QStandardItemModel*> ());
  
}


void
pcl::cloud_composer::ToolBoxModel::toolAction ()
{
  QModelIndex current_index = selection_model_->currentIndex ();
  if (!current_index.isValid ())
  {
    QMessageBox::warning (qobject_cast<QWidget *>(this->parent ()), "No Tool Selected", "Cannot execute action, no tool selected!");
    return;
  }
  ToolFactory* tool_factory = (current_index.data (FACTORY)).value <ToolFactory*> ();
  QStandardItemModel* parameter_model = (current_index.data (PARAMETER_MODEL)).value <QStandardItemModel*> ();
  AbstractTool* tool = tool_factory->createTool (parameter_model);
  
  emit enqueueToolAction (tool);
}