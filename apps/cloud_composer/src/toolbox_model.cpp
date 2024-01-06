#include <pcl/apps/cloud_composer/toolbox_model.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>

#include <QMessageBox>
#include <QTreeView>

pcl::cloud_composer::ToolBoxModel::ToolBoxModel (QTreeView* tool_view, QTreeView* parameter_view_, QObject* parent)
: QStandardItemModel (parent)
, tool_view_ (tool_view)
, parameter_view_ (parameter_view_)
, project_model_ (nullptr)
{
  
}

pcl::cloud_composer::ToolBoxModel::ToolBoxModel (const ToolBoxModel&)
: QStandardItemModel ()
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
  PropertiesModel* new_tool_parameters= tool_factory->createToolParameterModel (this);  
  new_tool_item->setData (QVariant::fromValue (new_tool_parameters), PARAMETER_MODEL);
  
  tool_items.insert (new_tool_item);
  QStandardItem* group_item = addToolGroup (tool_factory->getToolGroupName ());
  group_item->appendRow (new_tool_item); 
  //Expand the view for this tool group
  QModelIndex group_index = this->indexFromItem(group_item);
  tool_view_->setExpanded (group_index, true);  
}

void
pcl::cloud_composer::ToolBoxModel::setSelectionModel (QItemSelectionModel* selection_model)
{
  selection_model_ = selection_model;
}
  
QStandardItem*
pcl::cloud_composer::ToolBoxModel::addToolGroup (const QString& tool_group_name)
{
  QList <QStandardItem*> matches_name = findItems (tool_group_name);
  if (matches_name.empty ())
  {
    QStandardItem* new_group_item = new QStandardItem (tool_group_name);
    appendRow (new_group_item);
    new_group_item->setSelectable (false);
    new_group_item->setEditable (false);

    return new_group_item;
  }
  if (matches_name.size () > 1)
  {
    qWarning () << "Multiple tool groups with same name in ToolBoxModel!!";
  }
  
  return matches_name.value (0);
  
}

void 
pcl::cloud_composer::ToolBoxModel::activeProjectChanged(ProjectModel* new_model, ProjectModel*)
{
  //Disconnect old project model signal for selection change
  if (project_model_)
  {
    disconnect (project_model_->getSelectionModel (), SIGNAL (selectionChanged (QItemSelection,QItemSelection)),
                this, SLOT (selectedItemChanged (QItemSelection,QItemSelection)));
    disconnect (project_model_, SIGNAL (modelChanged()),
                this, SLOT (modelChanged()));
    
  } 
  qDebug () << "Active project changed in ToolBox Model!";
  project_model_ = new_model;
  
  //Update enabled tools, make connection for doing this automatically
  if (project_model_)
  {  
    updateEnabledTools (project_model_->getSelectionModel ()->selection ());
    connect (project_model_->getSelectionModel (), SIGNAL (selectionChanged (QItemSelection,QItemSelection)),
                this, SLOT (selectedItemChanged (QItemSelection,QItemSelection)));
    connect (project_model_, SIGNAL (modelChanged()),
             this, SLOT (modelChanged()));
  }

}

void
pcl::cloud_composer::ToolBoxModel::selectedToolChanged (const QModelIndex & current, const QModelIndex &)
{
  //qDebug() << "Selected Tool changed";
  if (!parameter_view_)
  {
    qCritical () << "Toolbox parameter view not set!!!";
    return;
  }  
  QVariant parameter_model = current.data (PARAMETER_MODEL);
  parameter_view_->setModel ( parameter_model.value <PropertiesModel*> ());
  parameter_view_->expandAll ();
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
  PropertiesModel* parameter_model = (current_index.data (PARAMETER_MODEL)).value <PropertiesModel*> ();
  //
  AbstractTool* tool = tool_factory->createTool (parameter_model);
  
  emit enqueueToolAction (tool);
}

void 
pcl::cloud_composer::ToolBoxModel::selectedItemChanged ( const QItemSelection & selected, const QItemSelection &)
{
  updateEnabledTools (selected);
}

void 
pcl::cloud_composer::ToolBoxModel::enableAllTools ()
{
  foreach (QStandardItem* tool, tool_items)
  {
    tool->setEnabled (true);
  }
}

void
pcl::cloud_composer::ToolBoxModel::modelChanged ()
{
  updateEnabledTools (project_model_->getSelectionModel ()->selection ());  
}
void
pcl::cloud_composer::ToolBoxModel::updateEnabledTools (const QItemSelection& current_selection)
{
  //qDebug () << "UPDATING ENABLED TOOLS!";
  QModelIndexList current_indices = current_selection.indexes ();
  QMultiMap < int, QStandardItem* > type_items_map;
  foreach (QModelIndex current, current_indices)
  {
    if (current.isValid ())
    {
      QStandardItem* current_item = project_model_->itemFromIndex (current);
      type_items_map.insert (current_item->type (), current_item);
    }
  }
  enableAllTools ();
  QList <QStandardItem*> enabled_tools = tool_items.values (); 
  QMap <QStandardItem*,QString> disabled_tools;
  QMutableListIterator<QStandardItem*> enabled_itr(enabled_tools);
  //Go through tools, removing from enabled list if they fail to pass tests
  while (enabled_itr.hasNext()) 
  {
    QStandardItem* tool_item = enabled_itr.next ();
    ToolFactory* tool_factory = (tool_item->data (FACTORY)).value <ToolFactory*> ();
    CloudComposerItem::ItemType input_type = tool_factory->getInputItemType ();
    QList <CloudComposerItem::ItemType> required_children_types = tool_factory->getRequiredInputChildrenTypes();
    //Check if enough items for tool are selected
    if ( tool_factory-> getNumInputItems() > current_indices.size() )
    {
        enabled_itr.remove ();
        disabled_tools.insert (tool_item, tr("Tool Requires %1 Items (%2 Selected)").arg(tool_factory-> getNumInputItems()).arg(current_indices.size ()));
    }
    //Check if selection includes at least one item with correct input type
    else if ( ! type_items_map.keys ().contains (input_type))
    {
      enabled_itr.remove ();
      disabled_tools.insert (tool_item, tr("Tool Requires item type %1 selected").arg (ITEM_TYPES_STRINGS.value (input_type - CloudComposerItem::CLOUD_COMPOSER_ITEM)));
    }
    //Check if any of selected items have required children
    else if ( !required_children_types.empty ())
    {  
      QList <QStandardItem*> matching_selected_items = type_items_map.values (input_type);
      bool found_valid_items = false;
      QList <CloudComposerItem::ItemType> missing_children = required_children_types;
      foreach (QStandardItem* item, matching_selected_items)
      {
        QList <CloudComposerItem::ItemType> found_children_types;
        if (!item->hasChildren ())
          continue;
        
        //Find types of all children
        for (int i = 0; i < item->rowCount(); ++i)
          found_children_types.append ( static_cast<CloudComposerItem::ItemType>(item->child (i)->type ()));
        //Make temporary copy, remove type from it if is present as child
        QList <CloudComposerItem::ItemType> req_children_temp = required_children_types;
        foreach (CloudComposerItem::ItemType type, found_children_types)
          req_children_temp.removeAll (type);
        //If temporary is empty, we found all required children
        if (req_children_temp.isEmpty ())
        {
          found_valid_items = true;
          break;
        }
        //Otherwise, set missing children list
        if (req_children_temp.size () < missing_children.size ())
          missing_children = req_children_temp;


      }
      //If we didn't find all required children
      if (!found_valid_items)
      {
        enabled_itr.remove ();
        QString missing_children_string;
        foreach (CloudComposerItem::ItemType type, missing_children)
          missing_children_string.append (" "+ITEM_TYPES_STRINGS.value (type - CloudComposerItem::CLOUD_COMPOSER_ITEM));
        disabled_tools.insert (tool_item, tr ("Tool Requires child item of type(s) %1").arg (missing_children_string));
      }
    }
  }
  foreach (QStandardItem* tool, tool_items)
  {
    if (enabled_tools.contains (tool))
    {
      //qDebug () << tool->text() << " is enabled!";
      tool->setToolTip (tool->text() + " is enabled");
    }
    else
    {
     // qDebug () << tool->text() << " disabled: "<<disabled_tools.value (tool);
      tool->setToolTip (disabled_tools.value (tool));
      tool->setEnabled (false);
    }
  }
  
  
  
}
