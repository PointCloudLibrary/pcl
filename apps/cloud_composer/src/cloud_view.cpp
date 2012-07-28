#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::CloudView::CloudView (QWidget* parent)
  : QWidget (parent)
{
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  
}

pcl::cloud_composer::CloudView::CloudView (ProjectModel* model, QWidget* parent)
  : QWidget (parent)
{
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  setModel(model);
  
}

pcl::cloud_composer::CloudView::CloudView (const CloudView& to_copy)
  : QWidget ()
  , vis_ (to_copy.vis_)
  , model_ (to_copy.model_)
  , qvtk_ (to_copy.qvtk_)
{
}


pcl::cloud_composer::CloudView::~CloudView ()
{
}

void
pcl::cloud_composer::CloudView::setModel (ProjectModel* new_model)
{
  //Create the QVTKWidget
  qvtk_ = new QVTKWidget ();
  qvtk_->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (qvtk_->GetInteractor (), qvtk_->GetRenderWindow ());
 
  QGridLayout *mainLayout = new QGridLayout (this);
  mainLayout-> addWidget (qvtk_,0,0);
  
  model_ = new_model;
  //Make Connections!
  connectSignalsAndSlots();
  //Refresh the view
  qvtk_->show();
  qvtk_->update ();
}

void
pcl::cloud_composer::CloudView::connectSignalsAndSlots()
{
  connect (model_, SIGNAL (itemChanged (QStandardItem*)),
           this, SLOT (itemChanged (QStandardItem*)));
  connect (model_, SIGNAL (rowsInserted (const QModelIndex&, int, int)),
           this, SLOT (rowsInserted (const QModelIndex&, int, int)));
  connect (model_, SIGNAL (rowsAboutToBeRemoved (const QModelIndex, int, int)),
           this, SLOT (rowsAboutToBeRemoved(const QModelIndex,int,int)));
}

void
pcl::cloud_composer::CloudView::refresh ()
{
  qvtk_->update (); 
}

void
pcl::cloud_composer::CloudView::itemChanged (QStandardItem* changed_item)
{
  qDebug () << "Item Changed - Redrawing!";
  CloudComposerItem* item;
  if (dynamic_cast<CloudComposerItem*> (changed_item))
  {
    item = dynamic_cast<CloudComposerItem*> (changed_item);
    item->paintView (vis_);
  }
  qvtk_->update ();
}



void
pcl::cloud_composer::CloudView::rowsInserted (const QModelIndex& parent, int start, int end)
{
  QStandardItem* parent_item;
  //If the parent is invisibleRootItem this will be true (parent is not valid)
  if (!parent.isValid())
    parent_item = model_->invisibleRootItem();
  else
    parent_item = model_->itemFromIndex (parent);
  QString project_name = model_->getName ();
  for (int row = start; row <= end; ++row)
  {
    QStandardItem* new_item = parent_item->child(row);
    CloudComposerItem* item = dynamic_cast<CloudComposerItem*> (new_item);
      item = dynamic_cast<CloudComposerItem*> (new_item);
    if (item)
      item->paintView (vis_);
    
    //Recursive call, need to paint all children as well
  //  qDebug () << "Making recursive call, start =0, end="<<new_item->rowCount ()-1;
    if (new_item->rowCount () > 0)  
      rowsInserted(new_item->index(),0,new_item->rowCount ()-1);
  }
  
  qvtk_->update ();

}

void
pcl::cloud_composer::CloudView::rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end)
{
  QStandardItem* parent_item;
  //If the parent is invisibleRootItem this will be true (parent is not valid)
  if (!parent.isValid())
    parent_item = model_->invisibleRootItem();
  else
    parent_item = model_->itemFromIndex (parent);
  QString project_name = model_->getName ();
  qDebug () << "Rows about to be removed, parent = "<<parent_item->text ()<<" start="<<start<<" end="<<end;
  for (int row = start; row <= end; ++row)
  {
    QStandardItem* item_to_remove = parent_item->child(row);
    if (item_to_remove)
      qDebug () << "Removing "<<item_to_remove->text ();
    CloudComposerItem* item = dynamic_cast<CloudComposerItem*> (item_to_remove);
    if (item )
      item->removeFromView (vis_);
    
    //Recursive call, need to remove all children as well
  //  qDebug () << "Making recursive call, start =0, end="<<item_to_remove->rowCount ()-1;
    if (item_to_remove->rowCount () > 0) 
      rowsAboutToBeRemoved(item_to_remove->index(),0,item_to_remove->rowCount ()-1);
  }
  qvtk_->update ();
}


void 
pcl::cloud_composer::CloudView::paintEvent (QPaintEvent*)
{
  qvtk_->update ();
}

void 
pcl::cloud_composer::CloudView::resizeEvent (QResizeEvent*)
{
  qvtk_->update ();
}

void
pcl::cloud_composer::CloudView::selectedItemChanged (const QItemSelection & selected, const QItemSelection & deselected)
{
  QModelIndexList current_indices = selected.indexes ();
  QModelIndexList previous_indices = deselected.indexes ();
  foreach (QModelIndex previous, previous_indices)
  {
    if (previous.isValid ())
    {
      QStandardItem* previous_item = model_->itemFromIndex (previous);
      if (previous_item->type () == CLOUD_ITEM || previous_item->type () == NORMALS_ITEM)
      {
        CloudComposerItem* cc_item = dynamic_cast <CloudComposerItem*> (previous_item);
        vis_->setPointCloudSelected (false, cc_item->getID ().toStdString ());
      }
    }
  }
  foreach (QModelIndex current, current_indices)
  {
    if (current.isValid ())
    {
      QStandardItem* current_item = model_->itemFromIndex (current);
      if (current_item->type () == CLOUD_ITEM || current_item->type () == NORMALS_ITEM)
      {
        CloudComposerItem* cc_item = dynamic_cast <CloudComposerItem*> (current_item);
        qDebug () << "Setting point cloud to selected";
        vis_->setPointCloudSelected (true, cc_item->getID ().toStdString ());
      }
    }
  }
  qvtk_->update ();
}
