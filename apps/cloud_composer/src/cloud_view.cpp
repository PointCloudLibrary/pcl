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
  QString project_name = model_->getName ();
  for (int row = start; row <= end; ++row)
  {
    
    QStandardItem* parent_item =  dynamic_cast<QStandardItemModel*>(model_)->itemFromIndex (parent);
    qDebug () << "Parent item = "<<parent_item;
    QStandardItem* new_item;
    if(parent_item == 0)
      new_item = model_->invisibleRootItem ()->child (row);
    else
      new_item = parent_item->child(row);
    CloudComposerItem* item;
    if (dynamic_cast<CloudComposerItem*> (new_item))
      item = dynamic_cast<CloudComposerItem*> (new_item);
    else
    {
      qCritical () << "Item for display in CloudView is not a CloudComposerItem!";
      continue;
    }
    item->paintView (vis_);
  }
  
  qvtk_->update ();

}

void
pcl::cloud_composer::CloudView::rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end)
{
  QString project_name = model_->getName ();
  QStandardItem* parent_item =  dynamic_cast<QStandardItemModel*>(model_)->itemFromIndex (parent);
  for (int row = start; row <= end; ++row)
  {
    QStandardItem* item_to_remove;
    if(parent_item == 0)
      item_to_remove = model_->invisibleRootItem ()->child (row);
    else
      item_to_remove = parent_item->child(row);
    CloudComposerItem* item;
    if (dynamic_cast<CloudComposerItem*> (item_to_remove))
      item = dynamic_cast<CloudComposerItem*> (item_to_remove);
    else
    {
      qCritical () << "Item for display in CloudView is not a CloudComposerItem!";
      continue;
    }
    switch (item->type ())
    {
      case NORMALS_ITEM:
      {  
        vis_->removePointCloud (item->getID ().toStdString ());
        break;
      }
      default:
        break;
    }
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
