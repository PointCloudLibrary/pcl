#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_view.h>

pcl::cloud_composer::CloudViewer::CloudViewer (QWidget* parent)
  : QTabWidget (parent)

{
  connect (this, SIGNAL (currentChanged (int)),
           this, SLOT (modelChanged (int)));
}

pcl::cloud_composer::CloudViewer::~CloudViewer ()
{
  
}

void
pcl::cloud_composer::CloudViewer::addModel (ProjectModel* new_model)
{
  CloudView* new_view = new CloudView (new_model);
  connect (new_model->getSelectionModel (), SIGNAL (selectionChanged (QItemSelection,QItemSelection)),
           new_view, SLOT (selectedItemChanged (QItemSelection,QItemSelection)));
  new_model->setCloudView (new_view);
  
  QStandardItem* title = new_model->horizontalHeaderItem (0);
  this->addTab (new_view, title->text ());
  
  model_view_map_.insert (new_model,new_view);
  
  setCurrentWidget (model_view_map_.value (new_model));
  //Refresh the view
  new_view->refresh ();

}
  
pcl::cloud_composer::ProjectModel*
pcl::cloud_composer::CloudViewer::getModel () const
{
  if (this->count() == 0)
    return 0;
  else
    return dynamic_cast<CloudView*> (currentWidget ())->getModel (); 
}

void
pcl::cloud_composer::CloudViewer::addNewProject (ProjectModel* new_model)
{
  //If we're already there, abort
  if (new_model == getModel ())
    return;
  //Check whether we've seen the model yet
  if ( !model_view_map_.contains (new_model))
  {
    addModel (new_model);
  }
  else
  {
    setCurrentWidget (model_view_map_.value (new_model));
    //Refresh the view
    model_view_map_.value (new_model)->refresh ();
  }
}

void
pcl::cloud_composer::CloudViewer::modelChanged (int)
{
  emit newModelSelected (getModel ());
}

