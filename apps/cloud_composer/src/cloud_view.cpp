#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>

#include <pcl/apps/cloud_composer/point_selectors/selection_event.h>
#include <pcl/apps/cloud_composer/point_selectors/manipulation_event.h>

#include <vtkGenericOpenGLRenderWindow.h>

#include <QDebug>


pcl::cloud_composer::CloudView::CloudView (QWidget* parent)
  : QWidget (parent)
{
  qvtk_ = new PCLQVTKWidget(this);
  //Create the QVTKWidget
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*qvtk_, *(vis_->getRenderWindow()));
  vis_->setupInteractor(getInteractorCompat(*qvtk_), getRenderWindowCompat(*qvtk_), style_switch_);
  vis_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  
  initializeInteractorSwitch ();
  
  QGridLayout *mainLayout = new QGridLayout (this);
  mainLayout-> addWidget (qvtk_,0,0);
}

pcl::cloud_composer::CloudView::CloudView (ProjectModel* model, QWidget* parent)
  : QWidget (parent)
{
  model_ = model;
  
  qvtk_ = new PCLQVTKWidget(this);
  //Create the QVTKWidget
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*qvtk_, *(vis_->getRenderWindow()));
  vis_->setupInteractor(getInteractorCompat(*qvtk_), getRenderWindowCompat(*qvtk_), style_switch_);
  //vis_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

  initializeInteractorSwitch ();
  setModel(model);
  
  QGridLayout *mainLayout = new QGridLayout (this);
  mainLayout-> addWidget (qvtk_,0,0);
}

pcl::cloud_composer::CloudView::CloudView (const CloudView& to_copy)
  : QWidget()
  , vis_ (to_copy.vis_)
  , model_ (to_copy.model_)
  , qvtk_ (to_copy.qvtk_)
{
}

void
pcl::cloud_composer::CloudView::setModel (ProjectModel* new_model)
{
  model_ = new_model;
  //Make Connections!
  connectSignalsAndSlots();
  //Refresh the view
  qvtk_->show();
  refresh();
  
 // vis_->addOrientationMarkerWidgetAxes (getInteractorCompat(qvtk_));
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
#if VTK_MAJOR_VERSION > 8
  qvtk_->renderWindow()->Render();
#else
  qvtk_->update (); 
#endif // VTK_MAJOR_VERSION > 8
}

void
pcl::cloud_composer::CloudView::itemChanged (QStandardItem* changed_item)
{
  qDebug () << "Item Changed - Redrawing!";
  CloudComposerItem* item = dynamic_cast<CloudComposerItem*> (changed_item);
  if (item)
  {
    item->paintView (vis_);
  }
  refresh();
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
  for (int row = start; row <= end; ++row)
  {
    QStandardItem* new_item = parent_item->child(row);
    CloudComposerItem* item = dynamic_cast<CloudComposerItem*> (new_item);
    if (item)
      item->paintView (vis_);
    
    //Recursive call, need to paint all children as well
  //  qDebug () << "Making recursive call, start =0, end="<<new_item->rowCount ()-1;
    if (new_item->rowCount () > 0)  
      rowsInserted(new_item->index(),0,new_item->rowCount ()-1);
  }
  
  refresh();
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
  //qDebug () << "Rows about to be removed, parent = "<<parent_item->text ()<<" start="<<start<<" end="<<end;
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
  refresh();
}


void 
pcl::cloud_composer::CloudView::paintEvent (QPaintEvent*)
{
  refresh();
}

void 
pcl::cloud_composer::CloudView::resizeEvent (QResizeEvent*)
{
  refresh();
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
      if (previous_item->type () == CloudComposerItem::CLOUD_ITEM || previous_item->type () == CloudComposerItem::NORMALS_ITEM)
      {
        vis_->setPointCloudSelected (false, previous_item->data (ItemDataRole::ITEM_ID).toString ().toStdString ());
      }
    }
  }
  foreach (QModelIndex current, current_indices)
  {
    if (current.isValid ())
    {
      QStandardItem* current_item = model_->itemFromIndex (current);
      if (current_item->type () == CloudComposerItem::CLOUD_ITEM || current_item->type () == CloudComposerItem::NORMALS_ITEM)
      {
        vis_->setPointCloudSelected (true, current_item->data (ItemDataRole::ITEM_ID).toString ().toStdString ());
      }
    }
  }
  refresh();
}

void
pcl::cloud_composer::CloudView::dataChanged (const QModelIndex &, const QModelIndex &)
{
    
  
}

////// Axis Functions
////////////////////////////////////////////////////////////////////
void
pcl::cloud_composer::CloudView::setAxisVisibility (bool visible)
{
  if (visible)
  {
    qDebug () << "Adding coordinate system!";
    vis_->addOrientationMarkerWidgetAxes(getInteractorCompat(*qvtk_));
  }
  else
  {
    vis_->removeOrientationMarkerWidgetAxes ();
  }

  refresh();
}

void
pcl::cloud_composer::CloudView::addOrientationMarkerWidgetAxes ()
{
  if (!axes_widget_)
  {
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New ();
   
    axes_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New ();
    axes_widget_->SetOutlineColor ( 0.9300, 0.5700, 0.1300 );
    axes_widget_->SetOrientationMarker( axes );
    axes_widget_->SetInteractor(getInteractorCompat(*qvtk_));
    axes_widget_->SetViewport( 0.0, 0.0, 0.4, 0.4 );
    axes_widget_->SetEnabled( 1 );
    axes_widget_->InteractiveOn();
  }
  else
  {
    axes_widget_->SetEnabled (true);
  }

}


void
pcl::cloud_composer::CloudView::removeOrientationMarkerWidgetAxes ()
{
  if (axes_widget_)
  {
    axes_widget_->SetEnabled (false);
  }
}

////////  Interactor Functions
/////////////////////////////////////////////////////////////////////////////
void
pcl::cloud_composer::CloudView::initializeInteractorSwitch ()
{
  style_switch_ = vtkSmartPointer<InteractorStyleSwitch>::New();
  style_switch_->initializeInteractorStyles (vis_, model_);
  style_switch_->SetInteractor(getInteractorCompat(*qvtk_));
  style_switch_->setCurrentInteractorStyle (interactor_styles::PCL_VISUALIZER);
  
  //Connect the events!
  connections_ = vtkSmartPointer<vtkEventQtSlotConnect>::New();
  connections_->Connect (style_switch_->getInteractorStyle (interactor_styles::RECTANGULAR_FRUSTUM),
                         interactor_events::SELECTION_COMPLETE_EVENT,
                         this,
                         SLOT (selectionCompleted (vtkObject*, unsigned long, void*, void*)));
 
  connections_->Connect (style_switch_->getInteractorStyle (interactor_styles::CLICK_TRACKBALL),
                         interactor_events::MANIPULATION_COMPLETE_EVENT,
                         this,
                         SLOT (manipulationCompleted (vtkObject*, unsigned long, void*, void*)));
}

void
pcl::cloud_composer::CloudView::setInteractorStyle (interactor_styles::INTERACTOR_STYLES style)
{
  style_switch_->setCurrentInteractorStyle (style);
}

void
pcl::cloud_composer::CloudView::selectionCompleted (vtkObject*, unsigned long, void*, void* call_data)
{
  std::shared_ptr<SelectionEvent> selected (static_cast<SelectionEvent*> (call_data));
  
  if (selected)
  {
    qDebug () << "Selection Complete! - Num points="<<selected->getNumPoints();
    model_->setPointSelection (selected);
    style_switch_->setCurrentInteractorStyle (interactor_styles::PCL_VISUALIZER);
  }
}


void
pcl::cloud_composer::CloudView::manipulationCompleted (vtkObject*, unsigned long, void*, void* call_data)
{
  std::shared_ptr<ManipulationEvent> manip_event (static_cast<ManipulationEvent*> (call_data));
  
  if (manip_event)
  {
    qDebug () << "Manipulation event received in cloud view!";
    model_->manipulateClouds (manip_event);   
  }
}

