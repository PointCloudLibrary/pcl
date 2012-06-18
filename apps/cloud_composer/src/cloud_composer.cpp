#include <QtGui>
#include <QFileInfo>


#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_item.h>
#include <pcl/apps/cloud_composer/item_inspector.h>
#include <pcl/apps/cloud_composer/commands.h>

/////////////////////////////////////////////////////////////
pcl::cloud_composer::ComposerMainWindow::ComposerMainWindow (QWidget *parent)
  : QMainWindow (parent)
{
  setupUi (this);

  this->setCorner (Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner (Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  //Register types in Qt
  qRegisterMetaType<sensor_msgs::PointCloud2::Ptr> ("PointCloud2Ptr");
  qRegisterMetaType<GeometryHandler::ConstPtr> ("GeometryHandlerConstPtr");
  qRegisterMetaType<ColorHandler::ConstPtr> ("ColorHandlerConstPtr");
  qRegisterMetaType<Eigen::Vector4f> ("EigenVector4f");
  qRegisterMetaType<Eigen::Quaternionf> ("EigenQuaternionf");
  qRegisterMetaType<ProjectModel> ("ProjectModel");
  qRegisterMetaType<CloudView> ("CloudView");
  

  last_directory_ = QDir (".");
  current_model_ = 0;
  
  initializeCloudBrowser();
  initializeCloudViewer();
  initializeItemInspector();
  
  undo_group_ = new QUndoGroup (this);
  undo_view_->setGroup (undo_group_);
  
  //Auto connect signals and slots
  // QMetaObject::connectSlotsByName(this);
  this->connectFileActionsToSlots ();
  this->connectEditActionsToSlots ();
}

pcl::cloud_composer::ComposerMainWindow::~ComposerMainWindow ()
{
  foreach (ProjectModel* to_delete, name_model_map_.values ())
    to_delete->deleteLater ();
}

void
pcl::cloud_composer::ComposerMainWindow::connectFileActionsToSlots ()
{

  
}

void
pcl::cloud_composer::ComposerMainWindow::connectEditActionsToSlots ()
{
  //Replace the actions in the menu with undo actions created using the undo group
  QAction* action_temp = undo_group_->createUndoAction (this);
  action_temp->setShortcut (action_undo_->shortcut ());
  menuEdit->insertAction (action_redo_, action_temp);
  menuEdit->removeAction (action_undo_);
  action_undo_ = action_temp;
  
  action_temp = undo_group_->createRedoAction (this);
  action_temp->setShortcut (action_redo_->shortcut ());
  menuEdit->insertAction (action_redo_, action_temp);
  menuEdit->removeAction (action_redo_);
  action_redo_ = action_temp;
  

}

void
pcl::cloud_composer::ComposerMainWindow::initializeCloudBrowser ()
{
  
}

void
pcl::cloud_composer::ComposerMainWindow::initializeCloudViewer ()
{
  //Signal emitted when user selects new tab (ie different project) in the viewer
  connect (cloud_viewer_, SIGNAL (newModelSelected (ProjectModel*)),
           this, SLOT (setCurrentModel (ProjectModel*)));
}

void
pcl::cloud_composer::ComposerMainWindow::initializeItemInspector ()
{
  
}


void 
pcl::cloud_composer::ComposerMainWindow::setCurrentModel (ProjectModel* model)
{
  current_model_ = model;
  cloud_browser_->setModel (current_model_);
  cloud_browser_->setSelectionModel (current_model_->getSelectionModel ());
  cloud_viewer_->setModel (current_model_);
  item_inspector_->setProjectAndSelectionModels (current_model_, current_model_->getSelectionModel ());
  undo_group_->setActiveStack (current_model_->getUndoStack ());
}


///////// FILE MENU SLOTS ///////////
void
pcl::cloud_composer::ComposerMainWindow::on_action_new_project__triggered (QString name)
{
  qDebug () << "Creating New Project";
  ProjectModel* new_project_model = new ProjectModel (this);
  // Check if we have a project with this name already, append int if so
  if (name_model_map_.contains (name))
  {
    int k = 2;
    while (name_model_map_.contains (name + tr ("-%1").arg (k)))
      ++k;
    name = name + tr ("-%1").arg (k);
  }
  
  new_project_model->setName (name);
  name_model_map_.insert (name,new_project_model);
  undo_group_->addStack (new_project_model->getUndoStack ());
  setCurrentModel (new_project_model);
  
}


void
pcl::cloud_composer::ComposerMainWindow::on_action_open_cloud_as_new_project__triggered ()
{
  qDebug () << "Opening cloud as new project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_open_project__triggered ()
{
  qDebug () << "Opening Project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_save_project__triggered ()
{
  qDebug () << "Saving Project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_save_project_as__triggered ()
{
  qDebug () << "Saving Project As...";
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_exit__triggered ()
{
  qDebug () << "Exiting...";
}

///////// EDIT MENU SLOTS ////////////
void
pcl::cloud_composer::ComposerMainWindow::on_action_insert_from_file__triggered ()
{
  qDebug () << "Inserting cloud from file...";
  QString filename = QFileDialog::getOpenFileName (0,tr ("Select cloud to open"), last_directory_.absolutePath (), tr ("PointCloud(*.pcd)"));
  if ( !filename.isNull ())
  {
    QFileInfo file_info (filename);
    last_directory_ = file_info.absoluteDir ();

    
    if (!current_model_)
      action_new_project_->trigger ();
    
    current_model_->insertNewCloudFromFile (filename);
    
  }
      
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_insert_from_openNi_source__triggered ()
{
  qDebug () << "Inserting cloud from OpenNi Source...";
}





