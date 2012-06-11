#include <QtGui>
#include <QFileInfo>


#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_item.h>
#include <pcl/apps/cloud_composer/item_inspector.h>

/////////////////////////////////////////////////////////////
pcl::cloud_composer::ComposerMainWindow::ComposerMainWindow (QWidget *parent)
  : QMainWindow (parent)
{
  setupUi (this);

  this->setCorner (Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner (Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  this->connectFileActionsToSlots ();
  this->connectEditActionsToSlots ();
  
  //Register types in Qt
  qRegisterMetaType<sensor_msgs::PointCloud2::Ptr> ("PointCloud2Ptr");
  qRegisterMetaType<GeometryHandler::ConstPtr> ("GeometryHandlerConstPtr");
  qRegisterMetaType<ColorHandler::ConstPtr> ("ColorHandlerConstPtr");
  qRegisterMetaType<Eigen::Vector4f> ("EigenVector4f");
  qRegisterMetaType<Eigen::Quaternionf> ("EigenQuaternionf");
  qRegisterMetaType<ProjectModel> ("ProjectModel");
  qRegisterMetaType<CloudView> ("CloudView");
  
  //Auto connect signals and slots
 // QMetaObject::connectSlotsByName(this);
  
  last_directory_ = QDir (".");
  current_model_ = 0;
  
  initializeCloudBrowser();
  initializeCloudViewer();
  initializeItemInspector();
  
}

pcl::cloud_composer::ComposerMainWindow::~ComposerMainWindow ()
{
  foreach (ProjectModel* to_delete, name_model_map_.values ())
    to_delete->deleteLater();
}

void
pcl::cloud_composer::ComposerMainWindow::connectFileActionsToSlots ()
{

  
}

void
pcl::cloud_composer::ComposerMainWindow::connectEditActionsToSlots ()
{

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
  item_inspector_->setModel (current_model_);
  item_inspector_->setSelectionModel (current_model_->getSelectionModel ());

}


///////// FILE MENU SLOTS ///////////
void
pcl::cloud_composer::ComposerMainWindow::on_actionNewProject_triggered (QString name)
{
  qDebug () << "Creating New Project";
  ProjectModel* newProjectModel = new ProjectModel (this);
  // Check if we have a project with this name already, append int if so
  if (name_model_map_.contains (name))
  {
    int k = 2;
    while (name_model_map_.contains (name + tr ("-%1").arg (k)))
      ++k;
    name = name + tr ("-%1").arg (k);
  }
  
  newProjectModel->setName (name);
  name_model_map_.insert (name,newProjectModel);
  setCurrentModel (newProjectModel);
  
}


void
pcl::cloud_composer::ComposerMainWindow::on_actionOpenCloudAsNewProject_triggered ()
{
  qDebug () << "Opening cloud as new project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_actionOpenProject_triggered ()
{
  qDebug () << "Opening Project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_actionSaveProject_triggered ()
{
  qDebug () << "Saving Project";
}

void
pcl::cloud_composer::ComposerMainWindow::on_actionSaveProjectAs_triggered ()
{
  qDebug () << "Saving Project As...";
}

void
pcl::cloud_composer::ComposerMainWindow::on_actionExit_triggered ()
{
  qDebug () << "Exiting...";
}

///////// EDIT MENU SLOTS ////////////
void
pcl::cloud_composer::ComposerMainWindow::on_actionInsertFromFile_triggered ()
{
  qDebug () << "Inserting cloud from file...";
  QString filename = QFileDialog::getOpenFileName (0,tr ("Select cloud to open"), last_directory_.absolutePath (), tr ("PointCloud(*.pcd)"));
  if ( !filename.isNull ())
  {
    QFileInfo file_info (filename);
    last_directory_ = file_info.absoluteDir ();

    
    if (!current_model_)
      actionNewProject->trigger ();
    
    current_model_->insertNewCloudFromFile (filename);
    
  }
      
}

void
pcl::cloud_composer::ComposerMainWindow::on_actionInsertFromOpenNiSource_triggered ()
{
  qDebug () << "Inserting cloud from OpenNi Source...";
}





