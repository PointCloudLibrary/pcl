#include <QtGui>
#include <QFileInfo>


#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>

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
  connect (this->actionNewProject, SIGNAL (triggered ()),
           this, SLOT (slotNewProject ()));
  connect (this->actionOpenCloudAsNewProject, SIGNAL (triggered ()),
           this, SLOT (slotOpenCloudAsNewProject ()));
  connect (this->actionOpenProject, SIGNAL (triggered ()),
           this, SLOT (slotOpenProject ()));
  connect (this->actionSaveProject, SIGNAL (triggered ()),
           this, SLOT (slotSaveProject ()));
  connect (this->actionSaveProjectAs, SIGNAL (triggered ()),
           this, SLOT (slotSaveProjectAs ()));
  connect (this->actionExit, SIGNAL (triggered ()),
           this, SLOT (slotExit ()));
}

void
pcl::cloud_composer::ComposerMainWindow::connectEditActionsToSlots ()
{
  connect (this->actionInsertFromFile, SIGNAL (triggered ()),
           this, SLOT (slotInsertFromFile ()));
  connect (this->actionInsertFromOpenNiSource, SIGNAL (triggered ()),
           this, SLOT (slotInsertFromOpenNiSource ()));
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

}


///////// FILE MENU SLOTS ///////////
void
pcl::cloud_composer::ComposerMainWindow::slotNewProject (QString name)
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
pcl::cloud_composer::ComposerMainWindow::slotOpenCloudAsNewProject ()
{

}

void
pcl::cloud_composer::ComposerMainWindow::slotOpenProject ()
{

}

void
pcl::cloud_composer::ComposerMainWindow::slotSaveProject ()
{

}

void
pcl::cloud_composer::ComposerMainWindow::slotSaveProjectAs ()
{

}

void
pcl::cloud_composer::ComposerMainWindow::slotExit ()
{

}

///////// EDIT MENU SLOTS ////////////
void
pcl::cloud_composer::ComposerMainWindow::slotInsertFromFile ()
{
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
pcl::cloud_composer::ComposerMainWindow::slotInsertFromOpenNiSource ()
{

}





