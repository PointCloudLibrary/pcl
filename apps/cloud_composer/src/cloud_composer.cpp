#include <QtGui>
#include <QFileInfo>


#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>

/////////////////////////////////////////////////////////////
pcl::cloud_composer::ComposerMainWindow::ComposerMainWindow (QWidget *parent)
  : QMainWindow (parent)
  , ui_ (new Ui::MainWindow)
{
  ui_->setupUi (this);

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
  
  //Cloud Viewer Connection
  connect (ui_->cloud_viewer_, SIGNAL (newModelSelected (ProjectModel*)),
           this, SLOT (setCurrentModel (ProjectModel*)));
  current_model_ = 0;
}

pcl::cloud_composer::ComposerMainWindow::~ComposerMainWindow ()
{
  delete ui_;
}

void
pcl::cloud_composer::ComposerMainWindow::connectFileActionsToSlots ()
{
  connect (this->ui_->actionNewProject, SIGNAL (triggered ()),
           this, SLOT (slotNewProject ()));
  connect (this->ui_->actionOpenCloudAsNewProject, SIGNAL (triggered ()),
           this, SLOT (slotOpenCloudAsNewProject ()));
  connect (this->ui_->actionOpenProject, SIGNAL (triggered ()),
           this, SLOT (slotOpenProject ()));
  connect (this->ui_->actionSaveProject, SIGNAL (triggered ()),
           this, SLOT (slotSaveProject ()));
  connect (this->ui_->actionSaveProjectAs, SIGNAL (triggered ()),
           this, SLOT (slotSaveProjectAs ()));
  connect (this->ui_->actionExit, SIGNAL (triggered ()),
           this, SLOT (slotExit ()));
}

void
pcl::cloud_composer::ComposerMainWindow::connectEditActionsToSlots ()
{
  connect (this->ui_->actionInsertFromFile, SIGNAL (triggered ()),
           this, SLOT (slotInsertFromFile ()));
  connect (this->ui_->actionInsertFromOpenNiSource, SIGNAL (triggered ()),
           this, SLOT (slotInsertFromOpenNiSource ()));
}

void
pcl::cloud_composer::ComposerMainWindow::initializeCloudBrowser ()
{
  
}


void 
pcl::cloud_composer::ComposerMainWindow::setCurrentModel (ProjectModel* model)
{
  current_model_ = model;
  ui_->cloud_browser_->setModel (current_model_);
  ui_->cloud_viewer_->setModel (current_model_);
  
}


QStandardItem* 
pcl::cloud_composer::ComposerMainWindow::createNewCloudItem (sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                                                             QString cloud_name, 
                                                             Eigen::Vector4f origin, 
                                                             Eigen::Quaternionf orientation)
{
  QStandardItem* new_item = new QStandardItem (cloud_name);
  QVariant new_cloud = QVariant::fromValue (cloud_ptr);
  new_item->setData (new_cloud, CLOUD);
  
  //Create a color and geometry handler for this cloud
  ColorHandler::ConstPtr color_handler;
  color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_ptr));
 // QVariant new_color_handler = QVariant::fromValue (color_handler);
  new_item->setData (QVariant::fromValue (color_handler), COLOR);
    
  GeometryHandler::ConstPtr geometry_handler;
  geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> (cloud_ptr));
  //QVariant new_geometry_handler = ;
  new_item->setData (QVariant::fromValue (geometry_handler), GEOMETRY);
  
  new_item->setData (QVariant::fromValue (origin), ORIGIN);
  new_item->setData (QVariant::fromValue (orientation), ORIENTATION);
  
  return new_item;
}

///////// FILE MENU SLOTS ///////////
void
pcl::cloud_composer::ComposerMainWindow::slotNewProject ()
{
  qDebug () << "Creating New Project";
  ProjectModel* newProjectModel = new ProjectModel (this);
  newProjectModel->setHorizontalHeaderItem ( 0, new QStandardItem ( "unnamed project" ));
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
    QString short_filename = file_info.baseName ();
    
    sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;

    pcl::PCDReader pcd;
    if (pcd.read (filename.toStdString (), *cloud_blob, origin, orientation, version) < 0)
    {
      qDebug () << "Failed to read cloud from file";
      return;
    }
    if (cloud_blob->width * cloud_blob->height == 0)
    {
      qDebug () << "Cloud read has zero size!";
      return;
    }
    
    QStandardItem* new_item = createNewCloudItem (cloud_blob, short_filename, origin, orientation);
    
    if (current_model_)
      current_model_->appendRow (new_item);
    else
    {
      ui_->actionNewProject->trigger ();
      current_model_->appendRow (new_item);
    }
  }
      
}

void
pcl::cloud_composer::ComposerMainWindow::slotInsertFromOpenNiSource ()
{

}





