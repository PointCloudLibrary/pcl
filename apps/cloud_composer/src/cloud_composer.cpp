#include <QtGui>
#include <QFileInfo>


#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_item.h>
#include <pcl/apps/cloud_composer/item_inspector.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/toolbox_model.h>

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
  qRegisterMetaType<ConstItemList> ("ConstComposerItemList");
  

  last_directory_ = QDir (".");
  current_model_ = 0;
  
  initializeCloudBrowser ();
  initializeCloudViewer ();
  initializeItemInspector ();
  initializeToolBox ();
  initializePlugins ();
  
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
pcl::cloud_composer::ComposerMainWindow::initializeToolBox ()
{
  tool_box_model_ = new ToolBoxModel (tool_parameter_view_,this);
  tool_selection_model_ = new QItemSelectionModel (tool_box_model_);
  tool_box_model_->setSelectionModel (tool_selection_model_);
  
  tool_box_view_->setModel (tool_box_model_);
  tool_box_view_->setSelectionModel (tool_selection_model_);
  tool_box_view_->setIconSize (QSize (32,32));
  tool_box_view_->setIndentation (10);
  
  connect ( tool_selection_model_, SIGNAL (currentChanged (const QModelIndex&, const QModelIndex&)),
            tool_box_model_, SLOT (selectedToolChanged (const QModelIndex&, const QModelIndex&)));
  
  connect ( tool_box_model_, SIGNAL (enqueueToolAction (AbstractTool*)),
            this, SLOT (enqueueToolAction (AbstractTool*)));
  
  //TODO : Remove this, tools should have a better way of being run
  connect ( action_run_tool_, SIGNAL (clicked ()),
            tool_box_model_, SLOT (toolAction ()));
  //tool_box_view_->setStyleSheet("branch:has-siblings:!adjoins-item:image none");
 // tool_box_view_->setStyleSheet("branch:!has-children:!has-siblings:adjoins-item:image: none");
  
  
}


void 
pcl::cloud_composer::ComposerMainWindow::initializePlugins ()
{
  QDir plugin_dir = QCoreApplication::applicationDirPath ();
  qDebug() << plugin_dir.path ()<< "   "<<QDir::cleanPath ("../lib/cloud_composer_plugins");
  if (!plugin_dir.cd (QDir::cleanPath ("../lib/cloud_composer_plugins")))
  {
    qCritical () << "Could not find plugin tool directory!!!";
  }
  QStringList plugin_filter;
  plugin_filter << "libpcl_cc_tool_*.so";
  plugin_dir.setNameFilters (plugin_filter);
  foreach (QString filename, plugin_dir.entryList (QDir::Files))
  {
    qDebug () << "Loading " << plugin_dir.relativeFilePath (filename);
    QPluginLoader loader (plugin_dir.absoluteFilePath (filename), this);
    // This is automatically deleted when the library is unloaded (on app exit)
    QObject *plugin = loader.instance ();
    ToolFactory* tool_factory = qobject_cast <ToolFactory*> (plugin);
    if (tool_factory) {
      qWarning () << "Loaded " << tool_factory->getPluginName ();
      //Create the action button for this tool
      tool_box_model_->addTool (tool_factory);
      
     /*
      processFactoryMap.insert(processFactory->processName(),processFactory);
      pluginFileNames.append(fileName);
      //Check what types the plugin uses
      QList<QString> usedTypes = processFactory->processTypes();
      TypeCreatorMap creatorMap = processFactory->getTypeCreatorMap();
      foreach(QString type, usedTypes){
        if(!dataTypes.contains(type)){
          if(creatorMap.contains(type)){
            qWarning() << "New type " << type<< " encountered in plugin "<<processFactory->processName();
            typeCreatorMap.insert(type,creatorMap.value(type));
            dataTypes.insert(type);
            //Create a temporary instance of the new datacontainer type to check if it is displayable
            AbstractDataContainer* tempContainer;
            //Get the function pointer
            AbstractDataContainer* (*creatorFunction)() = static_cast< AbstractDataContainer* (*)() > (creatorMap.value(type));
            //Create the new data container
            tempContainer = creatorFunction();
            //Check if this new type is displayble
            if(tempContainer->isDisplayable())
              displayableDataTypes.insert(type);
            delete tempContainer;
          }else{
            qCritical() << "New Type "<< type<< " encountered in plugin " <<processFactory->processName()<<" but NO creator function!!";
          }
        }
        
      }*/
    }
    else{
      qDebug() << "Could not load " << plugin_dir.relativeFilePath (filename);
      qDebug() << loader.errorString ();
    }
    
  }
}



void 
pcl::cloud_composer::ComposerMainWindow::setCurrentModel (ProjectModel* model)
{
  current_model_ = model;
  //qDebug () << "Setting cloud browser model";
  cloud_browser_->setModel (current_model_);
  //qDebug () << "Setting cloud browser selection model";
  cloud_browser_->setSelectionModel (current_model_->getSelectionModel ());
  //qDebug () << "Setting cloud viewer model";
  cloud_viewer_->setModel (current_model_);
  //qDebug () << "Item inspector setting project and selection models";
  item_inspector_->setProjectAndSelectionModels (current_model_, current_model_->getSelectionModel ());
  //qDebug () << "Setting active stack in undo group";
  undo_group_->setActiveStack (current_model_->getUndoStack ());
}

void
pcl::cloud_composer::ComposerMainWindow::enqueueToolAction (AbstractTool* tool)
{
  current_model_->enqueueToolAction (tool);
  
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
  //qDebug () << "Setting name";
  new_project_model->setName (name);
  //qDebug () << "Inserting into map";
  name_model_map_.insert (name,new_project_model);
  //qDebug () << "Adding to undo group";
  undo_group_->addStack (new_project_model->getUndoStack ());
  //qDebug () << "Setting current model";
  setCurrentModel (new_project_model);
  //qDebug () << "Project " <<name<<" created!";
  
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





