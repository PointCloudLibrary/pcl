#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/item_inspector.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/toolbox_model.h>
#include <pcl/apps/cloud_composer/signal_multiplexer.h>
#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>

/////////////////////////////////////////////////////////////
pcl::cloud_composer::ComposerMainWindow::ComposerMainWindow (QWidget *parent)
  : QMainWindow (parent)
{
  setupUi (this);

  this->setCorner (Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner (Qt::BottomRightCorner, Qt::RightDockWidgetArea);

   //Register types in Qt
  qRegisterMetaType<pcl::PCLPointCloud2::Ptr> ("PCLPointCloud2Ptr");
  qRegisterMetaType<GeometryHandler::ConstPtr> ("GeometryHandlerConstPtr");
  qRegisterMetaType<ColorHandler::ConstPtr> ("ColorHandlerConstPtr");
  qRegisterMetaType<Eigen::Vector4f> ("EigenVector4f");
  qRegisterMetaType<Eigen::Quaternionf> ("EigenQuaternionf");
  qRegisterMetaType<ProjectModel> ("ProjectModel");
  qRegisterMetaType<CloudView> ("CloudView");
  qRegisterMetaType<ConstItemList> ("ConstComposerItemList");
  
  current_model_ = 0;
  
  multiplexer_ = new SignalMultiplexer (this);
  
  initializeCloudBrowser ();
  initializeCloudViewer ();
  initializeItemInspector ();
  initializeToolBox ();
  initializePlugins ();
  
  undo_group_ = new QUndoGroup (this);
  undo_view_->setGroup (undo_group_);
  
  //Auto connect signals and slots
  // QMetaObject::connectSlotsByName(this);
  connectFileActions ();
  connectEditActions ();
  connectViewActions ();
  
}

pcl::cloud_composer::ComposerMainWindow::~ComposerMainWindow ()
{
  foreach (ProjectModel* to_delete, name_model_map_.values ())
    to_delete->deleteLater ();
}

void
pcl::cloud_composer::ComposerMainWindow::connectFileActions ()
{

  
}

void
pcl::cloud_composer::ComposerMainWindow::connectEditActions ()
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
  
  multiplexer_->connect (action_clear_selection_, SIGNAL (triggered ()), SLOT (clearSelection ()));
  
  multiplexer_->connect (action_delete_, SIGNAL (triggered ()), SLOT (deleteSelectedItems ()));
  multiplexer_->connect (SIGNAL (deleteAvailable (bool)), action_delete_, SLOT (setEnabled (bool)));
  
  multiplexer_->connect (this, SIGNAL (insertNewCloudFromFile()), SLOT (insertNewCloudFromFile()));
  multiplexer_->connect (this, SIGNAL (insertNewCloudFromRGBandDepth()), SLOT (insertNewCloudFromRGBandDepth()));
  multiplexer_->connect (this, SIGNAL (saveSelectedCloudToFile()), SLOT (saveSelectedCloudToFile()));
  
  
  mouse_style_group_ = new QActionGroup (this);
  mouse_style_group_->addAction (action_trackball_camera_style_);
  mouse_style_group_->addAction (action_manipulate_clicked_);
  mouse_style_group_->addAction (action_manipulate_selected_);
  mouse_style_group_->addAction (action_rectangular_frustum_select_);
  mouse_style_group_->setExclusive (true);
  multiplexer_->connect (mouse_style_group_, SIGNAL (triggered (QAction*)), SLOT (mouseStyleChanged (QAction*)));
  multiplexer_->connect(SIGNAL (mouseStyleState (interactor_styles::INTERACTOR_STYLES)), this, SLOT(setMouseStyleAction(interactor_styles::INTERACTOR_STYLES)));
  action_trackball_camera_style_->setData (QVariant::fromValue (interactor_styles::PCL_VISUALIZER));
  action_rectangular_frustum_select_->setData (QVariant::fromValue (interactor_styles::RECTANGULAR_FRUSTUM));
  action_manipulate_selected_->setData (QVariant::fromValue (interactor_styles::SELECTED_TRACKBALL));
  action_manipulate_clicked_->setData (QVariant::fromValue (interactor_styles::CLICK_TRACKBALL));
  //multiplexer_->connect (action_manipulate_selected_, SIGNAL (triggered ()), SLOT (selectedTrackballInteractorStyle ()));

  
  multiplexer_->connect (action_new_cloud_from_selection_, SIGNAL (triggered ()), SLOT (createNewCloudFromSelection ()));
  multiplexer_->connect (SIGNAL (newCloudFromSelectionAvailable (bool)), action_new_cloud_from_selection_, SLOT (setEnabled (bool)));
  
  multiplexer_->connect (action_select_all_, SIGNAL (triggered ()), SLOT (selectAllItems ()));
}

void
pcl::cloud_composer::ComposerMainWindow::setMouseStyleAction (interactor_styles::INTERACTOR_STYLES selected_style)
{
  action_trackball_camera_style_->setChecked (false);
  action_rectangular_frustum_select_->setChecked (false);
  action_manipulate_selected_->setChecked (false);
  action_manipulate_clicked_->setChecked (false);
 
  switch (selected_style)
  {
    case interactor_styles::PCL_VISUALIZER :
      action_trackball_camera_style_->setChecked (true);
      break;
    case interactor_styles::CLICK_TRACKBALL :
      action_manipulate_clicked_->setChecked (true);
      break;
    case interactor_styles::RECTANGULAR_FRUSTUM :
      action_rectangular_frustum_select_->setChecked (true);
      break;
    case interactor_styles::SELECTED_TRACKBALL :
      action_manipulate_selected_->setChecked (true);
      break;
    default :
      action_trackball_camera_style_->setChecked (true);
    
  }
}

void
pcl::cloud_composer::ComposerMainWindow::connectViewActions ()
{
  multiplexer_->connect (action_show_axes_, SIGNAL (toggled (bool)), SLOT (setAxisVisibility (bool)));
  multiplexer_->connect (SIGNAL (axisVisible (bool)), action_show_axes_, SLOT (setChecked (bool)));
  
}

void
pcl::cloud_composer::ComposerMainWindow::initializeCloudBrowser ()
{
  cloud_browser_->setSelectionMode (QAbstractItemView::ExtendedSelection);
  
  cloud_browser_->setStyleSheet("selection-background-color: red;");
  
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
  tool_box_model_ = new ToolBoxModel (tool_box_view_, tool_parameter_view_,this);
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
  
  connect (this, SIGNAL (activeProjectChanged (ProjectModel*,ProjectModel*)),
           tool_box_model_, SLOT (activeProjectChanged (ProjectModel*,ProjectModel*)));
  
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
#if _WIN32
  if (!plugin_dir.cd (QDir::cleanPath ("cloud_composer_plugins")))
#else
  if (!plugin_dir.cd (QDir::cleanPath ("../lib/cloud_composer_plugins")))
#endif
  {
    #if _WIN32
      if (!plugin_dir.cd (QDir::cleanPath ("cloud_composer_plugins")))
    #else
      if (!plugin_dir.cd (QDir::cleanPath ("../lib")))
    #endif
      {
        qCritical () << "Could not find plugin tool directory!!!";
      }
  }
  QStringList plugin_filter;
#if _WIN32
  plugin_filter << "pcl_cc_tool_*.dll";
#else
  plugin_filter << "libpcl_cc_tool_*.so";
#endif
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
  emit activeProjectChanged (model, current_model_);
  current_model_ = model;
  //qDebug () << "Setting cloud browser model";
  cloud_browser_->setModel (current_model_);
  //qDebug () << "Setting cloud browser selection model";
  cloud_browser_->setSelectionModel (current_model_->getSelectionModel ());
  //qDebug () << "Item inspector setting model";
  item_inspector_->setModel (current_model_);
  //qDebug () << "Setting active stack in undo group";
  undo_group_->setActiveStack (current_model_->getUndoStack ());
  
  multiplexer_->setCurrentObject (current_model_);
}

void
pcl::cloud_composer::ComposerMainWindow::enqueueToolAction (AbstractTool* tool)
{
  if (current_model_)
    current_model_->enqueueToolAction (tool);
  else
    QMessageBox::warning (this, "No Project Open!", "Cannot use tool, no project is open!");
}
///////// FILE MENU SLOTS ///////////
void
pcl::cloud_composer::ComposerMainWindow::on_action_new_project__triggered (/*QString name*/)
{
  QString name("unsaved project");

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
  cloud_viewer_->addNewProject (new_project_model);
  
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
pcl::cloud_composer::ComposerMainWindow::on_action_save_selected_cloud__triggered ()
{
  if (current_model_)
    emit saveSelectedCloudToFile();
  else
    QMessageBox::warning (this, "No Project Open!", "Cannot save cloud, no project is open!");

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
  if (!current_model_)
    action_new_project_->trigger ();
    
  emit insertNewCloudFromFile ();   
}

void
pcl::cloud_composer::ComposerMainWindow::on_action_insert_from_rgb_depth__triggered ()
{
  if (!current_model_)
    action_new_project_->trigger ();
  
  emit insertNewCloudFromRGBandDepth ();   
}


void
pcl::cloud_composer::ComposerMainWindow::on_action_insert_from_openNi_source__triggered ()
{
  qDebug () << "Inserting cloud from OpenNi Source...";
}



