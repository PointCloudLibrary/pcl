#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/cloud_view.h>
#include <pcl/apps/cloud_composer/cloud_composer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>

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
  connect (model_, SIGNAL (dataChanged (const QModelIndex&, const QModelIndex&)),
           this, SLOT (dataChanged (const QModelIndex&, const QModelIndex&)));
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
pcl::cloud_composer::CloudView::dataChanged (const QModelIndex&, const QModelIndex&)
{
  qDebug () << "Data Changed - Redrawing!";
  
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
    //Handle what type of item this is - normals, cloud, etc...
    switch (item->type ())
    {
      case CLOUD_ITEM:
      {  
        qDebug () << "New cloud inserted - adding to visualizer";
        QString cloud_name = item->text ();
        QVariant cloud_ptr = item->data (CLOUD);
        sensor_msgs::PointCloud2::Ptr cloud_blob = cloud_ptr.value<sensor_msgs::PointCloud2::Ptr> ();
        QVariant color_ptr = item->data (COLOR);
        ColorHandler::ConstPtr color_handler = color_ptr.value<ColorHandler::ConstPtr> ();
        QVariant geometry_ptr = item->data (GEOMETRY);
        GeometryHandler::ConstPtr geometry_handler = geometry_ptr.value<GeometryHandler::ConstPtr> ();
        Eigen::Vector4f origin = item->data (ORIGIN).value<Eigen::Vector4f> (); 
        Eigen::Quaternionf orientation = item->data (ORIENTATION).value<Eigen::Quaternionf> (); 
        vis_->addPointCloud (cloud_blob, geometry_handler, color_handler, origin, orientation, cloud_name.toStdString ());
        break;
      }
      case NORMALS_ITEM:
      {
        qDebug () << "New normals inserted - adding to visualizer";
        double radius = (item->getProperty ("Radius")).toDouble();
        
        QVariant normals_variant = item->data (NORMALS_CLOUD);
        pcl::PointCloud<pcl::Normal>::Ptr normals_ptr = normals_variant.value<pcl::PointCloud<pcl::Normal>::Ptr> ();
        QString normals_name = project_name + item->text () + tr ("%1%2").arg (radius).arg (long (item));
        //Get the parent cloud, convert to XYZ 
        if (parent_item->type () == CLOUD_ITEM)
        {
          QVariant cloud_ptr = parent_item->data (CLOUD);
          sensor_msgs::PointCloud2::Ptr cloud_blob = cloud_ptr.value<sensor_msgs::PointCloud2::Ptr> ();
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromROSMsg (*cloud_blob, *cloud); 
          //TODO: Add somewhere where these parameters can be adjusted!!
          vis_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals_ptr, 100, 0.04f, normals_name.toStdString ());
        }
        else
          qWarning () << "Normal item inserted, but parent not a cloud. Don't know how to draw that!";
        break;
      }
      default:
        qDebug () << "Unrecognized item type in CloudView- not displaying";
      
    }
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
        double radius = (item->getProperty ("Radius")).toDouble();
        QVariant normals_variant = item->data (NORMALS_CLOUD);
        pcl::PointCloud<pcl::Normal>::Ptr normals_ptr = normals_variant.value<pcl::PointCloud<pcl::Normal>::Ptr> ();
        QString normals_name = project_name + item->text () + tr ("%1%2").arg (radius).arg (long (item));
        vis_->removePointCloud (normals_name.toStdString ());
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
