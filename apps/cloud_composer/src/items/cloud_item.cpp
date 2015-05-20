#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/filters/passthrough.h>


#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>

pcl::cloud_composer::CloudItem::CloudItem (QString name,
                                           pcl::PCLPointCloud2::Ptr cloud_ptr,
                                           const Eigen::Vector4f& origin, 
                                           const Eigen::Quaternionf& orientation,
                                           bool make_templated_cloud)
  : CloudComposerItem (name)
  , origin_ (origin)
  , orientation_ (orientation)
  , template_cloud_set_ (false)
  , point_type_ (PointTypeFlags::NONE)
  , is_sanitized_ (false)
{
  
  //Sanitize the cloud data using passthrough
 // qDebug () << "Cloud size before passthrough : "<<cloud_ptr->width<<"x"<<cloud_ptr->height;

//  qDebug () << "Cloud size after passthrough : "<<cloud_filtered->width<<"x"<<cloud_filtered->height;
  cloud_blob_ptr_ = cloud_ptr;
  pcl::PCLPointCloud2::ConstPtr const_cloud_ptr = cloud_ptr;
  this->setData (QVariant::fromValue (const_cloud_ptr), ItemDataRole::CLOUD_BLOB);
  this->setData (QVariant::fromValue (origin_), ItemDataRole::ORIGIN);
  this->setData (QVariant::fromValue (orientation_), ItemDataRole::ORIENTATION);
   
  //Create a color and geometry handler for this cloud
  color_handler_.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (color_handler_), ItemDataRole::COLOR_HANDLER);
  geometry_handler_.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (geometry_handler_), ItemDataRole::GEOMETRY_HANDLER);
     
  properties_->addCategory ("Core Properties");
  properties_->addProperty ("Name", QVariant (this->text ()), Qt::NoItemFlags, "Core Properties");
  properties_->addProperty ("Height", QVariant (cloud_blob_ptr_->height), Qt::NoItemFlags, "Core Properties");
  properties_->addProperty ("Width", QVariant (cloud_blob_ptr_->width), Qt::NoItemFlags,"Core Properties");
  properties_->addCategory ("Display Variables");
  properties_->addProperty ("Point Size", QVariant (1.0), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Variables");
  properties_->addProperty ("Opacity", QVariant (1.0), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Variables");
 
  if (make_templated_cloud)
    setTemplateCloudFromBlob ();
  
  if (checkIfFinite ())
    is_sanitized_ = true;
  
}


pcl::cloud_composer::CloudItem*
pcl::cloud_composer::CloudItem::clone () const
{
  pcl::PCLPointCloud2::Ptr cloud_copy (new pcl::PCLPointCloud2 (*cloud_blob_ptr_));
  //Vector4f and Quaternionf do deep copies using constructor
  CloudItem* new_item = new CloudItem (this->text (), cloud_copy, origin_,orientation_);
  
  PropertiesModel* new_item_properties = new_item->getPropertiesModel ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::CloudItem::~CloudItem ()
{
  qDebug () << "Cloud item destructor";
}


void
pcl::cloud_composer::CloudItem::paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{
  vis->addPointCloud (cloud_blob_ptr_, geometry_handler_, color_handler_, origin_, orientation_, getId ().toStdString ());
  vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, properties_->getProperty ("Point Size").toDouble (), getId ().toStdString ());
  vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, properties_->getProperty ("Opacity").toDouble (), getId ().toStdString ());
 
}

void
pcl::cloud_composer::CloudItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{  
  vis->removePointCloud (getId ().toStdString ());
}

QVariant
pcl::cloud_composer::CloudItem::data (int role) const
{
  // Check if we're trying to get something which is template dependant, if so, create the template if it hasn't been set
  if ( (role == ItemDataRole::CLOUD_TEMPLATED || role == ItemDataRole::KD_TREE_SEARCH) && !template_cloud_set_)
  {
    qCritical () << "Attempted to access templated types which are not set!!";
  }
  
  return CloudComposerItem::data (role);
}

void
pcl::cloud_composer::CloudItem::setData ( const QVariant & value, int role)
{
  if ( role == ItemDataRole::CLOUD_TEMPLATED )
    template_cloud_set_ = true;
  
  CloudComposerItem::setData (value, role);
}

void
pcl::cloud_composer::CloudItem::setTemplateCloudFromBlob ()
{
  if (! template_cloud_set_ )
  {
    std::vector<pcl::PCLPointField>::iterator end = cloud_blob_ptr_->fields.end ();
    std::vector<pcl::PCLPointField>::iterator itr = cloud_blob_ptr_->fields.begin ();
    QStringList field_names;
    for ( itr = cloud_blob_ptr_->fields.begin ()  ; itr != end; ++itr)
    {
      field_names.append (QString::fromStdString ( itr->name ));
    }
    point_type_ = PointTypeFlags::NONE;
    if (field_names.contains ("x") && field_names.contains ("y") && field_names.contains ("z"))
      point_type_ = (point_type_ | PointTypeFlags::XYZ);  
    if (field_names.contains ("rgb"))
      point_type_ = point_type_ | PointTypeFlags::RGB;
    if (field_names.contains ("rgba"))
      point_type_ = point_type_ | PointTypeFlags::RGBA;
    if (field_names.contains ("normal_x") && field_names.contains ("normal_y") && field_names.contains ("normal_z"))
    {
      if (field_names.contains ("curvature"))
        point_type_ = point_type_ | PointTypeFlags::NORMAL;
      else
        point_type_ = point_type_ | PointTypeFlags::AXIS;
    }
    if (field_names.contains ("h") && field_names.contains ("s") && field_names.contains ("v"))
      point_type_ = point_type_ | PointTypeFlags::HSV; 
    
    QVariant cloud_pointer_variant;
    QVariant kd_tree_variant;
    switch (point_type_)
    {
      case (PointTypeFlags::XYZ):
      {
        pcl::PointCloud <PointXYZ>::Ptr cloud_ptr =  boost::make_shared<pcl::PointCloud <PointXYZ> >();
        pcl::fromPCLPointCloud2 (*cloud_blob_ptr_, *cloud_ptr);
        cloud_pointer_variant = QVariant::fromValue (cloud_ptr);
        //Initialize the search kd-tree for this cloud
        pcl::search::KdTree<PointXYZ>::Ptr kd_search = boost::make_shared<search::KdTree<PointXYZ> >();
        kd_search->setInputCloud (cloud_ptr);
        kd_tree_variant = QVariant::fromValue (kd_search);
        break;
      }
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
      {
        pcl::PointCloud <PointXYZRGB>::Ptr cloud_ptr =  boost::make_shared<pcl::PointCloud <PointXYZRGB> >();
        pcl::fromPCLPointCloud2 (*cloud_blob_ptr_, *cloud_ptr);
        cloud_pointer_variant = QVariant::fromValue (cloud_ptr);
        pcl::search::KdTree<PointXYZRGB>::Ptr kd_search = boost::make_shared<search::KdTree<PointXYZRGB> >();
        kd_search->setInputCloud (cloud_ptr);
        kd_tree_variant = QVariant::fromValue (kd_search);
        break;
      }
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
      {
        pcl::PointCloud <PointXYZRGBA>::Ptr cloud_ptr =  boost::make_shared<pcl::PointCloud <PointXYZRGBA> >();
        pcl::fromPCLPointCloud2 (*cloud_blob_ptr_, *cloud_ptr);
        cloud_pointer_variant = QVariant::fromValue (cloud_ptr);
        pcl::search::KdTree<PointXYZRGBA>::Ptr kd_search = boost::make_shared<search::KdTree<PointXYZRGBA> >();
        kd_search->setInputCloud (cloud_ptr);
        kd_tree_variant = QVariant::fromValue (kd_search);
        break;
      }
        
      case (PointTypeFlags::NONE):
        QMessageBox::warning (0,"Unknown blob type!", "Could not find appropriate template type for this cloud blob! Only blob functionality enabled!");
    }
    
    this->setData (cloud_pointer_variant, ItemDataRole::CLOUD_TEMPLATED);
    this->setData (kd_tree_variant, ItemDataRole::KD_TREE_SEARCH);
    template_cloud_set_ = true;
  }
  else
    qDebug () << "Trying to set cloud from blob, but template cloud already set!";
  
}


bool
pcl::cloud_composer::CloudItem::checkIfFinite ()
{
  if (! cloud_blob_ptr_)
    return false;
  
  pcl::PCLPointCloud2::Ptr cloud_filtered = boost::make_shared<pcl::PCLPointCloud2> ();
  pcl::PassThrough<pcl::PCLPointCloud2> pass_filter;
  pass_filter.setInputCloud (cloud_blob_ptr_);
  pass_filter.setKeepOrganized (false);
  pass_filter.filter (*cloud_filtered);
  
  if (cloud_filtered->data.size() == cloud_blob_ptr_->data.size ())
    return true;
  
  return false;

}
