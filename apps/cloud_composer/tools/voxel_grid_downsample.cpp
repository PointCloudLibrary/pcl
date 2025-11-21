#include <pcl/apps/cloud_composer/tools/voxel_grid_downsample.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>


Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")

pcl::cloud_composer::VoxelGridDownsampleTool::VoxelGridDownsampleTool (PropertiesModel* parameter_model, QObject* parent)
  : ModifyItemTool (parameter_model, parent)
{

  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::VoxelGridDownsampleTool::performAction (ConstItemList input_data, PointTypeFlags::PointType)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.empty ())
  {
    qCritical () << "Empty input in VoxelGridDownsampleTool!";
    return output;
  }
  if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in VoxelGridDownsampleTool";
  }
  input_item = input_data.value (0);
    
  if (input_item->type () == CloudComposerItem::CLOUD_ITEM)
  {
    double leaf_x = parameter_model_->getProperty("Leaf Size x").toDouble ();
    double leaf_y = parameter_model_->getProperty("Leaf Size y").toDouble ();
    double leaf_z = parameter_model_->getProperty("Leaf Size z").toDouble ();
    
    pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
    
    //////////////// THE WORK - FILTERING OUTLIERS ///////////////////
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
    vox_grid.setInputCloud (input_cloud);
    vox_grid.setLeafSize (float (leaf_x), float (leaf_y), float (leaf_z));
    
    
    //Create output cloud
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
    //Filter!  
    vox_grid.filter (*cloud_filtered);

    //////////////////////////////////////////////////////////////////
    //Get copies of the original origin and orientation
    Eigen::Vector4f source_origin = input_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
    Eigen::Quaternionf source_orientation =  input_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
    //Put the modified cloud into an item, stick in output
    CloudItem* cloud_item = new CloudItem (input_item->text () + tr (" vox ds")
                                           , cloud_filtered
                                           , source_origin
                                           , source_orientation);

    
    output.append (cloud_item);
  }
  else
  {
    qDebug () << "Input item in VoxelGridDownsampleTool is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::VoxelGridDownsampleToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Leaf Size x", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Leaf Size y", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Leaf Size z", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);

  
  return parameter_model;
}
