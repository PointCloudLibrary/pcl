#include <pcl/apps/cloud_composer/merge_selection.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/apps/cloud_composer/impl/merge_selection.hpp>

pcl::cloud_composer::MergeSelection::MergeSelection (QMap <const CloudItem*, pcl::PointIndices::ConstPtr > selected_item_index_map, QObject* parent)
  : MergeCloudTool (0, parent)
  , selected_item_index_map_ (selected_item_index_map)
{
  
}

pcl::cloud_composer::MergeSelection::~MergeSelection ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::MergeSelection::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch (type)
    {
      case (PointTypeFlags::XYZ):
        return this->performTemplatedAction<pcl::PointXYZ> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
        return this->performTemplatedAction<pcl::PointXYZRGB> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
        return this->performTemplatedAction<pcl::PointXYZRGBA> (input_data);
    }
  }
  
  QList <CloudComposerItem*> output;

  // Check input data length
  if ( input_data.size () == 0 && selected_item_index_map_.isEmpty() )
  {
    qCritical () << "Empty input in MergeSelection!";
    return output;
  }

  //Make sure all input items are cloud items
  foreach (const CloudComposerItem* input_item, input_data)
  {
    if (input_item->type () !=  CloudComposerItem::CLOUD_ITEM )
    {
      qCritical () << "Input in MergeSelection not valid, contained non-CloudItem input";
      return output;
    }
  }

  pcl::ExtractIndices<sensor_msgs::PointCloud2> filter;
  sensor_msgs::PointCloud2::Ptr merged_cloud (new sensor_msgs::PointCloud2);
  foreach (const CloudItem* input_cloud_item, selected_item_index_map_.keys ())
  {
    //If this cloud hasn't been completely selected 
    if (!input_data.contains (input_cloud_item))
    {
      sensor_msgs::PointCloud2::ConstPtr input_cloud = input_cloud_item->data (ItemDataRole::CLOUD_BLOB).value <sensor_msgs::PointCloud2::ConstPtr> ();
      qDebug () << "Extracting "<<selected_item_index_map_.value(input_cloud_item)->indices.size() << " points out of "<<input_cloud->width;
      filter.setInputCloud (input_cloud);
      filter.setIndices (selected_item_index_map_.value (input_cloud_item));
      sensor_msgs::PointCloud2::Ptr original_minus_indices = boost::make_shared <sensor_msgs::PointCloud2> ();
      filter.setNegative (true);
      filter.filter (*original_minus_indices);
      filter.setNegative (false);
      sensor_msgs::PointCloud2::Ptr selected_points (new sensor_msgs::PointCloud2);
      filter.filter (*selected_points);
      
      qDebug () << "Original minus indices is "<<original_minus_indices->width;
      Eigen::Vector4f source_origin = input_cloud_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
      Eigen::Quaternionf source_orientation =  input_cloud_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
      CloudItem* new_cloud_item = new CloudItem (input_cloud_item->text ()
                                             , original_minus_indices
                                             , source_origin
                                             , source_orientation);
      output.append (new_cloud_item);
      sensor_msgs::PointCloud2::Ptr temp_cloud = boost::make_shared <sensor_msgs::PointCloud2> ();
      concatenatePointCloud (*merged_cloud, *selected_points, *temp_cloud);
      merged_cloud = temp_cloud;
    }
    //Append the input item to the original list
    //input_data.append (input_cloud_item);
  }
  //Just concatenate for all fully selected clouds
  foreach (const CloudComposerItem* input_item, input_data)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <sensor_msgs::PointCloud2::ConstPtr> ();
    
    sensor_msgs::PointCloud2::Ptr temp_cloud = boost::make_shared <sensor_msgs::PointCloud2> ();
    concatenatePointCloud (*merged_cloud, *input_cloud, *temp_cloud);
    merged_cloud = temp_cloud;
  }
  
  CloudItem* cloud_item = new CloudItem ("Cloud from Selection"
                                         , merged_cloud);

  output.append (cloud_item);
    
  return output;
}