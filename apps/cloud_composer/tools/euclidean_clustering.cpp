#include <pcl/apps/cloud_composer/tools/euclidean_clustering.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/memory.h>  // for pcl::make_shared
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")

pcl::cloud_composer::EuclideanClusteringTool::EuclideanClusteringTool (PropertiesModel* parameter_model, QObject* parent)
  : SplitItemTool (parameter_model, parent)
{

}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::EuclideanClusteringTool::performAction (ConstItemList input_data, PointTypeFlags::PointType)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.empty ())
  {
    qCritical () << "Empty input in Euclidean Clustering Tool!";
    return output;
  }
  if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in Euclidean Clustering!";
  }
  input_item = input_data.value (0);

  if (input_item->type () == CloudComposerItem::CLOUD_ITEM)
  {
    const CloudItem* cloud_item = dynamic_cast <const CloudItem*> (input_item);
    if ( cloud_item->isSanitized())
    {
      double cluster_tolerance = parameter_model_->getProperty ("Cluster Tolerance").toDouble();
      int min_cluster_size = parameter_model_->getProperty ("Min Cluster Size").toInt();
      int max_cluster_size = parameter_model_->getProperty ("Max Cluster Size").toInt();

      pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
      //Get the cloud in template form
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2 (*input_cloud, *cloud);

      //////////////// THE WORK - COMPUTING CLUSTERS ///////////////////
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (cluster_tolerance); 
      ec.setMinClusterSize (min_cluster_size);
      ec.setMaxClusterSize (max_cluster_size);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud);
      ec.extract (cluster_indices);
      //////////////////////////////////////////////////////////////////
      //Get copies of the original origin and orientation
      Eigen::Vector4f source_origin = input_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
      Eigen::Quaternionf source_orientation =  input_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
      //Vector to accumulate the extracted indices
      pcl::IndicesPtr extracted_indices (new pcl::Indices ());
      //Put found clusters into new cloud_items!
      qDebug () << "Found "<<cluster_indices.size ()<<" clusters!";
      int cluster_count = 0;
      pcl::ExtractIndices<pcl::PCLPointCloud2> filter;
      for (const auto& cluster : cluster_indices)
      {
        filter.setInputCloud (input_cloud);
        // It's annoying that I have to do this, but Euclidean returns a PointIndices struct
        pcl::PointIndices::ConstPtr indices_ptr = pcl::make_shared<pcl::PointIndices>(cluster);
        filter.setIndices (indices_ptr);
        extracted_indices->insert (extracted_indices->end (), cluster.indices.begin (), cluster.indices.end ());
        //This means remove the other points
        filter.setKeepOrganized (false);
        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
        filter.filter (*cloud_filtered);

        qDebug() << "Cluster has " << cloud_filtered->width << " data points.";
        CloudItem* cloud_item = new CloudItem (input_item->text ()+tr("-Clstr %1").arg(cluster_count)
                                              , cloud_filtered
                                              , source_origin
                                              , source_orientation);
        output.append (cloud_item);
        ++cluster_count;
      }
      //We copy input cloud over for special case that no clusters found, since ExtractIndices doesn't work for 0 length vectors
      pcl::PCLPointCloud2::Ptr remainder_cloud (new pcl::PCLPointCloud2(*input_cloud));
      if (!cluster_indices.empty ())
      {
        //make a cloud containing all the remaining points
        filter.setIndices (extracted_indices);
        filter.setNegative (true);
        filter.filter (*remainder_cloud);
      }
      qDebug() << "Cloud has " << remainder_cloud->width << " data points after clusters removed.";
      CloudItem* cloud_item = new CloudItem (input_item->text ()+ " unclustered"
                                              , remainder_cloud
                                              , source_origin
                                              , source_orientation);
      output.push_front (cloud_item);
    }
    else
      qCritical () << "Input item in Clustering is not SANITIZED!!!";
  }
  else
  {
    qCritical () << "Input item in Clustering is not a cloud!!!";
  }


  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::EuclideanClusteringToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);

  parameter_model->addProperty ("Cluster Tolerance", 0.02,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Min Cluster Size", 100,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Max Cluster Size", 25000,  Qt::ItemIsEditable | Qt::ItemIsEnabled);

  return parameter_model;
}
