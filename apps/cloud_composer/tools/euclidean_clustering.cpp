#include <pcl/apps/cloud_composer/tools/euclidean_clustering.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>




Q_EXPORT_PLUGIN2(cloud_composer_euclidean_clustering_tool, pcl::cloud_composer::EuclideanClusteringToolFactory)


pcl::cloud_composer::EuclideanClusteringTool::EuclideanClusteringTool (PropertiesModel* parameter_model, QObject* parent)
  : NewItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::EuclideanClusteringTool::~EuclideanClusteringTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::EuclideanClusteringTool::performAction (ConstItemList input_data)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in Euclidean Clustering Tool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in Euclidean Clustering!";
  }
  input_item = input_data.value (0);
  
  sensor_msgs::PointCloud2::ConstPtr input_cloud;
  if (input_item->getCloudConstPtr (input_cloud))
  {
    double cluster_tolerance = parameter_model_->getProperty ("Cluster Tolerance").toDouble();
    int min_cluster_size = parameter_model_->getProperty ("Min Cluster Size").toInt();
    int max_cluster_size = parameter_model_->getProperty ("Max Cluster Size").toInt();
   

    //Get the cloud in template form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input_cloud, *cloud); 
     
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
    //Put found clusters into new cloud_items!
    qDebug () << "Found "<<cluster_indices.size ()<<" clusters!";
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      qDebug() << "Cluster has " << cloud_cluster->points.size () << " data points.";
    }
    //FPFHItem* fpfh_item = new FPFHItem (tr("FPFH r=%1").arg(radius),fpfhs,radius);
   // output.append (fpfh_item);
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