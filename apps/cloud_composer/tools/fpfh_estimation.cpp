#include <pcl/apps/cloud_composer/tools/fpfh_estimation.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/apps/cloud_composer/items/fpfh_item.h>

#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>



Q_EXPORT_PLUGIN2(cloud_composer_fpfh_estimation_tool, pcl::cloud_composer::FPFHEstimationToolFactory)


pcl::cloud_composer::FPFHEstimationTool::FPFHEstimationTool (PropertiesModel* parameter_model, QObject* parent)
  : NewItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::FPFHEstimationTool::~FPFHEstimationTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::FPFHEstimationTool::performAction (ConstItemList input_data)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in FPFH Estimation Tool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in FPFH Estimation!";
  }
  input_item = input_data.value (0);
  
  sensor_msgs::PointCloud2::ConstPtr input_cloud;
  if (input_item->getCloudConstPtr (input_cloud))
  {
    double radius = parameter_model_->getProperty("Radius").toDouble();;
       
    //Check if this cloud already has normals computed!
    const QStandardItem* normals_item = 0;
    for (int i = 0; i < input_item->rowCount (); ++i)
      if ( input_item->child (i)->type () == NORMALS_ITEM )
        normals_item = input_item->child (i);
    if ( !normals_item )
    {
      qCritical () << "No normals item found in this cloud (TODO: automatically do it now)";
      return output;
    }

    
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    //Get the cloud in template form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input_cloud, *cloud); 
 //   qDebug () << "Input cloud size = "<<cloud->size ();
  //  std::vector <int> indices;
  //  pcl::removeNaNFromPointCloud (*cloud, *cloud, indices);
  //  qDebug () << "Nans filtered size = "<<cloud->size ();
    
    //Get the normals cloud from the item
    QVariant normals_variant = normals_item->data (NORMALS_CLOUD);
    pcl::PointCloud<pcl::Normal>::Ptr input_normals = normals_variant.value<pcl::PointCloud<pcl::Normal>::Ptr> ();

    //////////////// THE WORK - COMPUTING FPFH ///////////////////
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (input_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    qDebug () << "Building KD Tree";
    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (radius);

    // Compute the features
    qDebug () << "Computing FPFH features";
    fpfh.compute (*fpfhs);
    qDebug () << "Size of computed features ="<<fpfhs->width;
    //////////////////////////////////////////////////////////////////
    FPFHItem* fpfh_item = new FPFHItem (tr("FPFH r=%1").arg(radius),fpfhs,radius);
    output.append (fpfh_item);
  }
  else
  {
    qCritical () << "Input item in FPFH Estimation is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::FPFHEstimationToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Radius", 0.03,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  return parameter_model;
}