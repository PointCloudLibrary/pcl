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
pcl::cloud_composer::FPFHEstimationTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
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
  
  
  if (input_item->type () == CloudComposerItem::CLOUD_ITEM)
  {
    //Check if this cloud has normals computed!
    QList <CloudComposerItem*> normals_list = input_item->getChildren (CloudComposerItem::NORMALS_ITEM);
    if ( normals_list.size () == 0 )
    {
      qCritical () << "No normals item child found in this cloud item";
      return output;
    }
    qDebug () << "Found item text="<<normals_list.at(0)->text();

    double radius = parameter_model_->getProperty("Radius").toDouble();
    
    pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
    //Get the cloud in template form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*input_cloud, *cloud);
    
    //Get the normals cloud, we just use the first normals that were found if there are more than one
    pcl::PointCloud<pcl::Normal>::ConstPtr input_normals = normals_list.value(0)->data(ItemDataRole::CLOUD_TEMPLATED).value <pcl::PointCloud<pcl::Normal>::ConstPtr> ();
    
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
 //   qDebug () << "Input cloud size = "<<cloud->size ();

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
