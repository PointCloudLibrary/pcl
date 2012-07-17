
#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>

#include <pcl/apps/cloud_composer/tools/fpfh_estimation.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/apps/cloud_composer/items/fpfh_item.h>

Q_EXPORT_PLUGIN2(cloud_composer_fpfh_estimation_tool, pcl::cloud_composer::FPFHEstimationToolFactory)


pcl::cloud_composer::FPFHEstimationTool::FPFHEstimationTool (QStandardItemModel* parameter_model, QObject* parent)
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
    //TODO: Helper function for extracting parameters
    QList <QStandardItem*> radius_param = parameter_model_->findItems ("Radius");
    double radius = 0;
    if (radius_param.size () > 0)
      if (radius_param.value (0)->hasChildren ())
        radius = (radius_param.value (0)->child (0)->data (Qt::EditRole)).toDouble ();
    qDebug () << "Received Radius = " <<radius;
    
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

    //Get the cloud in template form
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input_cloud, *cloud); 
    
    //Get the normals cloud from the item
    QVariant normals_variant = normals_item->data (NORMALS_CLOUD);
    pcl::PointCloud<pcl::Normal>::ConstPtr input_normals = normals_variant.value<pcl::PointCloud<pcl::Normal>::ConstPtr> ();
    
    
    //////////////// THE WORK - COMPUTING FPFH ///////////////////
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (input_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (radius);

    // Compute the features
    fpfh.compute (*fpfhs);
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
QStandardItemModel*
pcl::cloud_composer::FPFHEstimationToolFactory::createToolParameterModel (QObject* parent)
{
  QStandardItemModel* parameter_model = new QStandardItemModel(parent);

  // TODO: Make a helper function that returns parameters like this
  //QList <QStandardItem*> new_row;
  QStandardItem* new_property = new QStandardItem ("Radius");
  new_property->setEditable (false);
  //new_row.append (new_property);
  QStandardItem* new_value = new QStandardItem ();
  new_value->setData (0.02, Qt::EditRole);
  new_property->appendRow (new_value);
  // ///////////////////////////////
  parameter_model->appendRow (new_property);
  
  return parameter_model;
}