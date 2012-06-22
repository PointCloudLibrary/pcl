#include <pcl/apps/cloud_composer/tools/normal_estimation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>


Q_EXPORT_PLUGIN2(cloud_composer_normal_estimation_tool, pcl::cloud_composer::NormalEstimationToolFactory)


pcl::cloud_composer::NormalEstimationTool::NormalEstimationTool (QStandardItemModel* parameter_model, QObject* parent)
  : NewItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::NormalEstimationTool::~NormalEstimationTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::NormalEstimationTool::performAction (ConstItemList input_data)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in Normal Estimation Tool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in Normal Estimation!";
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
    
    //////////////// THE WORK - COMPUTING NORMALS ///////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input_cloud, *cloud); 
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (radius);

    // Compute the features
    ne.compute (*cloud_normals);
    //////////////////////////////////////////////////////////////////
    
    CloudComposerItem* normal_item = new CloudComposerItem ("Normals");
    normal_item->setData (QVariant::fromValue (0), Qt::UserRole);
    output.append (normal_item);
  }
  else
  {
    qDebug () << "Input item in Normal Estimation is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
QStandardItemModel*
pcl::cloud_composer::NormalEstimationToolFactory::createToolParameterModel (QObject* parent)
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