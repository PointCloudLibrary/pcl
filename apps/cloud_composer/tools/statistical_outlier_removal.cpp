/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/apps/cloud_composer/tools/statistical_outlier_removal.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  Q_EXPORT_PLUGIN2(cloud_composer_statistical_outlier_removal_tool, pcl::cloud_composer::StatisticalOutlierRemovalToolFactory)
#else
  Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")
#endif

pcl::cloud_composer::StatisticalOutlierRemovalTool::StatisticalOutlierRemovalTool (PropertiesModel* parameter_model, QObject* parent)
  : ModifyItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::StatisticalOutlierRemovalTool::~StatisticalOutlierRemovalTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::StatisticalOutlierRemovalTool::performAction (ConstItemList input_data, PointTypeFlags::PointType)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in StatisticalOutlierRemovalTool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in StatisticalOutlierRemovalTool";
  }
  input_item = input_data.value (0);
  if ( !input_item->isSanitized () )
  {
    qCritical () << "StatisticalOutlierRemovalTool requires sanitized input!";
    return output;
  }
  
  if (input_item->type () ==  CloudComposerItem::CLOUD_ITEM )
  {
    pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
    
    int mean_k = parameter_model_->getProperty("Mean K").toInt ();
    double std_dev_thresh = parameter_model_->getProperty ("Std Dev Thresh").toDouble ();
    
    //////////////// THE WORK - FILTERING OUTLIERS ///////////////////
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (mean_k);
    sor.setStddevMulThresh (std_dev_thresh);
    
    //Create output cloud
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
    //Filter!  
    sor.filter (*cloud_filtered);

    //////////////////////////////////////////////////////////////////
    //Get copies of the original origin and orientation
    Eigen::Vector4f source_origin = input_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
    Eigen::Quaternionf source_orientation =  input_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
    //Put the modified cloud into an item, stick in output
    CloudItem* cloud_item = new CloudItem (input_item->text () + tr (" sor filtered")
                                           , cloud_filtered
                                           , source_origin
                                           , source_orientation);

    
    output.append (cloud_item);
  }
  else
  {
    qDebug () << "Input item in StatisticalOutlierRemovalTool is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::StatisticalOutlierRemovalToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Mean K", 50,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Std Dev Thresh", 1.0,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  return parameter_model;
}
