 /*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef IMPL_MERGE_SELECTION_H_
#define IMPL_MERGE_SELECTION_H_

#include <pcl/apps/cloud_composer/merge_selection.h>
#include <pcl/point_cloud.h>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>

template <typename PointT> QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::MergeSelection::performTemplatedAction (QList <const CloudComposerItem*> input_data)
{
  QList <CloudComposerItem*> output;  
  
  foreach (const CloudComposerItem* input_item, input_data)
  {
    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    if ( ! variant.canConvert <typename PointCloud<PointT>::Ptr> () )
    {  
      qWarning () << "Attempted to cast to template type which does not exist in this item! (input list)";
      return output;
    }
  }
  foreach (const CloudItem* input_item, selected_item_index_map_.keys ())
  {
    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    if ( ! variant.canConvert <typename PointCloud<PointT>::Ptr> () )
    {  
      qWarning () << "Attempted to cast to template type which does not exist in this item! (selected list)";
      return output;
    }
  }  

  pcl::ExtractIndices<PointT> filter;
  typename PointCloud<PointT>::Ptr merged_cloud = boost::make_shared<PointCloud<PointT> > ();

  foreach (const CloudItem* input_cloud_item, selected_item_index_map_.keys ())
  {
    input_cloud_item->printNumPoints <PointT> ();
    //If this cloud hasn't been completely selected 
    if (!input_data.contains (input_cloud_item))
    {
      typename PointCloud<PointT>::Ptr input_cloud = input_cloud_item->data (ItemDataRole::CLOUD_TEMPLATED).value <typename PointCloud<PointT>::Ptr> ();
      qDebug () << "Extracting "<<selected_item_index_map_.value(input_cloud_item)->indices.size() << " points out of "<<input_cloud->width;
      filter.setInputCloud (input_cloud);
      filter.setIndices (selected_item_index_map_.value (input_cloud_item));
      typename PointCloud<PointT>::Ptr original_minus_indices = boost::make_shared<PointCloud<PointT> > ();
      filter.setNegative (true);
      filter.filter (*original_minus_indices);
      filter.setNegative (false);
      typename PointCloud<PointT>::Ptr selected_points = boost::make_shared<PointCloud<PointT> > ();
      filter.filter (*selected_points);
      
      qDebug () << "Original minus indices is "<<original_minus_indices->width;
      
      //Eigen::Vector4f source_origin = input_cloud_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
      //Eigen::Quaternionf source_orientation =  input_cloud_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
      //pcl::PCLPointCloud2::Ptr cloud_blob = boost::make_shared <pcl::PCLPointCloud2> ();;
      //toPCLPointCloud2 (*original_minus_indices, *cloud_blob);
      //CloudItem* new_cloud_item = new CloudItem (input_cloud_item->text ()
                                                  //, cloud_blob
                                            //      , source_origin
                                            //      , source_orientation);
      
      CloudItem*  new_cloud_item = CloudItem::createCloudItemFromTemplate<PointT>(input_cloud_item->text (),original_minus_indices);
      
      output.append (new_cloud_item);
      *merged_cloud += *selected_points;
    }
    //Append the input item to the original list
    //input_data.append (input_cloud_item);
  }
  //Just concatenate for all fully selected clouds
  foreach (const CloudComposerItem* input_item, input_data)
  {
    typename PointCloud<PointT>::Ptr input_cloud = input_item->data (ItemDataRole::CLOUD_TEMPLATED).value <typename PointCloud<PointT>::Ptr> ();
    *merged_cloud += *input_cloud;
  }
  CloudItem* cloud_item = CloudItem::createCloudItemFromTemplate<PointT>("Cloud from Selection",merged_cloud);
      
  output.append (cloud_item);
    
  return output;
  
}


#define PCL_INSTANTIATE_performTemplatedAction(T) template PCL_EXPORTS void pcl::cloud_composer::MergeSelection::performTemplatedAction<T> (QList <const CloudComposerItem*>);



#endif //IMPL_MERGE_SELECTION_H_