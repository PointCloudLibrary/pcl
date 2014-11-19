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

#ifndef IMPL_SUPERVOXELS_HPP_
#define IMPL_SUPERVOXELS_HPP_

#include <pcl/apps/cloud_composer/tools/supervoxels.h>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxel_clustering.h>


template <typename PointT> QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::SupervoxelsTool::performTemplatedAction (QList <const CloudComposerItem*> input_data)
{
  QList <CloudComposerItem*> output;  
  
  foreach (const CloudComposerItem* input_item, input_data)
  {
   // if ( !input_item->isSanitized () )
  //  {
  //    qCritical () << "SupervoxelsTool requires sanitized input!";
  //    return output;
  //  }
    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    if ( ! variant.canConvert <typename PointCloud<PointT>::Ptr> () )
    {  
      qWarning () << "Attempted to cast to template type which does not exist in this item! (input list)";
      return output;
    }
    typename PointCloud <PointT>::Ptr input_cloud = variant.value <typename PointCloud<PointT>::Ptr> ();
    //TODO: Check if Voxelized
    
}
  

  foreach (const CloudComposerItem* input_item, input_data)
  {  
    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    typename PointCloud <PointT>::Ptr input_cloud = variant.value <typename PointCloud<PointT>::Ptr> ();
    
    float resolution = parameter_model_->getProperty("Resolution").toFloat ();
    qDebug () << "Octree resolution = "<<resolution;
    float seed_resolution = parameter_model_->getProperty("Seed Resolution").toFloat ();
    qDebug () << "Seed resolution = "<<seed_resolution;
    
    float rgb_weight = parameter_model_->getProperty("RGB Weight").toFloat ();
    float normal_weight = parameter_model_->getProperty("Normals Weight").toFloat ();
    float spatial_weight = parameter_model_->getProperty("Spatial Weight").toFloat ();
    
  
    pcl::SupervoxelClustering<PointT> super (resolution, seed_resolution);
    super.setInputCloud (input_cloud);
    super.setColorImportance (rgb_weight);
    super.setSpatialImportance (spatial_weight);
    super.setNormalImportance (normal_weight);
    std::map <uint32_t, typename pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
    super.extract (supervoxel_clusters);
    
    std::map <uint32_t, typename pcl::Supervoxel<PointT>::Ptr > refined_supervoxel_clusters;
    super.refineSupervoxels (3, refined_supervoxel_clusters);
  
    typename pcl::PointCloud<PointXYZRGBA>::Ptr color_segments;
    color_segments= super.getColoredVoxelCloud ();
    
    CloudItem*  cloud_item_out = CloudItem::createCloudItemFromTemplate<PointXYZRGBA>(input_item->text(),color_segments);
 
    
    output.append (cloud_item_out);
    
  }
  
  
  
  
  return output;
  
}




#define PCL_INSTANTIATE_performTemplatedAction(T) template PCL_EXPORTS void pcl::cloud_composer::SupervoxelsTool::performTemplatedAction<T> (QList <const CloudComposerItem*>);



#endif //IMPL_SUPERVOXELS_HPP_