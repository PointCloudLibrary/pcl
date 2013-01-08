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

#ifndef IMPL_VOXEL_SUPERPIXELS_HPP_
#define IMPL_VOXEL_SUPERPIXELS_HPP_

#include <pcl/apps/cloud_composer/tools/voxel_superpixels.h>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxels.h>


template <typename PointT> QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::VoxelSuperpixelsTool::performTemplatedAction (QList <const CloudComposerItem*> input_data)
{
  QList <CloudComposerItem*> output;  
  
  foreach (const CloudComposerItem* input_item, input_data)
  {
   // if ( !input_item->isSanitized () )
  //  {
  //    qCritical () << "VoxelSuperpixelsTool requires sanitized input!";
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
    
    pcl::SuperVoxels<PointT> super (resolution, seed_resolution);
    super.setInputCloud (input_cloud);
    super.setColorImportance (rgb_weight);
    super.setSpatialImportance (spatial_weight);
    super.setNormalImportance (normal_weight);
    pcl::PointCloud<pcl::PointSuperVoxel>::Ptr supervoxel_cloud;
    super.extract (supervoxel_cloud);
    
    
    
    typename pcl::PointCloud<PointXYZRGB>::Ptr color_segments;
    color_segments = super.getColoredVoxelCloud ();
    
    typename pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud;
    label_cloud = super.getLabeledVoxelCloud ();
    
    std::map<uint32_t, PointSuperVoxel> label_centers;
    super.getSuperVoxelCenters (label_centers);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    std::map<uint32_t, PointSuperVoxel>::iterator itr_centers = label_centers.begin ();
    
    typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, pcl::PointSuperVoxel, pcl::octree::EdgeProperties> VoxelAdjacencyList;
    typedef boost::graph_traits<VoxelAdjacencyList>::vertex_iterator VertexIterator;
    VoxelAdjacencyList supervoxel_adjacency_list;  
    super.getSuperVoxelAdjacencyList (supervoxel_adjacency_list);
    //The vertices in the supervoxel adjacency list are the supervoxel centroids    
    std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
    vertex_iterator_range = boost::vertices(supervoxel_adjacency_list);
    for (VertexIterator itr=vertex_iterator_range.first ; itr != vertex_iterator_range.second; ++itr)
    {
      PointSuperVoxel label_centroid = supervoxel_adjacency_list[*itr];
      std::cout << label_centroid<<std::endl;
    }
    
    for (; itr_centers != label_centers.end () ; ++itr_centers)
      std::cout << itr_centers->second<<std::endl;
    for (int i = 0; i < label_cloud->points.size (); ++i)
    {
      pcl::PointXYZL point = label_cloud->points[i];
      pcl::PointSuperVoxel super_center = label_centers[point.label];
      
      pcl::Normal normal_point;
      //point.normal = itr_centers->second.normal;
      normal_point.normal_x = super_center.normal_x;
      normal_point.normal_y = super_center.normal_y;
      normal_point.normal_z = super_center.normal_z;
      normal_point.curvature = super_center.curvature;
     // std::cout << point.x <<" "<< point.y <<" "<< point.z <<" "<< point.label<<"\n";
    //  std::cout << super_center<< std::endl;
     // std::cout << normal_point<<std::endl<<std::endl;
      cloud_normals->points.push_back (normal_point);
    }    
    CloudItem*  cloud_item_out = CloudItem::createCloudItemFromTemplate<PointXYZRGB>(input_item->text(),color_segments);
    
    NormalsItem* normals_item = new NormalsItem (tr("Normals r=%1").arg(0.03),cloud_normals,0.03);

    cloud_item_out->addChild (normals_item);
    
    output.append (cloud_item_out);
    
  }
  
  
  
  
  return output;
  
}




#define PCL_INSTANTIATE_performTemplatedAction(T) template PCL_EXPORTS void pcl::cloud_composer::VoxelSuperpixelsTool::performTemplatedAction<T> (QList <const CloudComposerItem*>);



#endif //IMPL_VOXEL_SUPERPIXELS_HPP_