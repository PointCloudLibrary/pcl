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

#ifndef IMPL_TRANSFORM_CLOUDS_HPP_
#define IMPL_TRANSFORM_CLOUDS_HPP_

#include <pcl/apps/cloud_composer/transform_clouds.h>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

template <typename PointT> QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::TransformClouds::performTemplatedAction (QList <const CloudComposerItem*> input_data)
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
    if (!transform_map_.contains ("AllSelectedClouds") && !transform_map_.contains (input_item->getId ()))
    {
      qCritical () << "No transform found for id "<<input_item->getId ()<<" in TransformClouds::performTemplatedAction";
      return output;
    }
  }
  
  foreach (const CloudComposerItem* input_item, input_data)
  {
    qDebug () << "Transforming cloud "<<input_item->getId ();
    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    typename PointCloud <PointT>::Ptr input_cloud = variant.value <typename PointCloud<PointT>::Ptr> ();
    
    Eigen::Matrix4f transform;
    if (transform_map_.contains ("AllSelectedClouds"))
      pcl::visualization::PCLVisualizer::convertToEigenMatrix (transform_map_.value ("AllSelectedClouds"), transform);
    else
      pcl::visualization::PCLVisualizer::convertToEigenMatrix (transform_map_.value (input_item->getId ()), transform);
    
    typename PointCloud<PointT>::Ptr transformed_cloud = boost::make_shared<PointCloud<PointT> > ();
    
    transformPointCloud<PointT> (*input_cloud, *transformed_cloud, transform);
    CloudItem*  new_cloud_item = CloudItem::createCloudItemFromTemplate<PointT>(input_item->text (),transformed_cloud);
    
    output.append (new_cloud_item);
  }

  


  return output;
  
}


#define PCL_INSTANTIATE_performTemplatedAction(T) template PCL_EXPORTS void pcl::cloud_composer::TransformClouds::performTemplatedAction<T> (QList <const CloudComposerItem*>);



#endif //IMPL_TRANSFORM_CLOUDS_HPP_