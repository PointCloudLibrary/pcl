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

#ifndef IMPL_CLOUD_ITEM_H_
#define IMPL_CLOUD_ITEM_H_

#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/instantiate.hpp>

template <typename PointT> void
pcl::cloud_composer::CloudItem::printNumPoints () const
{
  //boost::shared_ptr <PointCloud<PointT> > cloud = this->data (ItemDataRole::CLOUD_TEMPLATED).value <boost::shared_ptr <PointCloud<PointT> > > ();
  QVariant variant = this->data (ItemDataRole::CLOUD_TEMPLATED);
  typename PointCloud<PointT>::Ptr cloud;
  if ( variant.canConvert <typename PointCloud<PointT>::Ptr> () )
  {  
    cloud = variant.value <typename PointCloud<PointT>::Ptr> ();
  }
  else
  {
    qWarning () << "Attempted to cast to template type which does not exist in this item!";
    return;
  }

  qDebug () << "CLOUD HAS WIDTH = "<< cloud->width;
  
}



template <typename PointT> pcl::cloud_composer::CloudItem* 
pcl::cloud_composer::CloudItem::createCloudItemFromTemplate (const QString name, typename PointCloud<PointT>::Ptr cloud_ptr)
{
  pcl::PCLPointCloud2::Ptr cloud_blob = boost::make_shared <pcl::PCLPointCloud2> ();
  toPCLPointCloud2 (*cloud_ptr, *cloud_blob);
  CloudItem* cloud_item = new CloudItem ( name, cloud_blob,  Eigen::Vector4f (), Eigen::Quaternionf (), false);
  cloud_item->setData (QVariant::fromValue(cloud_ptr), ItemDataRole::CLOUD_TEMPLATED);
  cloud_item->setPointType<PointT> ();
  return cloud_item;
}


#define PCL_INSTANTIATE_createCloudItemFromTemplate(T) template PCL_EXPORTS pcl::cloud_composer::CloudItem* pcl::cloud_composer::CloudItem::createCloudItemFromTemplate<T>(const QString, typename PointCloud<PointT>::Ptr);

#define PCL_INSTANTIATE_printNumPoints(T) template PCL_EXPORTS void pcl::cloud_composer::CloudItem::getNumPoints<T>();

#endif