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

#ifndef IMPL_ORGANIZED_SEGMENTATION_HPP_
#define IMPL_ORGANIZED_SEGMENTATION_HPP_

#include <pcl/apps/cloud_composer/tools/organized_segmentation.h>
#include <pcl/apps/cloud_composer/impl/cloud_item.hpp>
#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <pcl/memory.h>  // for pcl::make_shared
#include <pcl/point_cloud.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>


template <typename PointT> QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::OrganizedSegmentationTool::performTemplatedAction (const QList <const CloudComposerItem*>& input_data)
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
    typename PointCloud <PointT>::Ptr input_cloud = variant.value <typename PointCloud<PointT>::Ptr> ();
    if ( ! input_cloud->isOrganized ())
    {
      qCritical () << "Organized Segmentation requires an organized cloud!";
      return output;
    }
  }

  int min_inliers = parameter_model_->getProperty ("Min Inliers").toInt ();
  int min_plane_size = parameter_model_->getProperty ("Min Plane Size").toInt ();
  double angular_threshold = parameter_model_->getProperty ("Angular Threshold").toDouble ();
  double distance_threshold = parameter_model_->getProperty ("Distance Threshold").toDouble ();
  double cluster_distance_threshold = parameter_model_->getProperty ("Cluster Dist. Thresh.").toDouble ();
  int min_cluster_size = parameter_model_->getProperty ("Min Cluster Size").toInt ();

  foreach (const CloudComposerItem* input_item, input_data)
  {
    QList <CloudComposerItem*> normals_list = input_item->getChildren (CloudComposerItem::NORMALS_ITEM);
    //Get the normals cloud, we just use the first normals that were found if there are more than one
    pcl::PointCloud<pcl::Normal>::ConstPtr input_normals = normals_list.value(0)->data(ItemDataRole::CLOUD_TEMPLATED).value <pcl::PointCloud<pcl::Normal>::ConstPtr> ();

    QVariant variant = input_item->data (ItemDataRole::CLOUD_TEMPLATED);
    typename PointCloud <PointT>::Ptr input_cloud = variant.value <typename PointCloud<PointT>::Ptr> ();

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (min_inliers);
    mps.setAngularThreshold (pcl::deg2rad (angular_threshold)); // convert to radians
    mps.setDistanceThreshold (distance_threshold);
    mps.setInputNormals (input_normals);
    mps.setInputCloud (input_cloud);
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    auto plane_labels = pcl::make_shared<std::set<std::uint32_t> > ();
    for (std::size_t i = 0; i < label_indices.size (); ++i)
     if (label_indices[i].indices.size () > (std::size_t) min_plane_size)
       plane_labels->insert (i);
    typename PointCloud<PointT>::CloudVectorType clusters;

    typename EuclideanClusterComparator<PointT, pcl::Label>::Ptr euclidean_cluster_comparator(new EuclideanClusterComparator<PointT, pcl::Label>);
    euclidean_cluster_comparator->setInputCloud (input_cloud);
    euclidean_cluster_comparator->setLabels (labels);
    euclidean_cluster_comparator->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator->setDistanceThreshold (cluster_distance_threshold, false);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
    euclidean_segmentation.setInputCloud (input_cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

    pcl::IndicesPtr extracted_indices (new pcl::Indices ());
    for (std::size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () >= (std::size_t) min_cluster_size)
      {
        typename PointCloud<PointT>::Ptr cluster (new PointCloud<PointT>);
        pcl::copyPointCloud (*input_cloud,euclidean_label_indices[i].indices,*cluster);
        qDebug () << "Found cluster with size " << cluster->width;
        QString name = input_item->text () + tr ("- Clstr %1").arg (i);
        CloudItem*  new_cloud_item = CloudItem::createCloudItemFromTemplate<PointT>(name,cluster);
        output.append (new_cloud_item);
        extracted_indices->insert (extracted_indices->end (), euclidean_label_indices[i].indices.begin (), euclidean_label_indices[i].indices.end ());
      }
    }

    for (std::size_t i = 0; i < label_indices.size (); i++)
    {
      if (label_indices[i].indices.size () >= (std::size_t) min_plane_size)
      {
        typename PointCloud<PointT>::Ptr plane (new PointCloud<PointT>);
        pcl::copyPointCloud (*input_cloud,label_indices[i].indices,*plane);
        qDebug () << "Found plane with size " << plane->width;
        QString name = input_item->text () + tr ("- Plane %1").arg (i);
        CloudItem*  new_cloud_item = CloudItem::createCloudItemFromTemplate<PointT>(name,plane);
        output.append (new_cloud_item);
        extracted_indices->insert (extracted_indices->end (), label_indices[i].indices.begin (), label_indices[i].indices.end ());

      }
    }
    typename PointCloud<PointT>::Ptr leftovers (new PointCloud<PointT>);
    if (extracted_indices->empty ())
      pcl::copyPointCloud (*input_cloud,*leftovers);
    else
    {
      pcl::ExtractIndices<PointT> filter;
      filter.setInputCloud (input_cloud);
      filter.setIndices (extracted_indices);
      filter.setNegative (true);
      filter.filter (*leftovers);
    }
    CloudItem*  leftover_cloud_item = CloudItem::createCloudItemFromTemplate<PointT>(input_item->text(),leftovers);
    output.append (leftover_cloud_item);
  }




  return output;

}


#define PCL_INSTANTIATE_performTemplatedAction(T) template void pcl::cloud_composer::OrganizedSegmentationTool::performTemplatedAction<T> (QList <const CloudComposerItem*>);



#endif //IMPL_TRANSFORM_CLOUDS_HPP_
