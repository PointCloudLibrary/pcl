/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $id: $
 */

#ifndef PCL_SEEDED_HUE_SEGMENTATION_H_
#define PCL_SEEDED_HUE_SEGMENTATION_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the Euclidean distance between points
    * \param[in] cloud the point cloud message
    * \param[in] tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param[in] tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
    * \param[in] indices_in the cluster containing the seed point indices (as a vector of PointIndices)
    * \param[out] indices_out 
    * \param[in] delta_hue
    * \todo look how to make this templated!
    * \ingroup segmentation
    */
  void 
  seededHueSegmentation (const PointCloud<PointXYZRGB>                           &cloud, 
                         const boost::shared_ptr<search::Search<PointXYZRGB> >   &tree, 
                         float                                                   tolerance, 
                         PointIndices                                            &indices_in, 
                         PointIndices                                            &indices_out, 
                         float                                                   delta_hue = 0.0);

  /** \brief Decompose a region of space into clusters based on the Euclidean distance between points
    * \param[in] cloud the point cloud message
    * \param[in] tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param[in] tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
    * \param[in] indices_in the cluster containing the seed point indices (as a vector of PointIndices)
    * \param[out] indices_out 
    * \param[in] delta_hue
    * \todo look how to make this templated!
    * \ingroup segmentation
    */
  void 
  seededHueSegmentation (const PointCloud<PointXYZRGB>                           &cloud, 
                         const boost::shared_ptr<search::Search<PointXYZRGBL> >  &tree, 
                         float                                                   tolerance, 
                         PointIndices                                            &indices_in, 
                         PointIndices                                            &indices_out, 
                         float                                                   delta_hue = 0.0);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief SeededHueSegmentation 
    * \author Koen Buys
    * \ingroup segmentation
    */
  class SeededHueSegmentation: public PCLBase<PointXYZRGB>
  {
    typedef PCLBase<PointXYZRGB> BasePCLBase;

    public:
      typedef pcl::PointCloud<PointXYZRGB> PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      typedef pcl::search::Search<PointXYZRGB> KdTree;
      typedef pcl::search::Search<PointXYZRGB>::Ptr KdTreePtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      SeededHueSegmentation () : tree_ (), cluster_tolerance_ (0), delta_hue_ (0.0)
      {};

      /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void 
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr 
      getSearchMethod () const { return (tree_); }

      /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
        * \param[in] tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
        */
      inline void 
      setClusterTolerance (double tolerance) { cluster_tolerance_ = tolerance; }

      /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space. */
      inline double 
      getClusterTolerance () const { return (cluster_tolerance_); }

      /** \brief Set the tollerance on the hue
        * \param[in] delta_hue the new delta hue
        */
      inline void 
      setDeltaHue (float delta_hue) { delta_hue_ = delta_hue; }

      /** \brief Get the tolerance on the hue */
      inline float 
      getDeltaHue () const { return (delta_hue_); }

      /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
        * \param[in] indices_in
        * \param[out] indices_out
        */
      void 
      segment (PointIndices &indices_in, PointIndices &indices_out);

    protected:
      // Members derived from the base class
      using BasePCLBase::input_;
      using BasePCLBase::indices_;
      using BasePCLBase::initCompute;
      using BasePCLBase::deinitCompute;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
      double cluster_tolerance_;

      /** \brief The allowed difference on the hue*/
      float delta_hue_;

      /** \brief Class getName method. */
      virtual std::string getClassName () const { return ("seededHueSegmentation"); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/seeded_hue_segmentation.hpp>
#endif

#endif  //#ifndef PCL_SEEDED_HUE_SEGMENTATION_H_
