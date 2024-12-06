/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 *
 */

#ifndef PCL_SEGMENTATION_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
#define PCL_SEGMENTATION_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_

#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/comparator.h>

namespace pcl
{
  /** \brief OrganizedConnectedComponentSegmentation allows connected
    * components to be found within organized point cloud data, given a
    * comparison function.  Given an input cloud and a comparator, it will
    * output a PointCloud of labels, giving each connected component a unique
    * id, along with a vector of PointIndices corresponding to each component.
    * See OrganizedMultiPlaneSegmentation for an example application.
    *
    * \author Alex Trevor, Suat Gedikli
    */
  template <typename PointT, typename PointLT>
  class OrganizedConnectedComponentSegmentation : public PCLBase<PointT>
  {
    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

    public:
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointLT> PointCloudL;
      typedef typename PointCloudL::Ptr PointCloudLPtr;
      typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

      typedef typename pcl::Comparator<PointT> Comparator;
      typedef typename Comparator::Ptr ComparatorPtr;
      typedef typename Comparator::ConstPtr ComparatorConstPtr;
      
      /** \brief Constructor for OrganizedConnectedComponentSegmentation
        * \param[in] compare A pointer to the comparator to be used for segmentation.  Must be an instance of pcl::Comparator.
        */
      OrganizedConnectedComponentSegmentation (const ComparatorConstPtr& compare)
        : compare_ (compare)
      {
      }

      /** \brief Destructor for OrganizedConnectedComponentSegmentation. */
      virtual
      ~OrganizedConnectedComponentSegmentation ()
      {
      }

      /** \brief Provide a pointer to the comparator to be used for segmentation.
        * \param[in] compare the comparator
        */
      void
      setComparator (const ComparatorConstPtr& compare)
      {
        compare_ = compare;
      }
      
      /** \brief Get the comparator.*/
      ComparatorConstPtr
      getComparator () const { return (compare_); }

      /** \brief Perform the connected component segmentation.
        * \param[out] labels a PointCloud of labels: each connected component will have a unique id.
        * \param[out] label_indices a vector of PointIndices corresponding to each label / component id.
        */
      void
      segment (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
      
      /** \brief Find the boundary points / contour of a connected component
        * \param[in] start_idx the first (lowest) index of the connected component for which a boundary shoudl be returned
        * \param[in] labels the Label cloud produced by segmentation
        * \param[out] boundary_indices the indices of the boundary points for the label corresponding to start_idx
        */
      static void
      findLabeledRegionBoundary (int start_idx, PointCloudLPtr labels, pcl::PointIndices& boundary_indices);      
      

    protected:
      ComparatorConstPtr compare_;
      
      inline unsigned
      findRoot (const std::vector<unsigned>& runs, unsigned index) const
      {
        unsigned idx = index;
        while (runs[idx] != idx)
          idx = runs[idx];

        return (idx);
      }

    private:
      struct Neighbor
      {
        Neighbor (int dx, int dy, int didx)
        : d_x (dx)
        , d_y (dy)
        , d_index (didx)
        {}
        
        int d_x;
        int d_y;
        int d_index; // = dy * width + dx: pre-calculated
      };
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/organized_connected_component_segmentation.hpp>
#endif

#endif //#ifndef PCL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
