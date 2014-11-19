/*
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) Alexandru-Eugen Ichim
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ORGANIZED_BOUNDARY_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ORGANIZED_BOUNDARY_H_

#include <pcl/registration/correspondence_rejection.h>


namespace pcl
{
  namespace registration
  {
    /**
      * @brief The CorrespondenceRejectionOrganizedBoundary class implements a simple correspondence rejection measure.
      * For each pair of points in correspondence, it checks whether they are on the boundary of a silhouette. This is
      * done by counting the number of NaN dexels in a window around the points (the threshold and window size can be set
      * by the user).
      * \note Both the source and the target clouds need to be organized, otherwise all the correspondences will be rejected.
      *
      * \author Alexandru E. Ichim
      * \ingroup registration
      */
    class PCL_EXPORTS CorrespondenceRejectionOrganizedBoundary : public CorrespondenceRejector
    {
    public:
      /** @brief Empty constructor. */
      CorrespondenceRejectionOrganizedBoundary ()
        : boundary_nans_threshold_ (8)
        , window_size_ (5)
        , depth_step_threshold_ (0.025f)
        , data_container_ ()
      { }

      void
      getRemainingCorrespondences (const pcl::Correspondences& original_correspondences,
                                   pcl::Correspondences& remaining_correspondences);

      inline void
      setNumberOfBoundaryNaNs (int val)
      { boundary_nans_threshold_ = val; }


      template <typename PointT> inline void
      setInputSource (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
      {
        if (!data_container_)
          data_container_.reset (new pcl::registration::DataContainer<PointT>);
        boost::static_pointer_cast<pcl::registration::DataContainer<PointT> > (data_container_)->setInputSource (cloud);
      }

      template <typename PointT> inline void
      setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
      {
        if (!data_container_)
          data_container_.reset (new pcl::registration::DataContainer<PointT>);
        boost::static_pointer_cast<pcl::registration::DataContainer<PointT> > (data_container_)->setInputTarget (cloud);
      }

      /** \brief See if this rejector requires source points */
      bool
      requiresSourcePoints () const
      { return (true); }

      /** \brief Blob method for setting the source cloud */
      void
      setSourcePoints (pcl::PCLPointCloud2::ConstPtr cloud2)
      { 
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromPCLPointCloud2 (*cloud2, *cloud);
        setInputSource<PointXYZ> (cloud);
      }
      
      /** \brief See if this rejector requires a target cloud */
      bool
      requiresTargetPoints () const
      { return (true); }

      /** \brief Method for setting the target cloud */
      void
      setTargetPoints (pcl::PCLPointCloud2::ConstPtr cloud2)
      { 
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        fromPCLPointCloud2 (*cloud2, *cloud);
        setInputTarget<PointXYZ> (cloud);
      }

      virtual bool
      updateSource (const Eigen::Matrix4d &)
      { return (true); }

    protected:

      /** \brief Apply the rejection algorithm.
        * \param[out] correspondences the set of resultant correspondences.
        */
      inline void
      applyRejection (pcl::Correspondences &correspondences)
      { getRemainingCorrespondences (*input_correspondences_, correspondences); }

      int boundary_nans_threshold_;
      int window_size_;
      float depth_step_threshold_;

      typedef boost::shared_ptr<pcl::registration::DataContainerInterface> DataContainerPtr;
      DataContainerPtr data_container_;
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_organized_boundary.hpp>


#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ORGANIZED_BOUNDARY_H_ */
