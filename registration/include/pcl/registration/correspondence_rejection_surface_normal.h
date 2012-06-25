/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Open Perception, Inc.
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
 *   * Neither the name of Open Perception, Inc. nor the names of its
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
 */
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SURFACE_NORMAL_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SURFACE_NORMAL_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace registration
  {
    /**
      * @b CorrespondenceRejectorSurfaceNormal implements a simple correspondence
      * rejection method based on the angle between the normals at correspondent points.
      *
      * \note If \ref setInputCloud and \ref setInputTarget are given, then the
      * distances between correspondences will be estimated using the given XYZ
      * data, and not read from the set of input correspondences.
      *
      * \author Aravindhan K Krishnan. This code is ported from libpointmatcher (https://github.com/ethz-asl/libpointmatcher)
      * \ingroup registration
      */
    class CorrespondenceRejectorSurfaceNormal : public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:

        /** \brief Empty constructor. */
        CorrespondenceRejectorSurfaceNormal () : threshold_ (1.0), 
                                            data_container_ ()
        {
          rejection_name_ = "CorrespondenceRejectorSurfaceNormal";
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Set the thresholding angle between the normals for correspondence rejection. 
          * \param[in] threshold cosine of the thresholding angle between the normals for rejection
          */
        inline void
        setThreshold (double threshold) { threshold_ = threshold; };

        /** \brief Get the thresholding angle between the normals for correspondence rejection. */
        inline double
        getThreshold () const { return threshold_; };

        /** \brief Initialize the data container object for the point type and the normal type
          */
        template <typename PointT, typename NormalT> inline void 
        initializeDataContainer ()
        {
            data_container_.reset (new DataContainer<PointT, NormalT>);
        }
        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        template <typename PointT> inline void 
        setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &input)
        {
          assert (data_container_ && "Initilize the data container object by calling intializeDataContainer () before using this function");
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputCloud (input);
        }

        /** \brief Provide a target point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] target a cloud containing XYZ data
          */
        template <typename PointT> inline void 
        setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &target)
        {
          assert (data_container_ && "Initilize the data container object by calling intializeDataContainer () before using this function");
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputTarget (target);
        }

        /** \brief Set the normals computed on the input point cloud
          * \param[in] normals the normals computed for the input cloud
          */
        template <typename PointT, typename NormalT> inline void 
        setInputNormals (const typename pcl::PointCloud<NormalT>::ConstPtr &normals)
        {
          assert (data_container_ && "Initilize the data container object by calling intializeDataContainer () before using this function");
          boost::static_pointer_cast<DataContainer<PointT, NormalT> > (data_container_)->setInputNormals (normals);
        }

        /** \brief Set the normals computed on the target point cloud
          * \param[in] normals the normals computed for the input cloud
          */
        template <typename PointT, typename NormalT> inline void 
        setTargetNormals (const typename pcl::PointCloud<NormalT>::ConstPtr &normals)
        {
          assert (data_container_ && "Initilize the data container object by calling intializeDataContainer () before using this function");
          boost::static_pointer_cast<DataContainer<PointT, NormalT> > (data_container_)->setTargetNormals (normals);
        }

        /** \brief Get the normals computed on the input point cloud */
        template <typename NormalT> inline typename pcl::PointCloud<NormalT>::Ptr
        getInputNormals () const { return boost::static_pointer_cast<DataContainer<pcl::PointXYZ, NormalT> > (data_container_)->getInputNormals (); }

        /** \brief Get the normals computed on the target point cloud */
        template <typename NormalT> inline typename pcl::PointCloud<NormalT>::Ptr
        getTargetNormals () const { return boost::static_pointer_cast<DataContainer<pcl::PointXYZ, NormalT> > (data_container_)->getTargetNormals (); }

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }

        /** \brief The median distance threshold between two correspondent points in source <-> target.
          */
        double threshold_;

        typedef boost::shared_ptr<DataContainerInterface> DataContainerPtr;

        /** \brief A pointer to the DataContainer object containing the input and target point clouds */
        DataContainerPtr data_container_;
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_surface_normal.hpp>

#endif
