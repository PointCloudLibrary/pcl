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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_VAR_TRIMMED_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_VAR_TRIMMED_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

#include <vector>

namespace pcl
{
  namespace registration
  {
    /**
      * @b CorrespondenceRejectoVarTrimmed implements a simple correspondence
      * rejection method by considering as inliers a certain percentage of correspondences 
      * with the least distances. The percentage of inliers is computed internally as mentioned
      * in the paper 'Outlier Robust ICP for minimizing Fractional RMSD, J. M. Philips et al'
      *
      * \note If \ref setInputCloud and \ref setInputTarget are given, then the
      * distances between correspondences will be estimated using the given XYZ
      * data, and not read from the set of input correspondences.
      *
      * \author Aravindhan K Krishnan. This code is ported from libpointmatcher (https://github.com/ethz-asl/libpointmatcher)
      * \ingroup registration
      */
    class PCL_EXPORTS CorrespondenceRejectorVarTrimmed: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:
        typedef boost::shared_ptr<CorrespondenceRejectorVarTrimmed> Ptr;
        typedef boost::shared_ptr<const CorrespondenceRejectorVarTrimmed> ConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceRejectorVarTrimmed () : 
          trimmed_distance_ (0), 
          factor_ (),
          min_ratio_ (0.05),
          max_ratio_ (0.95),
          lambda_ (0.95),
          data_container_ ()
        {
          rejection_name_ = "CorrespondenceRejectorVarTrimmed";
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Get the trimmed distance used for thresholding in correspondence rejection. */
        inline double
        getTrimmedDistance () const { return trimmed_distance_; };

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        template <typename PointT> inline void 
        setInputSource (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
        {
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputSource (cloud);
        }

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        template <typename PointT> inline void 
        setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
        {
          PCL_WARN ("[pcl::registration::%s::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.\n", getClassName ().c_str ());
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputSource (cloud);
        }

        /** \brief Provide a target point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] target a cloud containing XYZ data
          */
        template <typename PointT> inline void 
        setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &target)
        {
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputTarget (target);
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

        /** \brief Provide a pointer to the search object used to find correspondences in
          * the target cloud.
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputTarget. Only use if you are 
          * confident that the tree will be set correctly.
          */
        template <typename PointT> inline void
        setSearchMethodTarget (const boost::shared_ptr<pcl::search::KdTree<PointT> > &tree, 
                               bool force_no_recompute = false) 
        { 
          boost::static_pointer_cast< DataContainer<PointT> > 
            (data_container_)->setSearchMethodTarget (tree, force_no_recompute );
        }

        /** \brief Get the computed inlier ratio used for thresholding in correspondence rejection. */
        inline double
        getTrimFactor () const { return factor_; }

        /** brief set the minimum overlap ratio
          * \param[in] ratio the overlap ratio [0..1]
          */
        inline void
        setMinRatio (double ratio) { min_ratio_ = ratio; }

        /** brief get the minimum overlap ratio
          */
        inline double
        getMinRatio () const { return min_ratio_; }

        /** brief set the maximum overlap ratio
          * \param[in] ratio the overlap ratio [0..1]
          */
        inline void
        setMaxRatio (double ratio) { max_ratio_ = ratio; }

        /** brief get the maximum overlap ratio
          */
        inline double
        getMaxRatio () const { return max_ratio_; }

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }

        /** \brief The inlier distance threshold (based on the computed trim factor) between two correspondent points in source <-> target.
          */
        double trimmed_distance_;

        /** \brief The factor for correspondence rejection. Only factor times the total points sorted based on 
         *  the correspondence distances will be considered as inliers. Remaining points are rejected. This factor is
         *  computed internally 
         */
        double factor_;

        /** \brief The minimum overlap ratio between the input and target clouds
         */
        double min_ratio_;

        /** \brief The maximum overlap ratio between the input and target clouds
         */
        double max_ratio_;

       /** \brief part of the term that balances the root mean square difference. This is an internal parameter
         */
        double lambda_;

        typedef boost::shared_ptr<DataContainerInterface> DataContainerPtr;

        /** \brief A pointer to the DataContainer object containing the input and target point clouds */
        DataContainerPtr data_container_;

      private:

        /** \brief finds the optimal inlier ratio. This is based on the paper 'Outlier Robust ICP for minimizing Fractional RMSD, J. M. Philips et al'
         */
        inline float optimizeInlierRatio (std::vector <double> &dists);
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_var_trimmed.hpp>

#endif    // PCL_REGISTRATION_CORRESPONDENCE_REJECTION_VAR_TRIMMED_H_ 
