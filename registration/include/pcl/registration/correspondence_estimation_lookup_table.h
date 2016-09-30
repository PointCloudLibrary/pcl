/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_

#include <algorithm>
#include <string>
#include <vector>

#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b CorrespondenceLookupTableCell represents a match between
      * a CorrespondenceLookupTable cell and the closest point in the reference point cloud
      *
      * \author Carlos M. Costa
      * \ingroup registration
      */
    struct CorrespondenceLookupTableCell
    {
      /** \brief Index of the closest point in the reference point cloud */
      int closest_point_index;

      /** \brief Distance to the closest point */
      float distance_to_closest_point;

      /** \brief Empty constructor.
        * Sets \ref closest_point_index to 0, \ref distance_to_closest_point to FLT_MAX.
        */
      inline CorrespondenceLookupTableCell ()
      : closest_point_index (0), distance_to_closest_point (std::numeric_limits<float>::max ()) {}

      /** \brief Constructor. */
      inline CorrespondenceLookupTableCell (int index, float distance)
      : closest_point_index (index), distance_to_closest_point(distance) {}

      /** \brief Empty destructor. */
      virtual ~CorrespondenceLookupTableCell () {}
    };

    /** \brief @b LookupTable provides fast correspondence estimation
      * by pre-computing the closest point for each cell within a uniform volume grid.
      * It is recommended for 2D point cloud registration or 3D point cloud registration of
      * small volumes (it requires more memory than kd-trees but it is much faster).
      *
      * \author Carlos M. Costa
      * \ingroup registration
      */
    template <typename PointT>
    class CorrespondenceLookupTable
    {
      public:
        /** \brief Empty constructor. */
        CorrespondenceLookupTable ()
          : cell_resolution_ (0.01)
          , cell_resolution_inverse_ (100.0)
          , lookup_table_margin_ (1.0, 1.0, 1.0, 0.0)
          , number_cells_x_ (0)
          , number_cells_y_ (0)
          , number_cells_z_ (0)
          , number_of_queries_on_lookup_table_ (0)
          , number_of_queries_on_search_tree_ (0) {}

        /** \brief Empty destructor. */
        virtual ~CorrespondenceLookupTable () {}

        /** \brief Set the lookup table cell resolution.
          * \param[in] cell_resolution is the lookup table cell size
          */
        inline void
        setCellResolution (double cell_resolution) { cell_resolution_ = cell_resolution; cell_resolution_inverse_ = 1.0 / cell_resolution_; }

        /** \brief Get the lookup table cell size */
        inline double
        getCellResolution () { return (cell_resolution_); }

        /** \brief Set the lookup table margin (in x, y and z axis).
          * \param[in] lookup_table_margin is the extra space that will be filled with extra cells around the data bounding box
          * in order to have pre-computed correspondences available when registering point clouds that are not aligned
          */
        inline void
        setLookupTableMargin (Eigen::Vector4f lookup_table_margin) { lookup_table_margin_ = lookup_table_margin; }

        /** \brief Gets the lookup table margin. */
        inline Eigen::Vector4f
        getLookupTableMargin () { return (lookup_table_margin_); }

        /** \brief Gets the lookup table minimum bounds. */
        inline Eigen::Vector4f
        getMinimumBounds () { return (minimum_bounds_); }

        /** \brief Gets the lookup table maximum bounds. */
        inline Eigen::Vector4f
        getMaximumBounds () { return (maximum_bounds_); }

        /** \brief Gets the number of queries performed on the lookup table */
        inline size_t
        getNumberOfQueriesOnLookupTable () { return (number_of_queries_on_lookup_table_); }

        /** \brief Resets the number of queries performed on the lookup table */
        inline void
        resetNumberOfQueriesOnLookupTable () { number_of_queries_on_lookup_table_ = 0; }

        /** \brief Gets the number of queries performed on the lookup table */
        inline size_t
        getNumberOfQueriesOnSearchTree () { return (number_of_queries_on_search_tree_); }

        /** \brief Resets the number of queries performed on the lookup table */
        inline void
        resetNumberOfQueriesOnSearchTree () { number_of_queries_on_search_tree_ = 0; }

        /**
          * \brief Computes the lookup table minimum and maximum bounds with the additional margin that was set previously.
          * @param pointcloud Point cloud with the data
          * @return True if the computed bounds are valid
          */
        virtual bool
        computeLookupTableBounds (const pcl::PointCloud<PointT>& pointcloud);

        /**
          * \brief Initialize the lookup table using the provided tree.
          * @param tree Search tree with the point data
          * @return True if the lookup table was initialized successfully
          */
        virtual bool
        initLookupTable (const typename pcl::search::Search<PointT>::Ptr& tree);

        /**
         * Gets the pre-computed correspondence associated with the provided query point.
         * @param query_point Coordinates of the query point
         * @param maximum_correspondence_distance_squared The maximum distance squared that a valid correspondence can have
         * @param correspondence The correspondence found for the provided query point. If the query point is outside the pre-computed cells,
         * the search tree will be used for finding the correspondence
         * @return True if it was found a valid correspondence
         */
        bool
        getCorrespondence (const PointT& query_point, double maximum_correspondence_distance_squared, pcl::registration::CorrespondenceLookupTableCell& correspondance);

      protected:
        /** \brief 3 Dimensional array containing the pre-computed correspondences. */
        std::vector<pcl::registration::CorrespondenceLookupTableCell> lookup_table_;

        /** \brief Search tree associated with the lookup table. */
        typename pcl::search::Search<PointT>::Ptr search_tree_;

        /** \brief Resolution of the lookup table. */
        double cell_resolution_;

        /** \brief Inverse of the resolution of the lookup table. */
        double cell_resolution_inverse_;

        /** \brief Margin added to the data bounding box in order to create extra cells in the lookup table surrounding the point cloud data. */
        Eigen::Vector4f lookup_table_margin_;

        /** \brief Lookup table minimum bounds. */
        Eigen::Vector4f minimum_bounds_;

        /** \brief Lookup table maximum bounds. */
        Eigen::Vector4f maximum_bounds_;

        /** \brief Number of cells in the x dimension. */
        size_t number_cells_x_;

        /** \brief Number of cells in the y dimension. */
        size_t number_cells_y_;

        /** \brief Number of cells in the z dimension. */
        size_t number_cells_z_;

        /** \brief Number of queries performed on the lookup table */
        size_t number_of_queries_on_lookup_table_;

        /** \brief Number of queries performed on the search tree (because they were outside the lookup table bounds) */
        size_t number_of_queries_on_search_tree_;
    };

    /** \brief @b CorrespondenceEstimationLookupTable provides fast correspondence estimation
      * by using a lookup table with pre-computed correspondences.
      *
      * \author Carlos M. Costa
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimationLookupTable: public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_reciprocal_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
        using PCLBase<PointSource>::deinitCompute;

        typedef pcl::search::KdTree<PointTarget> KdTree;
        typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::search::KdTree<PointSource> KdTreeReciprocal;
        typedef typename KdTree::Ptr KdTreeReciprocalPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimationLookupTable ()
        {
          corr_name_  = "CorrespondenceEstimationLookupTable";
        }

        /** \brief Empty destructor. */
        virtual ~CorrespondenceEstimationLookupTable () {}

        /** \brief Get the source lookup table. */
        inline CorrespondenceLookupTable<PointSource>&
        getSourceCorrespondencesLookupTable () { return (source_correspondences_lookup_table_); }

        /** \brief Get the target lookup table. */
        inline CorrespondenceLookupTable<PointTarget>&
        getTargetCorrespondencesLookupTable () { return (target_correspondences_lookup_table_); }

        /** \brief Internal computation initialization. */
        bool
        initComputeReciprocal ();

        /** \brief Internal computation initialization for reciprocal correspondences. */
        bool
        initCompute ();

        /** \brief Provide a pointer to the search object used for finding correspondences in
          * the source cloud (used for reciprocal correspondence finding).
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree and the associated lookup table
          * will never be recomputed, regardless of calls to setInputSource.
          */
        virtual void
        setSearchMethodSource (const KdTreeReciprocalPtr &tree,
                               bool force_no_recompute = false)
        {
          CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setSearchMethodSource (tree, force_no_recompute);
          //if (!force_no_recompute)
          //  source_correspondences_lookup_table_.initLookupTable(tree);
        }

        /** \brief Provide a pointer to the search object used for finding correspondences in the target cloud.
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree and the associated lookup table
          * will never be recomputed, regardless of calls to setInputTarget.
          */
        virtual void
        setSearchMethodTarget (const KdTreePtr &tree,
                               bool force_no_recompute = false)
        {
          CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setSearchMethodTarget (tree, force_no_recompute);
          if (!force_no_recompute)
            target_correspondences_lookup_table_.initLookupTable(tree);
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences The found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance Maximum allowed distance between correspondences
          */
        virtual void
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if the Src_i -> Tgt_i
          * correspondence is the same as Tgt_i -> Src_i.
          *
          * \param[out] correspondences The found correspondences (index of query and target point, distance)
          * \param[in] max_distance Maximum allowed distance between correspondences
          */
        virtual void
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ());

        /** \brief Clone and cast to CorrespondenceEstimationBase */
        virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> >
        clone () const
        {
          Ptr copy (new CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }

      protected:
        /** \brief LookupTable containing the pre-computed source correspondences */
        CorrespondenceLookupTable<PointSource> source_correspondences_lookup_table_;

        /** \brief LookupTable containing the pre-computed target correspondences */
        CorrespondenceLookupTable<PointTarget> target_correspondences_lookup_table_;
     };
  }
}

#include <pcl/registration/impl/correspondence_estimation_lookup_table.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_ */
