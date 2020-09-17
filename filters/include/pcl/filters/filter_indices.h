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
 * $Id: filter_indices.h 4707 2012-02-23 10:34:17Z florentinus $
 *
 */

#pragma once

#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Removes points with x, y, or z equal to NaN (dry run).
    *
    * This function only computes the mapping between the points in the input
    * cloud and the cloud that would result from filtering. It does not
    * actually construct and output the filtered cloud.
    *
    * \note This function does not modify the input point cloud!
    *
    * \param cloud_in the input point cloud
    * \param index the mapping (ordered): filtered_cloud[i] = cloud_in[index[i]]
    *
    * \see removeNaNFromPointCloud
    * \ingroup filters
    */
  template<typename PointT> void
  removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, Indices &index);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b FilterIndices represents the base class for filters that are about binary point removal.
    * <br>
    * All derived classes have to implement the \a filter (PointCloud &output) and the \a filter (Indices &indices) methods.
    * Ideally they also make use of the \a negative_, \a keep_organized_ and \a extract_removed_indices_ systems.
    * The distinguishment between the \a negative_ and \a extract_removed_indices_ systems only makes sense if the class automatically
    * filters non-finite entries in the filtering methods (recommended).
    * \warning PCLPointCloud2 is not currently supported by executors
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT, typename DerivedFilter = void>
  class FilterIndices : public Filter<PointT, FilterIndices<PointT, DerivedFilter>>
  {
    using Self = FilterIndices<PointT, DerivedFilter>;
    using Base = Filter<PointT, Self>;
    friend Base; // Alows the base class to call protected/private methods of this class
                 // i.e. applyFilter overloaded for executor

  public:
      using PointCloud = pcl::PointCloud<PointT>;

      using Ptr = shared_ptr<Self>;
      using ConstPtr = shared_ptr<const Self>;


      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      FilterIndices (bool extract_removed_indices = false) :
          Base (extract_removed_indices),
          negative_ (false),
          keep_organized_ (false),
          user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
      }

      using Base::filter;

      /** \brief Calls the filtering method and returns the filtered point cloud indices.
        * \param[out] indices the resultant filtered point cloud indices
        */
      void
      filter (Indices &indices)
      {
        auto filterCloud = [this](Indices& indices) {
          applyFilter (indices);
        };

        filterImpl(filterCloud, indices);
      }

      /** \brief Calls the filtering method with specified executor and returns the filtered point cloud indices.
        * \param[int] exec the executor to run the filter using
        * \param[out] indices the resultant filtered point cloud indices
        */
      template <typename Executor>
      void
      filter (const Executor &exec, Indices &indices)
      {
        static_assert(pcl::is_invocable_v<
                          decltype(&DerivedFilter::template applyFilter<Executor>),
                          DerivedFilter&,
                          Executor const&,
                          Indices&>,
                      "An executor overload for applyFilter doesn't exist.");

        auto filterCloud = [exec, this](Indices& indices) {
          static_cast<DerivedFilter&>(*this).applyFilter(exec, indices);
        };

        filterImpl(filterCloud, indices);
      }

    /** \brief Set whether the regular conditions for points filtering should apply, or the inverted conditions.
        * \param[in] negative false = normal filter behavior (default), true = inverted behavior.
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get whether the regular conditions for points filtering should apply, or the inverted conditions.
        * \return The value of the internal \a negative_ parameter; false = normal filter behavior (default), true = inverted behavior.
        */
      inline bool
      getNegative () const
      {
        return (negative_);
      }

      /** \brief Set whether the filtered points should be kept and set to the value given through \a setUserFilterValue (default: NaN),
        * or removed from the PointCloud, thus potentially breaking its organized structure.
        * \param[in] keep_organized false = remove points (default), true = redefine points, keep structure.
        */
      inline void
      setKeepOrganized (bool keep_organized)
      {
        keep_organized_ = keep_organized;
      }

      /** \brief Get whether the filtered points should be kept and set to the value given through \a setUserFilterValue (default = NaN),
        * or removed from the PointCloud, thus potentially breaking its organized structure.
        * \return The value of the internal \a keep_organized_ parameter; false = remove points (default), true = redefine points, keep structure.
        */
      inline bool
      getKeepOrganized () const
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to instead of removing them.
        * Used in conjunction with \a setKeepOrganized ().
        * \param[in] value the user given value that the filtered point dimensions should be set to (default = NaN).
        */
      inline void
      setUserFilterValue (float value)
      {
        user_filter_value_ = value;
      }

    protected:
      using Base::initCompute;
      using Base::deinitCompute;
      using Base::input_;
      using Base::removed_indices_;
      using Base::extract_removed_indices_;

    /** \brief False = normal filter behavior (default), true = inverted behavior. */
      bool negative_;

      /** \brief False = remove points (default), true = redefine points, keep structure. */
      bool keep_organized_;

      /** \brief The user given value that the filtered point dimensions should be set to (default = NaN). */
      float user_filter_value_;

    /** \brief implementation of filter method
      *  Added to ensure no code duplication is present between the with and without
      * execuotor filter method
      *
      * \param[int] filterIndices the callable which calls the filter method
      * \param[out] output the resultant filtered point cloud
      */
      template <typename Callable>
      void
      filterImpl (Callable &filterCloud, Indices &indices)
      {
        if (!initCompute ())
          return;

        // Apply the actual filter
        filterCloud(indices);

        deinitCompute ();
      }

      /** \brief Abstract filter method for point cloud indices. */
      virtual void
      applyFilter (Indices &indices) = 0;

      /** \brief Abstract filter method for point cloud. */
      void
      applyFilter (PointCloud &output) override;

    /** \brief overloaded filter method with specified executor.
      *
      * The implementation needs to set output.{points, width, height, is_dense}.
      *
      * \param[int] exec the executor to run the filter using
      * \param[out] output the resultant filtered point cloud
      */
      template <typename Executor>
      void
      applyFilter (const Executor &exec, PointCloud &output);

    /** \brief implementation of applyFilter method
     *  Added to ensure no code duplication is present between the with and without
     * execuotor applyFilter method
     *
     * \param[int] filterIndices the callable which calls the applyFilter method
     * \param[out] output the resultant filtered point cloud
     */
      template <typename Callable>
      void
      applyFilterImpl (Callable &filterIndices, PointCloud &output);
    };

  template <typename PointT>
  using FilterIndicesLegacy = FilterIndices<PointT>;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b FilterIndices represents the base class for filters that are about binary point removal.
    * <br>
    * All derived classes have to implement the \a filter (PointCloud &output) and the \a filter (Indices &indices) methods.
    * Ideally they also make use of the \a negative_, \a keep_organized_ and \a extract_removed_indices_ systems.
    * The distinguishment between the \a negative_ and \a extract_removed_indices_ systems only makes sense if the class automatically
    * filters non-finite entries in the filtering methods (recommended).
    * \warning PCLPointCloud2 is not currently supported by executors
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS FilterIndices<pcl::PCLPointCloud2> : public Filter<pcl::PCLPointCloud2>
  {
    public:
      using PCLPointCloud2 = pcl::PCLPointCloud2;

      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to extract the indices of points being removed (default = false).
        */
      FilterIndices (bool extract_removed_indices = false) :
          Filter<PCLPointCloud2> (extract_removed_indices),
          negative_ (false), 
          keep_organized_ (false), 
          user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
      }

      using Filter<PCLPointCloud2>::filter;

      /** \brief Calls the filtering method and returns the filtered point cloud indices.
        * \param[out] indices the resultant filtered point cloud indices
        */
      void
      filter (Indices &indices);

      /** \brief Set whether the regular conditions for points filtering should apply, or the inverted conditions.
        * \param[in] negative false = normal filter behavior (default), true = inverted behavior.
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get whether the regular conditions for points filtering should apply, or the inverted conditions.
        * \return The value of the internal \a negative_ parameter; false = normal filter behavior (default), true = inverted behavior.
        */
      inline bool
      getNegative () const
      {
        return (negative_);
      }

      /** \brief Set whether the filtered points should be kept and set to the value given through \a setUserFilterValue (default: NaN),
        * or removed from the PointCloud, thus potentially breaking its organized structure.
        * \param[in] keep_organized false = remove points (default), true = redefine points, keep structure.
        */
      inline void
      setKeepOrganized (bool keep_organized)
      {
        keep_organized_ = keep_organized;
      }

      /** \brief Get whether the filtered points should be kept and set to the value given through \a setUserFilterValue (default = NaN),
        * or removed from the PointCloud, thus potentially breaking its organized structure.
        * \return The value of the internal \a keep_organized_ parameter; false = remove points (default), true = redefine points, keep structure.
        */
      inline bool
      getKeepOrganized () const
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to instead of removing them.
        * Used in conjunction with \a setKeepOrganized ().
        * \param[in] value the user given value that the filtered point dimensions should be set to (default = NaN).
        */
      inline void
      setUserFilterValue (float value)
      {
        user_filter_value_ = value;
      }

    protected:

      /** \brief False = normal filter behavior (default), true = inverted behavior. */
      bool negative_;

      /** \brief False = remove points (default), true = redefine points, keep structure. */
      bool keep_organized_;

      /** \brief The user given value that the filtered point dimensions should be set to (default = NaN). */
      float user_filter_value_;

      /** \brief Abstract filter method for point cloud indices. */
      virtual void
      applyFilter (Indices &indices) = 0;

      /** \brief Abstract filter method for point cloud. */
      void
      applyFilter (PCLPointCloud2 &output) override = 0;
  };
}

#include <pcl/filters/impl/filter_indices.hpp>
