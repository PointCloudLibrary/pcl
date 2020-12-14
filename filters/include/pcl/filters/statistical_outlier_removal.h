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
 * $Id$
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/search/search.h> // for Search

namespace pcl
{
  /** \brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data.
    * \details The algorithm iterates through the entire input twice:
    * During the first iteration it will compute the average distance that each point has to its nearest k neighbors.
    * The value of k can be set using setMeanK().
    * Next, the mean and standard deviation of all these distances are computed in order to determine a distance threshold.
    * The distance threshold will be equal to: mean + stddev_mult * stddev.
    * The multiplier for the standard deviation can be set using setStddevMulThresh().
    * During the next iteration the points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
    * <br>
    * The neighbors found for each query point will be found amongst ALL points of setInputCloud(), not just those indexed by setIndices().
    * The setIndices() method only indexes the points that will be iterated through as search query points.
    * <br><br>
    * For more information:
    *   - R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
    *     Towards 3D Point Cloud Based Object Maps for Household Environments
    *     Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
    * <br><br>
    * Usage example:
    * \code
    * pcl::StatisticalOutlierRemoval<PointType> sorfilter (true); // Initializing with true will allow us to extract the removed indices
    * sorfilter.setInputCloud (cloud_in);
    * sorfilter.setMeanK (8);
    * sorfilter.setStddevMulThresh (1.0);
    * sorfilter.filter (*cloud_out);
    * // The resulting cloud_out contains all points of cloud_in that have an average distance to their 8 nearest neighbors that is below the computed threshold
    * // Using a standard deviation multiplier of 1.0 and assuming the average distances are normally distributed there is a 84.1% chance that a point will be an inlier
    * indices_rem = sorfilter.getRemovedIndices ();
    * // The indices_rem array indexes all points of cloud_in that are outliers
    * \endcode
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class StatisticalOutlierRemoval : public FilterIndices<PointT>
  {
    protected:
      using PointCloud = typename FilterIndices<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using SearcherPtr = typename pcl::search::Search<PointT>::Ptr;

    public:

      using Ptr = shared_ptr<StatisticalOutlierRemoval<PointT> >;
      using ConstPtr = shared_ptr<const StatisticalOutlierRemoval<PointT> >;


      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      StatisticalOutlierRemoval (bool extract_removed_indices = false) :
        FilterIndices<PointT> (extract_removed_indices),
        searcher_ (),
        mean_k_ (1),
        std_mul_ (0.0)
      {
        filter_name_ = "StatisticalOutlierRemoval";
      }

      /** \brief Set the number of nearest neighbors to use for mean distance estimation.
        * \param[in] nr_k The number of points to use for mean distance estimation.
        */
      inline void
      setMeanK (int nr_k)
      {
        mean_k_ = nr_k;
      }

      /** \brief Get the number of nearest neighbors to use for mean distance estimation.
        * \return The number of points to use for mean distance estimation.
        */
      inline int
      getMeanK ()
      {
        return (mean_k_);
      }

      /** \brief Set the standard deviation multiplier for the distance threshold calculation.
        * \details The distance threshold will be equal to: mean + stddev_mult * stddev.
        * Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
        * \param[in] stddev_mult The standard deviation multiplier.
        */
      inline void
      setStddevMulThresh (double stddev_mult)
      {
        std_mul_ = stddev_mult;
      }

      /** \brief Get the standard deviation multiplier for the distance threshold calculation.
        * \details The distance threshold will be equal to: mean + stddev_mult * stddev.
        * Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
        */
      inline double
      getStddevMulThresh ()
      {
        return (std_mul_);
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (Indices &indices);

    private:
      /** \brief A pointer to the spatial search object. */
      SearcherPtr searcher_;

      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief Standard deviations threshold (i.e., points outside of 
        * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
      double std_mul_;
  };

  /** \brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. For more
    * information check:
    *   - R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
    *     Towards 3D Point Cloud Based Object Maps for Household Environments
    *     Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
    *
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS StatisticalOutlierRemoval<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    using FilterIndices<pcl::PCLPointCloud2>::filter_name_;
    using FilterIndices<pcl::PCLPointCloud2>::getClassName;

    using FilterIndices<pcl::PCLPointCloud2>::removed_indices_;
    using FilterIndices<pcl::PCLPointCloud2>::extract_removed_indices_;

    using KdTree = pcl::search::Search<pcl::PointXYZ>;
    using KdTreePtr = pcl::search::Search<pcl::PointXYZ>::Ptr;

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    public:
      /** \brief Empty constructor. */
      StatisticalOutlierRemoval (bool extract_removed_indices = false) :
        FilterIndices<pcl::PCLPointCloud2>::FilterIndices (extract_removed_indices), mean_k_ (2),
        std_mul_ (0.0)
      {
        filter_name_ = "StatisticalOutlierRemoval";
      }

      /** \brief Set the number of points (k) to use for mean distance estimation
        * \param nr_k the number of points to use for mean distance estimation
        */
      inline void
      setMeanK (int nr_k)
      {
        mean_k_ = nr_k;
      }

      /** \brief Get the number of points to use for mean distance estimation. */
      inline int
      getMeanK ()
      {
        return (mean_k_);
      }

      /** \brief Set the standard deviation multiplier threshold. All points outside the
        * \f[ \mu \pm \sigma \cdot std\_mul \f]
        * will be considered outliers, where \f$ \mu \f$ is the estimated mean,
        * and \f$ \sigma \f$ is the standard deviation.
        * \param std_mul the standard deviation multiplier threshold
        */
      inline void
      setStddevMulThresh (double std_mul)
      {
        std_mul_ = std_mul;
      }

      /** \brief Get the standard deviation multiplier threshold as set by the user. */
      inline double
      getStddevMulThresh ()
      {
        return (std_mul_);
      }

    protected:
      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief Standard deviations threshold (i.e., points outside of 
        * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). 
        */
      double std_mul_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      void
      applyFilter (Indices &indices) override;

      void
      applyFilter (PCLPointCloud2 &output) override;

      /**
       * \brief Compute the statistical values used in both applyFilter methods.
       *
       * This method tries to avoid duplicate code.
       */
      virtual void
      generateStatistics (double& mean, double& variance, double& stddev, std::vector<float>& distances);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#endif
