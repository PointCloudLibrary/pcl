/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: statistical_outlier_removal.h 35433 2011-01-25 01:51:41Z rusu $
 *
 */

#ifndef PCL_FILTERS_STATISTICALOUTLIERREMOVAL_H_
#define PCL_FILTERS_STATISTICALOUTLIERREMOVAL_H_

#include "pcl/point_types.h"
#include "pcl/filters/filter.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/common.h"

namespace pcl
{

  /** \brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. For more
    * information check:
    * <ul>
    * <li> R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
    *      Towards 3D Point Cloud Based Object Maps for Household Environments
    *      Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
    * </ul>
    *
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    */
  template <typename PointT>
  class StatisticalOutlierRemoval: public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    typedef typename pcl::KdTree<PointT> KdTree;
    typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      StatisticalOutlierRemoval () : mean_k_ (2), std_mul_ (0.0), tree_ (), negative_ (false) 
      {
        filter_name_ = "StatisticalOutlierRemoval";
      };

      /** \brief Set the number of points (k) to use for mean distance estimation
        * \param nr_k the number of points to use for mean distance estimation
        */
      inline void setMeanK (int nr_k) { mean_k_ = nr_k; }

      /** \brief Get the number of points to use for mean distance estimation. */
      inline int getMeanK () { return (mean_k_); }

      /** \brief Set the standard deviation multiplier threshold. All points outside the
        * \f[ \mu \pm \sigma \cdot std\_mul \f]
        * will be considered outliers, where \f$ \mu \f$ is the estimated mean,
        * and \f$ \sigma \f$ is the standard deviation.
        * \param std_mul the standard deviation multiplier threshold
        */
      inline void setStddevMulThresh (double std_mul) { std_mul_ = std_mul; }

      /** \brief Get the standard deviation multiplier threshold as set by the user. */
      inline double getStddevMulThresh () { return (std_mul_); }

      /** \brief Set whether the inliers should be returned (true), or the outliers (false).
        * \param negative true if the inliers should be returned, false otherwise
        */
      inline void setNegative (bool negative) { negative_ = negative; }

      /** \brief Get the value of the internal \ref negative_ parameter. If
        * true, all points _except_ the input indices will be returned. 
        */
      inline bool getNegative () { return (negative_); }

    protected:
      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief Standard deviations threshold (i.e., points outside of 
        * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
      double std_mul_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief If true, the outliers will be returned instead of the inliers (default: false). */
      bool negative_;

      /** \brief Apply the filter
        * \param output the resultant point cloud message
        */
      void applyFilter (PointCloud &output);
  };

  /** \brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. For more
    * information check:
    * <ul>
    * <li> R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
    *      Towards 3D Point Cloud Based Object Maps for Household Environments
    *      Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
    * </ul>
    *
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    */
  template <>
  class StatisticalOutlierRemoval<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    typedef pcl::KdTree<pcl::PointXYZ> KdTree;
    typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      StatisticalOutlierRemoval () : mean_k_ (2), std_mul_ (0.0), tree_ (), negative_ (false) 
      {
        filter_name_ = "StatisticalOutlierRemoval";
      };

      /** \brief Set the number of points (k) to use for mean distance estimation
        * \param nr_k the number of points to use for mean distance estimation
        */
      inline void setMeanK (int nr_k) { mean_k_ = nr_k; }

      /** \brief Get the number of points to use for mean distance estimation. */
      inline int getMeanK () { return (mean_k_); }

      /** \brief Set the standard deviation multiplier threshold. All points outside the
        * \f[ \mu \pm \sigma \cdot std\_mul \f]
        * will be considered outliers, where \f$ \mu \f$ is the estimated mean,
        * and \f$ \sigma \f$ is the standard deviation.
        * \param std_mul the standard deviation multiplier threshold
        */
      inline void setStddevMulThresh (double std_mul) { std_mul_ = std_mul; }

      /** \brief Get the standard deviation multiplier threshold as set by the user. */
      inline double getStddevMulThresh () { return (std_mul_); }

      /** \brief Set whether the indices should be returned, or all points _except_ the indices.
        * \param negative true if all points _except_ the input indices will be returned, false otherwise
        */
      inline void setNegative (bool negative) { negative_ = negative; }

      /** \brief Get the value of the internal \ref negative_ parameter. If
        * true, all points _except_ the input indices will be returned. 
        */
      inline bool getNegative () { return (negative_); }

    protected:
      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief Standard deviations threshold (i.e., points outside of 
        * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
      double std_mul_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief If true, the outliers will be returned instead of the inliers (default: false). */
      bool negative_;

      void applyFilter (PointCloud2 &output);
  };
}

#endif  //#ifndef PCL_FILTERS_STATISTICALOUTLIERREMOVAL_H_
