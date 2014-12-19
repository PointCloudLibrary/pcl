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
 */

#ifndef PCL_SEGMENTATION_CONDITIONAL_EUCLIDEAN_CLUSTERING_H_
#define PCL_SEGMENTATION_CONDITIONAL_EUCLIDEAN_CLUSTERING_H_

#include <boost/function.hpp>

#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  typedef std::vector<pcl::PointIndices> IndicesClusters;
  typedef boost::shared_ptr<std::vector<pcl::PointIndices> > IndicesClustersPtr;

  /** \brief @b ConditionalEuclideanClustering performs segmentation based on Euclidean distance and a user-defined clustering condition.
    * \details The condition that need to hold is currently passed using a function pointer.
    * For more information check the documentation of setConditionFunction() or the usage example below:
    * \code
    * bool
    * enforceIntensitySimilarity (const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
    * {
    *   if (fabs (point_a.intensity - point_b.intensity) < 0.1f)
    *     return (true);
    *   else
    *     return (false);
    * }
    * // ...
    * // Somewhere down to the main code
    * // ...
    * pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
    * cec.setInputCloud (cloud_in);
    * cec.setConditionFunction (&enforceIntensitySimilarity);
    * // Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
    * cec.setClusterTolerance (0.09f);
    * // Size constraints for the clusters:
    * cec.setMinClusterSize (5);
    * cec.setMaxClusterSize (30);
    * // The resulting clusters (an array of pointindices):
    * cec.segment (*clusters);
    * // The clusters that are too small or too large in size can also be extracted separately:
    * cec.getRemovedClusters (small_clusters, large_clusters);
    * \endcode
    * \author Frits Florentinus
    * \ingroup segmentation
    */
  template<typename PointT>
  class ConditionalEuclideanClustering : public PCLBase<PointT>
  {
    protected:
      typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::deinitCompute;

    public:
      /** \brief Constructor.
        * \param[in] extract_removed_clusters Set to true if you want to be able to extract the clusters that are too large or too small (default = false)
        */
      ConditionalEuclideanClustering (bool extract_removed_clusters = false) :
          searcher_ (),
          condition_function_ (),
          cluster_tolerance_ (0.0f),
          min_cluster_size_ (1),
          max_cluster_size_ (std::numeric_limits<int>::max ()),
          extract_removed_clusters_ (extract_removed_clusters),
          small_clusters_ (new pcl::IndicesClusters),
          large_clusters_ (new pcl::IndicesClusters)
      {
      }

      /** \brief Set the condition that needs to hold for neighboring points to be considered part of the same cluster.
        * \details Any two points within a certain distance from one another will need to evaluate this condition in order to be made part of the same cluster.
        * The distance can be set using setClusterTolerance().
        * <br>
        * Note that for a point to be part of a cluster, the condition only needs to hold for at least 1 point pair.
        * To clarify, the following statement is false:
        * Any two points within a cluster always evaluate this condition function to true.
        * <br><br>
        * The input arguments of the condition function are:
        * <ul>
        *  <li>PointT The first point of the point pair</li>
        *  <li>PointT The second point of the point pair</li>
        *  <li>float The squared distance between the points</li>
        * </ul>
        * The output argument is a boolean, returning true will merge the second point into the cluster of the first point.
        * \param[in] condition_function The condition function that needs to hold for clustering
        */
      inline void
      setConditionFunction (bool (*condition_function) (const PointT&, const PointT&, float)) 
      {
        condition_function_ = condition_function;
      }

      /** \brief Set the condition that needs to hold for neighboring points to be considered part of the same cluster.
        * This is an overloaded function provided for convenience. See the documentation for setConditionFunction(). */
      inline void
      setConditionFunction (boost::function<bool (const PointT&, const PointT&, float)> condition_function)
      {
        condition_function_ = condition_function;
      }

      /** \brief Set the spatial tolerance for new cluster candidates.
        * \details Any two points within this distance from one another will need to evaluate a certain condition in order to be made part of the same cluster.
        * The condition can be set using setConditionFunction().
        * \param[in] cluster_tolerance The distance to scan for cluster candidates (default = 0.0)
        */
      inline void
      setClusterTolerance (float cluster_tolerance)
      {
        cluster_tolerance_ = cluster_tolerance;
      }

      /** \brief Get the spatial tolerance for new cluster candidates.*/
      inline float
      getClusterTolerance ()
      {
        return (cluster_tolerance_);
      }

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] min_cluster_size The minimum cluster size (default = 1)
        */
      inline void
      setMinClusterSize (int min_cluster_size)
      {
        min_cluster_size_ = min_cluster_size;
      }

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid.*/
      inline int
      getMinClusterSize ()
      {
        return (min_cluster_size_);
      }

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] max_cluster_size The maximum cluster size (default = unlimited)
        */
      inline void
      setMaxClusterSize (int max_cluster_size)
      {
        max_cluster_size_ = max_cluster_size;
      }

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid.*/
      inline int
      getMaxClusterSize ()
      {
        return (max_cluster_size_);
      }

      /** \brief Segment the input into separate clusters.
        * \details The input can be set using setInputCloud() and setIndices().
        * <br>
        * The size constraints for the resulting clusters can be set using setMinClusterSize() and setMaxClusterSize().
        * <br>
        * The region growing parameters can be set using setConditionFunction() and setClusterTolerance().
        * <br>
        * \param[out] clusters The resultant set of indices, indexing the points of the input cloud that correspond to the clusters
        */
      void
      segment (IndicesClusters &clusters);

      /** \brief Get the clusters that are invalidated due to size constraints.
        * \note The constructor of this class needs to be initialized with true, and the segment method needs to have been called prior to using this method.
        * \param[out] small_clusters The resultant clusters that contain less than min_cluster_size points
        * \param[out] large_clusters The resultant clusters that contain more than max_cluster_size points
        */
      inline void
      getRemovedClusters (IndicesClustersPtr &small_clusters, IndicesClustersPtr &large_clusters)
      {
        if (!extract_removed_clusters_)
        {
          PCL_WARN("[pcl::ConditionalEuclideanClustering::getRemovedClusters] You need to set extract_removed_clusters to true (in this class' constructor) if you want to use this functionality.\n");
          return;
        }
        small_clusters = small_clusters_;
        large_clusters = large_clusters_;
      }

    private:
      /** \brief A pointer to the spatial search object */
      SearcherPtr searcher_;

      /** \brief The condition function that needs to hold for clustering */
      boost::function<bool (const PointT&, const PointT&, float)> condition_function_;

      /** \brief The distance to scan for cluster candidates (default = 0.0) */
      float cluster_tolerance_;

      /** \brief The minimum cluster size (default = 1) */
      int min_cluster_size_;

      /** \brief The maximum cluster size (default = unlimited) */
      int max_cluster_size_;

      /** \brief Set to true if you want to be able to extract the clusters that are too large or too small (default = false) */
      bool extract_removed_clusters_;

      /** \brief The resultant clusters that contain less than min_cluster_size points */
      pcl::IndicesClustersPtr small_clusters_;

      /** \brief The resultant clusters that contain more than max_cluster_size points */
      pcl::IndicesClustersPtr large_clusters_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#endif

#endif  // PCL_SEGMENTATION_CONDITIONAL_EUCLIDEAN_CLUSTERING_H_

