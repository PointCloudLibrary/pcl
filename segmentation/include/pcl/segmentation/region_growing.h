/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Sergey Ushakov
 * Email  : mine_all_mine@bk.ru
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief
    * Implements the well known Region Growing algorithm used for segmentation.
    * Description can be found in the article
    * "Segmentation of point clouds using smoothness constraint"
    * by T. Rabbani, F. A. van den Heuvel, G. Vosselman.
    * In addition to residual test, the possibility to test curvature is added.
    * \ingroup segmentation
    */
  template <typename PointT, typename NormalT>
  class PCL_EXPORTS RegionGrowing : public pcl::PCLBase<PointT>
  {
    public:

      using KdTree = pcl::search::Search<PointT>;
      using KdTreePtr = typename KdTree::Ptr;
      using Normal = pcl::PointCloud<NormalT>;
      using NormalPtr = typename Normal::Ptr;
      using PointCloud = pcl::PointCloud<PointT>;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      RegionGrowing ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */

      ~RegionGrowing () override;

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      pcl::uindex_t
      getMinClusterSize ();

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMinClusterSize (pcl::uindex_t min_cluster_size);

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      pcl::uindex_t
      getMaxClusterSize ();

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMaxClusterSize (pcl::uindex_t max_cluster_size);

      /** \brief Returns the flag value. This flag signalizes which mode of algorithm will be used.
        * If it is set to true than it will work as said in the article. This means that
        * it will be testing the angle between normal of the current point and it's neighbours normal.
        * Otherwise, it will be testing the angle between normal of the current point
        * and normal of the initial point that was chosen for growing new segment.
        */
      bool
      getSmoothModeFlag () const;

      /** \brief This function allows to turn on/off the smoothness constraint.
        * \param[in] value new mode value, if set to true then the smooth version will be used.
        */
      void
      setSmoothModeFlag (bool value);

      /** \brief Returns the flag that signalize if the curvature test is turned on/off. */
      bool
      getCurvatureTestFlag () const;

      /** \brief Allows to turn on/off the curvature test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning curvature test off
        * then residual test will be turned on automatically.
        * \param[in] value new value for curvature test. If set to true then the test will be turned on
        */
      virtual void
      setCurvatureTestFlag (bool value);

      /** \brief Returns the flag that signalize if the residual test is turned on/off. */
      bool
      getResidualTestFlag () const;

      /** \brief
        * Allows to turn on/off the residual test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning residual test off
        * then curvature test will be turned on automatically.
        * \param[in] value new value for residual test. If set to true then the test will be turned on
        */
      virtual void
      setResidualTestFlag (bool value);

      /** \brief Returns smoothness threshold. */
      float
      getSmoothnessThreshold () const;

      /** \brief Allows to set smoothness threshold used for testing the points.
        * \param[in] theta new threshold value for the angle between normals
        */
      void
      setSmoothnessThreshold (float theta);

      /** \brief Returns residual threshold. */
      float
      getResidualThreshold () const;

      /** \brief Allows to set residual threshold used for testing the points.
        * \param[in] residual new threshold value for residual testing
        */
      void
      setResidualThreshold (float residual);

      /** \brief Returns curvature threshold. */
      float
      getCurvatureThreshold () const;

      /** \brief Allows to set curvature threshold used for testing the points.
        * \param[in] curvature new threshold value for curvature testing
        */
      void
      setCurvatureThreshold (float curvature);

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbours () const;

      /** \brief Allows to set the number of neighbours. For more information check the article.
        * \param[in] neighbour_number number of neighbours to use
        */
      void
      setNumberOfNeighbours (unsigned int neighbour_number);

      /** \brief Returns the pointer to the search method that is used for KNN. */
      KdTreePtr
      getSearchMethod () const;

      /** \brief Allows to set search method that will be used for finding KNN.
        * \param[in] tree pointer to a KdTree
        */
      void
      setSearchMethod (const KdTreePtr& tree);

      /** \brief Returns normals. */
      NormalPtr
      getInputNormals () const;

      /** \brief This method sets the normals. They are needed for the algorithm, so if
        * no normals will be set, the algorithm would not be able to segment the points.
        * \param[in] norm normals that will be used in the algorithm
        */
      void
      setInputNormals (const NormalPtr& norm);

      /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
      virtual void
      extract (std::vector <pcl::PointIndices>& clusters);

      /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
        * \param[in] index index of the initial point which will be the seed for growing a segment.
        * \param[out] cluster cluster to which the point belongs.
        */
      virtual void
      getSegmentFromPoint (pcl::index_t index, pcl::PointIndices& cluster);

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud ();

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      getColoredCloudRGBA ();

    protected:

      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This method finds KNN for each point and saves them to the array
        * because the algorithm needs to find KNN a few times.
        */
      virtual void
      findPointNeighbours ();

      /** \brief This function implements the algorithm described in the article
        * "Segmentation of point clouds using smoothness constraint"
        * by T. Rabbani, F. A. van den Heuvel, G. Vosselman.
        */
      void
      applySmoothRegionGrowingAlgorithm ();

      /** \brief This method grows a segment for the given seed point. And returns the number of its points.
        * \param[in] initial_seed index of the point that will serve as the seed point
        * \param[in] segment_number indicates which number this segment will have
        */
      int
      growRegion (int initial_seed, int segment_number);

      /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
        * If so, then it returns true. It also checks if this point can serve as the seed.
        * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
        * \param[in] point index of the current seed point
        * \param[in] nghbr index of the point that is neighbour of the current seed
        * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
        */
      virtual bool
      validatePoint (pcl::index_t initial_seed, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const;

      /** \brief This function simply assembles the regions from list of point labels.
        * Each cluster is an array of point indices.
        */
      void
      assembleRegions ();

    protected:

      /** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
      pcl::uindex_t min_pts_per_cluster_;

      /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
      pcl::uindex_t max_pts_per_cluster_;

      /** \brief Flag that signalizes if the smoothness constraint will be used. */
      bool smooth_mode_flag_;

      /** \brief If set to true then curvature test will be done during segmentation. */
      bool curvature_flag_;

      /** \brief If set to true then residual test will be done during segmentation. */
      bool residual_flag_;

      /** \brief Threshold used for testing the smoothness between points. */
      float theta_threshold_;

      /** \brief Threshold used in residual test. */
      float residual_threshold_;

      /** \brief Threshold used in curvature test. */
      float curvature_threshold_;

      /** \brief Number of neighbours to find. */
      unsigned int neighbour_number_;

      /** \brief Search method that will be used for KNN. */
      KdTreePtr search_;

      /** \brief Contains normals of the points that will be segmented. */
      NormalPtr normals_;

      /** \brief Contains neighbours of each point. */
      std::vector<pcl::Indices> point_neighbours_;

      /** \brief Point labels that tells to which segment each point belongs. */
      std::vector<int> point_labels_;

      /** \brief If set to true then normal/smoothness test will be done during segmentation.
        * It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
        * for smoothness in the child class RegionGrowingRGB.*/
      bool normal_flag_;

      /** \brief Tells how much points each segment contains. Used for reserving memory. */
      std::vector<pcl::uindex_t> num_pts_in_segment_;

      /** \brief After the segmentation it will contain the segments. */
      std::vector <pcl::PointIndices> clusters_;

      /** \brief Stores the number of segments. */
      int number_of_segments_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief This function is used as a comparator for sorting. */
  inline bool
  comparePair (std::pair<float, int> i, std::pair<float, int> j)
  {
    return (i.first < j.first);
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/region_growing.hpp>
#endif
