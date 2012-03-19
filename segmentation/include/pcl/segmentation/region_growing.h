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
 * Author : Sergey Ushakov
 * Email  : mine_all_mine@bk.ru
 *
 */

#ifndef PCL_REGION_GROWING_H_
#define PCL_REGION_GROWING_H_

#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <math.h>
#include <time.h>

namespace pcl
{
  /** \brief
    * Implements the well known Region Growing algorithm used for segmentation.
    * Description can be found in the article
    * "Segmentation of point clouds using smoothness constraint"
    * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
    * In addition to residual test, the possibility to test curvature is added.
    */
  template <typename PointT>
  class PCL_EXPORTS RegionGrowing
  {
    public:

      /** \brief Constructor that sets default values for member variables. */
      RegionGrowing ();

      /** \brief Returns the flag that signalize if the curvature test is turned on/off. */
      bool
      getCurvatureTestFlag () const;

      /** \brief Returns the flag that signalize if the residual test is turned on/off. */
      bool
      getResidualTestFlag () const;

      /** \brief Returns the flag value. This flag signalizes which mode of algorithm will be used.
        * If it is set to true than it will work as said in the article. This means that
        * it will be testing the angle between normal of the current point and it's neighbours normal.
        * Otherwise, it will be testing the angle between normal of the current point
        * and normal of the initial point that was chosen for growing new segment.
        */
      bool
      getSmoothModeFlag () const;

      /** \brief Returns smoothness threshold. */
      float
      getSmoothnessThreshold () const;

      /** \brief Returns residual threshold. */
      float
      getResidualThreshold () const;

      /** \brief Returns curvature threshold. */
      float
      getCurvatureThreshold () const;

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbours () const;

      /** \brief Returns the pointer to the search method that is used for KNN. */
      typename pcl::search::Search<PointT>::Ptr
      getNeighbourSearchMethod () const;

      /** \brief Returns normals. */
      pcl::PointCloud<pcl::Normal>::Ptr
      getNormals () const;

      /** \brief Returns the cloud that was passed through setCloud(). */
      typename pcl::PointCloud<PointT>::Ptr
      getCloud () const;

      /** \brief Returns list of segments. Each segment is a list of indices of points. */
      std::vector<std::vector<int> >
      getSegments () const;

      /** \brief This function allows to turn on/off the smoothness constraint.
        * \param[in] value new mode value, if set to true then the smooth version will be used.
        */
      void
      setSmoothMode (bool value);

      /** \brief Allows to set smoothness threshold used for testing the points.
        * \param[in] theta new threshold value for the angle between normals
        */
      void
      setSmoothnessThreshold (float theta);

      /** \brief Allows to set residual threshold used for testing the points.
        * \param[in] residual new threshold value for residual testing
        */
      void
      setResidualThreshold (float residual);

      /** \brief Allows to set curvature threshold used for testing the points.
        * \param[in] curvature new threshold value for curvature testing
        */
      void
      setCurvatureThreshold (float curvature);

      /** \brief Allows to set the number of neighbours. For more information check the article.
        * \param[in] neighbour_number number of neighbours to use
        */
      void
      setNumberOfNeighbours (unsigned int neighbour_number);

      /** \brief Allows to set search method that will be used for finding KNN.
        * \param[in] search search method to use
        */
      void
      setNeighbourSearchMethod (typename pcl::search::Search<PointT>::Ptr search);

      /** \brief This method sets the normals. They are needed for the algorithm, so if
        * no normals will be set, the algorithm would not be able to segment the points.
        * \param[in] normals normals that will be used in the algorithm
        */
      void
      setNormals (pcl::PointCloud<pcl::Normal>::Ptr normals);

      /** \brief This method sets the cloud that must be segmented.
        * \param[in] input_cloud point cloud that must be segmented
        */
      void
      setCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud);

    public:

      /** \brief This method simply launches the segmentation algorithm */
      virtual unsigned int
      segmentPoints ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */
      virtual
      ~RegionGrowing ();

      /** \brief Allows to turn on/off the curvature test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning curvature test off
        * then residual test will be turned on automatically.
        * \param[in] value new value for curvature test. If set to true then the test will be turned on
        */
      virtual void
      setCurvatureTest (bool value);

      /** \brief
        * Allows to turn on/off the residual test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning residual test off
        * then curvature test will be turned on automatically.
        * \param[in] value new value for residual test. If set to true then the test will be turned on
        */
      virtual void
      setResidualTest (bool value);

      /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
        * \param[in] point initial point which will be the seed for growing a segment.
        */
      virtual std::vector<int>
      getSegmentFromPoint (const PointT &point);

      /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
        * \param[in] index index of the initial point which will be the seed for growing a segment.
        */
      virtual std::vector<int>
      getSegmentFromPoint (int index);

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG).
        */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud ();

    protected:

      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This function implements the algorithm described in the article
        * "Segmentation of point clouds using smoothness constraint"
        * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
        */
      unsigned int
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
      validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;

      /** \brief This function simply assembles the regions from list of point labels. */
      void
      assembleRegions ();

      /** \brief This method finds KNN for each point and saves them to the array
        * because the algorithm needs to find KNN a few times.
        */
      virtual void
      findPointNeighbours ();

    protected:

      /** \brief If set to true then normal/smoothness test will be done during segmentation. */
      bool normal_flag_;

      /** \brief If set to true then curvature test will be done during segmentation. */
      bool curvature_flag_;

      /** \brief Flag that signalizes if the smoothness constraint will be used. */
      bool smooth_mode_;

      /** \brief If set to true then residual test will be done during segmentation. */
      bool residual_flag_;

      /** \brief Thershold used for testing the smoothness between points. */
      float theta_threshold_;

      /** \brief Thershold used in residual test. */
      float residual_threshold_;

      /** \brief Thershold used in curvature test. */
      float curvature_threshold_;

      /** \brief Number of neighbours to find. */
      unsigned int neighbour_number_;

      /** \brief Serch method that will be used for KNN. */
      typename pcl::search::Search<PointT>::Ptr search_;

      /** \brief Contains normals of the points that will be segmented. */
      typename pcl::PointCloud<pcl::Normal>::Ptr normals_;

      /** \brief Stores the cloud that will be segmented. */
      typename pcl::PointCloud<PointT>::Ptr cloud_for_segmentation_;

      /** \brief After the segmentation it will contain the list of segments, which in turn are lists of indices. */
      std::vector< std::vector<int> > segments_;

      /** \brief Point labels that tells to which segment each point belongs. */
      std::vector<int> point_labels_;

      /** \brief Tells how much points each segment contains. Used for reserving memory. */
      std::vector<int> num_pts_in_segment_;

      /** \brief Contains neighbours of each point. */
      std::vector< std::vector<int> > point_neighbours_;

      /** \brief Stores the number of segments. */
      int number_of_segments_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };

 /** \brief This function is used as a comparator for sorting. */
 bool
 comparePair (std::pair<float, int>i, std::pair<float, int> j);
}

#endif
