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

#ifndef PCL_REGION_GROWING_RGB_H_
#define PCL_REGION_GROWING_RGB_H_

#include <pcl/segmentation/region_growing.h>

namespace pcl
{
  /** \brief
    * Implements the well known Region Growing algorithm used for segmentation based on color of points.
    * Description can be found in the article
    * "Color-based segmentation of point clouds"
    * by Qingming Zhan, Yubin Liang, Yinghui Xiao
    */
  template <typename PointT>
  class PCL_EXPORTS RegionGrowingRGB : public RegionGrowing<PointT>
  {
    using RegionGrowing<PointT>::normal_flag_;
    using RegionGrowing<PointT>::curvature_flag_;
    using RegionGrowing<PointT>::residual_flag_;
    using RegionGrowing<PointT>::number_of_segments_;
    using RegionGrowing<PointT>::segments_;
    using RegionGrowing<PointT>::point_labels_;
    using RegionGrowing<PointT>::num_pts_in_segment_;
    using RegionGrowing<PointT>::point_neighbours_;
    using RegionGrowing<PointT>::applySmoothRegionGrowingAlgorithm;
    using RegionGrowing<PointT>::cloud_for_segmentation_;
    using RegionGrowing<PointT>::theta_threshold_;
    using RegionGrowing<PointT>::normals_;
    using RegionGrowing<PointT>::smooth_mode_;
    using RegionGrowing<PointT>::curvature_threshold_;
    using RegionGrowing<PointT>::residual_threshold_;
    using RegionGrowing<PointT>::neighbour_number_;
    using RegionGrowing<PointT>::search_;

    public:

      /** \brief Constructor that sets default values for member variables. */
      RegionGrowingRGB ();

      /** \brief Destructor that frees memory. */
      virtual
      ~RegionGrowingRGB ();

      /** \brief Returns the color threshold value used for testing if points belong to the same region. */
      float
      getPointColorThreshold () const;

      /** \brief Returns the color threshold value used for testing if regions can be merged. */
      float
      getRegionColorThreshold () const;

      /** \brief Returns the distance threshold. If the distance between two points is less or equal to
        * distance threshold value, then those points assumed to be neighbouring points.
        */
      float
      getDistanceThreshold () const;

      /** \brief Returns the desired minimum number of points that each region must have. */
      unsigned int
      getMinPointNumber () const;

      /** \brief Returns the number of nearest neighbours used for searching K nearest segments.
        * Note that here it refers to the segments(not the points).
        */
      unsigned int
      getNumberOfRegionNeighbours () const;

      /** \brief This method specifies the threshold value for color test between the points.
        * This kind of testing is made at the first stage of the algorithm(region growing).
        * If the difference between points color is less than threshold value, then they are considered
        * to be in the same region.
        * \param[in] thresh new threshold value for color test
        */
      void
      setPointColorThreshold (float thresh);

      /** \brief This method specifies the threshold value for color test between the regions.
        * This kind of testing is made at the second stage of the algorithm(region merging).
        * If the difference between segments color is less than threshold value, then they are merged together.
        * \param[in] thresh new threshold value for color test
        */
      void
      setRegionColorThreshold (float thresh);

      /** \brief Allows to set distance threshold.
        * \param[in] thresh new threshold value for neighbour test
        */
      void
      setDistanceThreshold (float thresh);

      /** \brief Allows to set a threshold that will be used for growing regions.
        * If the region is to small(it has less points than specified in point_number), then
        * it will be merged with the nearest neighbouring region. It provides a subtle approach to
        * control the under- and over- segmentation.
        * \param[in] point_number new threshold value for min number of points in region
        */
      void
      setMinPointNumber (unsigned int point_number);

      /** \brief This method allows to set the number of neighbours that is used for finding
        * neighbouring segments. Neighbouring segments are needed for the merging process.
        * \param[in] nghbr_number the number of neighbouring segments to find
        */
      void
      setNumberOfRegionNeighbours (unsigned int nghbr_number);

      /** \brief Returns the flag that signalize if the normal/smoothness test is turned on/off. */
      bool
      getNormalTestFlag () const;

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

       /** \brief
         * Allows to turn on/off the normal/smoothness test.
         * \param[in] value new value for normal/smoothness test. If set to true then the test will be turned on
         */
      void
      setNormalTest (bool value);

      /** \brief This method simply launches the segmentation algorithm */
      virtual unsigned int
      segmentPoints ();

      /** \brief Allows to turn on/off the curvature test.
        * \param[in] value new value for curvature test. If set to true then the test will be turned on
        */
      virtual void
      setCurvatureTest (bool value);

      /** \brief
        * Allows to turn on/off the residual test.
        * \param[in] value new value for residual test. If set to true then the test will be turned on
        */
      virtual void
      setResidualTest (bool value);

    protected:

      /** \brief This function implements the merging algorithm described in the article
        * "Color-based segmentation of point clouds"
        * by Qingming Zhan, Yubin Liang, Yinghui Xiao
        */
      unsigned int
      applyRegionMergingAlgorithm ();

      /** \brief This method calculates the colorimetrical difference between two points.
        * In this case it simply returns the euclidean distance between two colors.
        * \param[in] first_color the color of the first point
        * \param[in] second_color the color of the second point
        */
      float
      calculateColorimetricalDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color) const;

      /** \brief This method finds K nearest neighbours of the given segment.
        * \param[in] index index of the segment for which neighbours will be found
        * \param[in] nghbr_number the number of neighbours to find
        * \param[out] nghbrs the array of indices of the neighbours that were found
        * \param[out] dist the array of distances to the corresponding neighbours
        */
      void
      findRegionsKNN (int index, int nghbr_number, std::vector<int>& nghbrs, std::vector<float>& dist);

      /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
        * If so, then it returns true. It also checks if this point can serve as the seed.
        * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
        * \param[in] point index of the current seed point
        * \param[in] nghbr index of the point that is neighbour of the current seed
        * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
        */
      virtual bool
      validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;

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

      /** \brief This method simply calls the findRegionsKNN for each segment and
        * saves the results for later use.
        */
      void
      findSegmentNeighbours ();

      /** \brief This method assembles the array containing neighbours of each homogeneous region.
        * Homogeneous region is the union of some segments. This array is used when the regions
        * with a few points need to be merged with the neighbouring region.
        * \param[out] neighbours_out vector of lists of neighbours for every homogeneous region
        * \param[in] regions_in vector of lists, each list contains indices of segments that belong
        * to the corresponding homogeneous region.
        */
      void
      findRegionNeighbours (std::vector< std::vector< std::pair<float, int> > >& neighbours_out, std::vector< std::vector<int> >& regions_in);

      /** \brief This function simply assembles the regions from list of point labels.
        * \param[in] num_pts_in_region for each final region it stores the corresponding number of points in it
        * \param[in] num_regions number of regions to assemble
        */
      void
      assembleRegions (std::vector<unsigned int>& num_pts_in_region, int num_regions);

    protected:

      /** \brief Number of neighbouring segments to find. */
      unsigned int region_neighbour_number_;

      /** \brief Thershold used in color test for points. */
      float color_p2p_threshold_;

      /** \brief Thershold used in color test for regions. */
      float color_r2r_threshold_;

      /** \brief Thershold that specifies the desired minimum number of points in region. */
      unsigned int min_point_number_;

      /** \brief Threshold that tells which points we need to assume neighbouring. */
      float distance_threshold_;

      /** \brief Stores distances for the point neighbours from point_neighbours_ */
      std::vector< std::vector<float> > point_distances_;

      /** \brief Stores new indices for segments that were obtained at the region growing stage. */
      std::vector<int> segment_labels_;

      /** \brief Stores the neighboures for the corresponding segments. */
      std::vector< std::vector<int> > segment_neighbours_;

      /** \brief Stores distances for the segment neighbours from segment_neighbours_ */
      std::vector< std::vector<float> > segment_distances_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif
