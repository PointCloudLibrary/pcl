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
 */

#ifndef PCL_SIFT_KEYPOINT_H_
#define PCL_SIFT_KEYPOINT_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  template<typename PointT>
  struct SIFTKeypointFieldSelector
  {
    inline float
    operator () (const PointT & p) const
    {
      return p.intensity;
    }
  };
  template<>
  struct SIFTKeypointFieldSelector<PointNormal>
  {
    inline float
    operator () (const PointNormal & p) const
    {
      return p.curvature;
    }
  };
  template<>
  struct SIFTKeypointFieldSelector<PointXYZRGB>
  {
    inline float
    operator () (const PointXYZRGB & p) const
    {
      return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) / 1000.0f);
    }
  };
  template<>
  struct SIFTKeypointFieldSelector<PointXYZRGBA>
  {
    inline float
    operator () (const PointXYZRGBA & p) const
    {
      return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) / 1000.0f);
    }
  };

  /** \brief @b SIFTKeypoint detects the Scale Invariant Feature Transform
    * keypoints for a given point cloud dataset containing points and intensity.
    * This implementation adapts the original algorithm from images to point
    * clouds. 
    *
    * For more information about the image-based SIFT interest operator, see:
    *
    *    David G. Lowe, "Distinctive image features from scale-invariant keypoints," 
    *    International Journal of Computer Vision, 60, 2 (2004), pp. 91-110.
    *
    * \author Michael Dixon
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT>
  class SIFTKeypoint : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<SIFTKeypoint<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const SIFTKeypoint<PointInT, PointOutT> > ConstPtr;

      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::initCompute;    

      /** \brief Empty constructor. */
      SIFTKeypoint () : min_scale_ (0.0), nr_octaves_ (0), nr_scales_per_octave_ (0), 
        min_contrast_ (-std::numeric_limits<float>::max ()), scale_idx_ (-1), 
        out_fields_ (), getFieldValue_ ()
      {
        name_ = "SIFTKeypoint";
      }

      /** \brief Specify the range of scales over which to search for keypoints
        * \param min_scale the standard deviation of the smallest scale in the scale space
        * \param nr_octaves the number of octaves (i.e. doublings of scale) to compute 
        * \param nr_scales_per_octave the number of scales to compute within each octave
        */
      void 
      setScales (float min_scale, int nr_octaves, int nr_scales_per_octave);

      /** \brief Provide a threshold to limit detection of keypoints without sufficient contrast
        * \param min_contrast the minimum contrast required for detection
        */
      void 
      setMinimumContrast (float min_contrast);

    protected:
      bool
      initCompute ();

      /** \brief Detect the SIFT keypoints for a set of points given in setInputCloud () using the spatial locator in 
        * setSearchMethod ().
        * \param output the resultant cloud of keypoints
        */
      void 
      detectKeypoints (PointCloudOut &output);

    private:
      /** \brief Detect the SIFT keypoints for a given point cloud for a single octave.
        * \param input the point cloud to detect keypoints in
        * \param tree a k-D tree of the points in \a input
        * \param base_scale the first (smallest) scale in the octave
        * \param nr_scales_per_octave the number of scales to to compute
        * \param output the resultant point cloud containing the SIFT keypoints
        */
      void 
      detectKeypointsForOctave (const PointCloudIn &input, KdTree &tree, 
                                float base_scale, int nr_scales_per_octave, 
                                PointCloudOut &output);

      /** \brief Compute the difference-of-Gaussian (DoG) scale space for the given input and scales
        * \param input the point cloud for which the DoG scale space will be computed
        * \param tree a k-D tree of the points in \a input
        * \param scales a vector containing the scales over which to compute the DoG scale space
        * \param diff_of_gauss the resultant DoG scale space (in a number-of-points by number-of-scales matrix)
        */
      void 
      computeScaleSpace (const PointCloudIn &input, KdTree &tree, 
                         const std::vector<float> &scales, 
                         Eigen::MatrixXf &diff_of_gauss);

      /** \brief Find the local minima and maxima in the provided difference-of-Gaussian (DoG) scale space
        * \param input the input point cloud 
        * \param tree a k-D tree of the points in \a input
        * \param diff_of_gauss the DoG scale space (in a number-of-points by number-of-scales matrix)
        * \param extrema_indices the resultant vector containing the point indices of each keypoint
        * \param extrema_scales the resultant vector containing the scale indices of each keypoint
        */
      void 
      findScaleSpaceExtrema (const PointCloudIn &input, KdTree &tree, 
                             const Eigen::MatrixXf &diff_of_gauss,
                             std::vector<int> &extrema_indices, std::vector<int> &extrema_scales);


      /** \brief The standard deviation of the smallest scale in the scale space.*/
      float min_scale_;

      /** \brief The number of octaves (i.e. doublings of scale) over which to search for keypoints.*/
      int nr_octaves_;

      /** \brief The number of scales to be computed for each octave.*/
      int nr_scales_per_octave_;

      /** \brief The minimum contrast required for detection.*/
      float min_contrast_;

      /** \brief Set to a value different than -1 if the output cloud has a "scale" field and we have to save 
        * the keypoints scales. */
      int scale_idx_;

      /** \brief The list of fields present in the output point cloud data. */
      std::vector<pcl::PCLPointField> out_fields_;

      SIFTKeypointFieldSelector<PointInT> getFieldValue_;
  };
}

#include <pcl/keypoints/impl/sift_keypoint.hpp>

#endif // #ifndef PCL_SIFT_KEYPOINT_H_
