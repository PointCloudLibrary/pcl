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
 *
 */

#ifndef PCL_FEATURES_COLORH_3D_H_
#define PCL_FEATURES_COLORH_3D_H_

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief ColorHEstimation estimates the Color Histogram (ColorH) descriptor for a given
   * point cloud dataset containing XYZ data and RGB
   * The suggested PointOutT is pcl::Histogram<90>. 
   * Histogram is formed with 3 components of the color space taken into account at a time. 
   * i.e. bin of the histogram is a cell in a 3d matrix formed by the 3 components of color.
   * This is implemented in reference to the publication:
   * "A.Richtsfeld, T. Mörwald, J. Prankl, M. Zillich and M. Vincze: Learning of Perceptual Grouping for Object Segmentation on RGB-D Data;
   * Journal of Visual Communication and Image Representation (JVCI), Special Issue on Visual Understanding and Applications with RGB-D
   * Cameras, July 2013."
   *
   * \author Karthik Desingh
   * \ingroup features
   */
  template<typename PointInT,typename PointOutT = pcl::Histogram<64> >
  class ColorHEstimation3D: public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<ColorHEstimation3D<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const ColorHEstimation3D<PointInT, PointOutT> > ConstPtr;
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Constructor. */
      ColorHEstimation3D () :
        nbins_ (64), isrgb_ (true), ishsv_ (false), isyuv_ (false)
      {
        feature_name_ = "ColorHEstimation3D";
      }
      ;

       /** \brief Set the bin size.
       * \param[in] nbins the number of bins of the histogram
       */
      inline void
      setNBins (int nbins) { nbins_ = nbins;}

      /** \brief Get the bin size. 
       * \return the number of bins of the histogram
       */
      inline int
      getNBins () { return (nbins_);}
      
      /** \brief Set the rgb type flag. 
       * \param[in] isrgb the flag that says if rgb space of color histogram to be processed
       */
      inline void
      setRGBHistogram (bool isrgb) { isrgb_ = isrgb; ishsv_ = !isrgb; isyuv_ = !isrgb; }

      /** \brief Get the rgb type flag. 
       * \return the flag that says if rgb space of color histogram being processed
       */
      inline bool
      getRGBHistogram () { return (isrgb_);}

      /** \brief Set the hsv type flag. 
       * \param[in] ishsv the flag that says if hsv space of color histogram to be processed
       */
      inline void
      setHSVHistogram (bool ishsv) { ishsv_ = ishsv; isyuv_ = !ishsv; isrgb_ = !ishsv; }
      
      /** \brief Get the hsv type flag. 
       * \return  the flag that says if hsv space of color histogram being processed
       */
      inline bool
      getHSVHistogram () { return (ishsv_);}

      /** \brief Set the yuv type flag. 
       * \param[in] isyuv the flag that says if yuv space of color histogram to be processed
       */
      inline void 
      setYUVHistogram (bool isyuv) { isyuv_ = isyuv; isrgb_ = !isyuv; ishsv_ = !isyuv; }

      /** \brief Get the yuv type flag. 
       * \return the flag that says if yuv space of color histogram being processed
       */
      inline bool
      getYUVHistogram () { return (isyuv_);}
      
      /** \brief Estimate the ColorH histogram at
       * a set of points given by <setInputCloud (), setIndices ()> using the surface in
       * setSearchSurface ()
       * \param[out] output the resultant point cloud with a ColorH histogram
       */
      void
      computeFeature (PointCloudOut &output);

    private:
      /** \brief Number of bins, this should match the Output type */
      int nbins_;

      /** \brief Signals if rgb color space is used to compute histogram */
      bool isrgb_;

      /** \brief Signals if hsv color space is used to compute histogram */
      bool ishsv_;

      /** \brief Signals if yuv color space is used to compute histogram */
      bool isyuv_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/colorh_3d.hpp>
#endif

#endif  //#ifndef PCL_FEATURES_COLORH_3D_H_

