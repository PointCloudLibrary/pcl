/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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
#ifndef PCL_POINT_CLOUD_IMAGE_EXTRACTORS_H_
#define PCL_POINT_CLOUD_IMAGE_EXTRACTORS_H_

#include <pcl/point_cloud.h>
#include <pcl/PCLImage.h>

namespace pcl
{
  namespace io
  {
    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base Image Extractor class for organized point clouds.
      *
      * This is an abstract class that declares an interface for image extraction from
      * organized point clouds. The family of its subclasses provide functionality to
      * extract images from particular fields.
      *
      * The following piece of code demonstrates typical usage of a PointCloudImageExtractor
      * subclass. Here the data are extracted from the "label" field and are saved as a
      * PNG image file.
      *
      * \code
      *   // Source point cloud (needs to be filled with data of course)
      *   pcl::PointCloud<pcl::PointXYZLabel> cloud;
      *   // Target image
      *   pcl::PCLImage image;
      *   // Create PointCloudImageExtractor subclass that can handle "label" field
      *   pcl::io::PointCloudImageExtractorFromLabelField<pcl::XYZLabel> pcie;
      *   // Set it up if not happy with the defaults
      *   pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
      *   // Try to extract an image
      *   bool success = pcie.extract(cloud, image);
      *   // Save to file if succeeded
      *   if (success)
      *     pcl::io::saveImage ("filename.png", image);
      * \endcode
      *
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractor
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;

        typedef boost::shared_ptr<PointCloudImageExtractor<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractor<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudImageExtractor () {}

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractor () {}

        /** \brief Obtain the image from the given cloud.
          * \param[in] cloud organized point cloud to extract image from
          * \param[out] image the output image
          * \return true if the operation was successful, false otherwise
          */
        virtual bool
        extract (const PointCloud& cloud, pcl::PCLImage& image) const = 0;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor extension which provides functionality to apply scaling to
      * the values extracted from a field.
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorWithScaling : public PointCloudImageExtractor<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorWithScaling<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorWithScaling<PointT> > ConstPtr;

        /** \brief Different scaling methods.
          * <ul>
          *   <li><b>SCALING_NO</b> - no scaling.</li>
          *   <li><b>SCALING_FULL_RANGE</b> - scales to full range of the output value.</li>
          *   <li><b>SCASING_FIXED_FACTOR</b> - scales by a given fixed factor.</li>
          * </ul>
          */
        enum ScalingMethod
        {
          SCALING_NO,
          SCALING_FULL_RANGE,
          SCALING_FIXED_FACTOR
        };

        /** \brief Constructor. */
        PointCloudImageExtractorWithScaling (const std::string& field_name, const ScalingMethod scaling_method)
          : field_name_ (field_name)
          , scaling_method_ (scaling_method)
          , scaling_factor_ (1.0f)
        {
        }

        /** \brief Constructor. */
        PointCloudImageExtractorWithScaling (const std::string& field_name, const float scaling_factor)
          : field_name_ (field_name)
          , scaling_method_ (SCALING_FIXED_FACTOR)
          , scaling_factor_ (scaling_factor)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorWithScaling () {}

        /** \brief Obtain the image from the given cloud.
          * \param[in] cloud organized point cloud to extract image from
          * \param[out] image the output image
          * \return true if the operation was successful, false otherwise
          */
        virtual bool
        extract (const PointCloud& cloud, pcl::PCLImage& image) const;

        /** \brief Set scaling method. */
        inline void
        setScalingMethod (const ScalingMethod scaling_method)
        {
          scaling_method_ = scaling_method;
        }

        /** \brief Set fixed scaling factor. */
        inline void
        setScalingFactor (const float scaling_factor)
        {
          scaling_factor_ = scaling_factor;
        }

      protected:

        std::string field_name_;
        ScalingMethod scaling_method_;
        float scaling_factor_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "normal" field. Normal
      * vector components (x, y, z) are mapped to color channels (r, g, b respectively).
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromNormalField : public PointCloudImageExtractor<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromNormalField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromNormalField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudImageExtractorFromNormalField () {}

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromNormalField () {}

        /** \brief Obtain the color image from the given cloud.
          * The cloud should contain "normal" field.
          * \param[in] cloud organized point cloud to extract image from
          * \param[out] img the output image
          * \return true if the operation was successful, false otherwise
          */
        virtual bool
        extract (const PointCloud& cloud, pcl::PCLImage& img) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "rgb" or "rgba" fields
      * to produce a color image with rgb8 encoding.
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromRGBField : public PointCloudImageExtractor<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromRGBField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromRGBField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudImageExtractorFromRGBField () {}

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromRGBField () {}

        /** \brief Obtain the color image from the given cloud.
          * The cloud should contain either "rgb" or "rgba" field.
          * \param[in] cloud organized point cloud to extract image from
          * \param[out] img the output image
          * \return true if the operation was successful, false otherwise
          */
        virtual bool
        extract (const PointCloud& cloud, pcl::PCLImage& img) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "label" field to produce
      * either monochrome or RGB image where different labels correspond to different
      * colors. In the monochrome case colors are shades of gray, in the RGB case the
      * colors are generated randomly.
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromLabelField : public PointCloudImageExtractor<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromLabelField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromLabelField<PointT> > ConstPtr;

        /** \brief Different modes for color mapping.
          * <ul>
          *   <li><b>COLORS_MONO</b> - shades of gray (according to label id).</li>
          *   <li><b>COLORS_RGB_RANDOM</b> - randomly generated RGB colors.</li>
          * </ul>
          */
        enum ColorMode
        {
          COLORS_MONO,
          COLORS_RGB_RANDOM,
        };

        /** \brief Constructor. */
        PointCloudImageExtractorFromLabelField (const ColorMode color_mode = COLORS_MONO)
          : color_mode_ (color_mode)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromLabelField () {}

        /** \brief Obtain the label image from the given cloud.
          * The cloud should contain "label" field.
          * \note Labels using more than 16 bits will cause problems in COLORS_MONO mode.
          * \param[in] cloud organized point cloud to extract image from
          * \param[out] img the output image
          * \return true if the operation was successful, false otherwise
          */
        virtual bool
        extract (const PointCloud& cloud, pcl::PCLImage& img) const;

        /** \brief Set color mapping mode. */
        inline void
        setColorMode (const ColorMode color_mode)
        {
          color_mode_ = color_mode;
        }

      private:

        ColorMode color_mode_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "z" field to produce a
      * depth map (as a monochrome image with mono16 encoding).
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromZField : public PointCloudImageExtractorWithScaling<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;
      typedef typename PointCloudImageExtractorWithScaling<PointT>::ScalingMethod ScalingMethod;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromZField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromZField<PointT> > ConstPtr;

        /** \brief Constructor.
          * \param[in] scaling_factor a scaling factor to apply to each depth value (default 10000)
          */
        PointCloudImageExtractorFromZField (const float scaling_factor = 10000)
          : PointCloudImageExtractorWithScaling<PointT> ("z", scaling_factor)
        {
        }

        /** \brief Constructor.
          * \param[in] scaling_method a scaling method to use
          */
        PointCloudImageExtractorFromZField (const ScalingMethod scaling_method)
          : PointCloudImageExtractorWithScaling<PointT> ("z", scaling_method)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromZField () {}

      protected:
        // Members derived from the base class
        using PointCloudImageExtractorWithScaling<PointT>::field_name_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_method_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_factor_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "curvature" field to
      * produce a curvature map (as a monochrome image with mono16 encoding).
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromCurvatureField : public PointCloudImageExtractorWithScaling<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;
      typedef typename PointCloudImageExtractorWithScaling<PointT>::ScalingMethod ScalingMethod;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromCurvatureField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromCurvatureField<PointT> > ConstPtr;

        /** \brief Constructor.
          * \param[in] scaling_method a scaling method to use (default SCALING_FULL_RANGE)
          */
        PointCloudImageExtractorFromCurvatureField (const ScalingMethod scaling_method = PointCloudImageExtractorWithScaling<PointT>::SCALING_FULL_RANGE)
          : PointCloudImageExtractorWithScaling<PointT> ("curvature", scaling_method)
        {
        }

        /** \brief Constructor.
          * \param[in] scaling_factor a scaling factor to apply to each curvature value
          */
        PointCloudImageExtractorFromCurvatureField (const float scaling_factor)
          : PointCloudImageExtractorWithScaling<PointT> ("curvature", scaling_factor)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromCurvatureField () {}

      protected:
        // Members derived from the base class
        using PointCloudImageExtractorWithScaling<PointT>::field_name_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_method_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_factor_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Image Extractor which uses the data present in the "intensity" field to produce a
      * monochrome intensity image (with mono16 encoding).
      * \author Sergey Alexandrov
      * \ingroup io
      */
    template <typename PointT>
    class PointCloudImageExtractorFromIntensityField : public PointCloudImageExtractorWithScaling<PointT>
    {
      typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;
      typedef typename PointCloudImageExtractorWithScaling<PointT>::ScalingMethod ScalingMethod;

      public:
        typedef boost::shared_ptr<PointCloudImageExtractorFromIntensityField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudImageExtractorFromIntensityField<PointT> > ConstPtr;

        /** \brief Constructor.
          * \param[in] scaling_method a scaling method to use (default SCALING_NO)
          */
        PointCloudImageExtractorFromIntensityField (const ScalingMethod scaling_method = PointCloudImageExtractorWithScaling<PointT>::SCALING_NO)
          : PointCloudImageExtractorWithScaling<PointT> ("intensity", scaling_method)
        {
        }

        /** \brief Constructor.
          * \param[in] scaling_factor a scaling factor to apply to each intensity value
          */
        PointCloudImageExtractorFromIntensityField (const float scaling_factor)
          : PointCloudImageExtractorWithScaling<PointT> ("intensity", scaling_factor)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudImageExtractorFromIntensityField () {}

      protected:
        // Members derived from the base class
        using PointCloudImageExtractorWithScaling<PointT>::field_name_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_method_;
        using PointCloudImageExtractorWithScaling<PointT>::scaling_factor_;
    };

  }
}

#include <pcl/io/impl/point_cloud_image_extractors.hpp>

#endif  //#ifndef PCL_POINT_CLOUD_IMAGE_EXTRACTORS_H_
