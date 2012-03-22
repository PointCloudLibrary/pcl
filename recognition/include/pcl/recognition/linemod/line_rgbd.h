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

#ifndef PCL_RECOGNITION_LINEMOD_LINE_RGBD
#define PCL_RECOGNITION_LINEMOD_LINE_RGBD

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
#include <pcl/io/tar.h>

namespace pcl
{

  struct BoundingBoxXYZ
  {
    /** x-coordinate of the upper left front point */
    float x;
    /** y-coordinate of the upper left front point */
    float y;
    /** z-coordinate of the upper left front point */
    float z;

    /** width of the bounding box */
    float width;
    /** height of the bounding box */
    float height;
    /** depth of the bounding box */
    float depth;
  };

  /** \brief High-level class for template matching using the LINEMOD approach based on RGB and Depth data.
    * \author Stefan Holzer
    */
  template <typename PointXYZT, typename PointRGBT>
  class PCL_EXPORTS LineRGBD
  {
    public:

      /** \brief A LineRGBD detection. */
      struct Detection
      {
        /** The ID of the template. */
        size_t template_id;
        /** The ID of the object corresponding to the template. */
        size_t object_id;
        /** The ID of this detection. This is only valid for the last call of the method detect (...). */
        size_t detection_id;
        /** The response of this detection. Responses are between 0 and 1, where 1 is best. */
        float response;
        /** The 3D bounding box of the detection. */
        BoundingBoxXYZ bounding_box;
      };

      /** \brief Constructor */
      LineRGBD ()
      {
      }

      /** \brief Destructor */
      virtual ~LineRGBD ()
      {
      }

      /** \brief Loads templates from a LMT (LineMod Template) file. Overrides old templates.
        *
        * LineMod Template files are TAR files that store pairs of PCD datasets
        * together with their LINEMOD signatures in \ref
        * SparseQuantizedMultiModTemplate format.
        *
        * \param[in] file_name The name of the file that stores the templates.
        *
        * \return true, if the operation was succesful, false otherwise.
        */
      bool
      loadTemplates (const std::string &file_name, size_t object_id = 0);

      /** \brief Sets the threshold for the detection responses. Responses are between 0 and 1, where 1 is a best. 
        * \param[in] threshold The threshold used to decide where a template is detected.
        */
      inline void
      setDetectionThreshold (float threshold)
      {
        linemod_.setDetectionThreshold (threshold);
      }

      /** \brief Sets the threshold on the magnitude of color gradients. Color gradients with a magnitude below this threshold are not considered in the detection process.
        * \param[in] threshold The threshold on the magnitude of color gradients.
        */
      inline void
      setGradientMagnitudeThreshold (const float threshold)
      {
        color_gradient_mod_.setGradientMagnitudeThreshold (threshold);
      }

      /** \brief Sets the input cloud with xyz point coordinates. The cloud has to be organized. 
        * \param[in] cloud The input cloud with xyz point coordinates.
        */
      inline void
      setInputCloud (const typename pcl::PointCloud<PointXYZT>::ConstPtr & cloud)
      {
        cloud_xyz_ = cloud;

        surface_normal_mod_.setInputCloud (cloud);
        surface_normal_mod_.processInputData ();
      }

      /** \brief Sets the input cloud with rgb values. The cloud has to be organized. 
        * \param[in] cloud The input cloud with rgb values.
        */
      inline void
      setInputColors (const typename pcl::PointCloud<PointRGBT>::ConstPtr & cloud)
      {
        cloud_rgb_ = cloud;

        color_gradient_mod_.setInputCloud (cloud);
        color_gradient_mod_.processInputData ();
      }

      /** \brief Applies the detection process and fills the supplied vector with the detection instances. 
        * \param[out] detections The storage for the detection instances.
        */
      void 
      detect (std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections);

      /** Computes and returns the point cloud of the specified detection. This is the template point cloud transformed to the detection coordinates. The detection ID refers to the last call of the method detect (...).
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        * \param[out] cloud The storage for the transformed points.
        */
      void
      computeTransformedTemplatePoints (const size_t detection_id,
                                        pcl::PointCloud<pcl::PointXYZRGB> & cloud);

      /** Finds the indices of the points in the input cloud which correspond to the specified detection. The detection ID refers to the last call of the method detect (...).
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        */
      inline std::vector<size_t>
      findObjectPointIndices (const size_t detection_id)
      {
        if (detection_id >= detections_.size ())
          std::cerr << "ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds" << std::endl;

        // TODO: compute transform from detection
        // TODO: transform template points
      }


    protected:

      /** Aligns the template points with the points found at the detection position of the specified detection. The detection ID refers to the last call of the method detect (...). 
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        */
      inline std::vector<size_t>
      alignTemplatePoints (const size_t detection_id)
      {
        if (detection_id >= detections_.size ())
          std::cerr << "ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds" << std::endl;

        // TODO: compute transform from detection
        // TODO: transform template points
      }

    private:
      /** \brief Read another LTM header chunk. */
      bool 
      readLTMHeader (int fd, pcl::io::TARHeader &header);

      /** \brief LINEMOD instance. */
      public: pcl::LINEMOD linemod_;
      /** Color gradient modality */
      pcl::ColorGradientModality<PointRGBT> color_gradient_mod_;
      /** Surface normal modality */
      pcl::SurfaceNormalModality<PointXYZT> surface_normal_mod_;

      /** XYZ point cloud */
      typename pcl::PointCloud<PointXYZT>::ConstPtr cloud_xyz_;
      /** RGB point cloud */
      typename pcl::PointCloud<PointRGBT>::ConstPtr cloud_rgb_;

      /** Point clouds corresponding to the templates */
      std::vector<pcl::PointCloud<pcl::PointXYZRGB> > template_point_clouds_;
      /** Bounding boxes corresonding to the templates */
      std::vector<pcl::BoundingBoxXYZ> bounding_boxes_;
      /** Object IDs corresponding to the templates */
      std::vector<size_t> object_ids_;

      /** Detections from last call of method detect (...) */
      std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> detections_; 
  };

}

#include <pcl/recognition/impl/linemod/line_rgbd.hpp>

#endif  
