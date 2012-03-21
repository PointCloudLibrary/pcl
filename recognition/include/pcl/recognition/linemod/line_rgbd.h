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

  /**
    * \brief High-level class for template matching using the LINEMOD approach based on RGB and Depth data.
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

      /** \brief Loads templates from a file. Overrides old templates.
        * \param[in] file_name The name of the file that stores the templates.
        */
      inline void
      loadTemplates (const char * file_name)
      {
        linemod_.loadTemplates (file_name);

        // TODO: we need to load pcd files too...

        // TODO: we need to compute and store the 3D bounding boxes of all templates...
      }

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
      inline void 
      detect (std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections)
      {
        std::vector<pcl::QuantizableModality*> modalities;
        modalities.push_back (&color_gradient_mod_);
        modalities.push_back (&surface_normal_mod_);

        std::vector<pcl::LINEMODDetection> linemod_detections;
        linemod_.detectTemplates (modalities, linemod_detections);

        detections_.clear ();
        detections_.reserve (linemod_detections.size ());
        detections.clear ();
        detections.reserve (linemod_detections.size ());
        for (size_t detection_id = 0; detection_id < linemod_detections.size (); ++detection_id)
        {
          pcl::LINEMODDetection & linemod_detection = linemod_detections[detection_id];

          pcl::LineRGBD<PointXYZT, PointRGBT>::Detection detection;
          detection.template_id = linemod_detection.template_id;
          detection.detection_id = detection_id;
          detection.response = linemod_detection.score;
          
          // compute bounding box:
          // we assume that the bounding boxes of the templates are relative to the center of mass 
          // of the template points; so we also compute the center of mass of the points
          // covered by the 

          const pcl::SparseQuantizedMultiModTemplate & linemod_template = linemod_.getTemplate (linemod_detection.template_id);

          const size_t start_x = std::max (linemod_detection.x, 0);
          const size_t start_y = std::max (linemod_detection.y, 0);
          const size_t end_x = std::min (static_cast<size_t> (start_x + linemod_template.region.width), static_cast<size_t> (cloud_xyz_->width));
          const size_t end_y = std::min (static_cast<size_t> (start_y + linemod_template.region.height), static_cast<size_t> (cloud_xyz_->height));

          float center_x = 0.0f;
          float center_y = 0.0f;
          float center_z = 0.0f;
          size_t counter = 0;
          for (size_t row_index = start_y; row_index < end_y; ++row_index)
          {
            for (size_t col_index = start_x; col_index < end_x; ++col_index)
            {
              const pcl::PointXYZ & point = (*cloud_xyz_) (col_index, row_index);

              if (pcl_isfinite (point.x) && pcl_isfinite (point.y) && pcl_isfinite (point.z))
              {
                center_x += point.x;
                center_y += point.y;
                center_z += point.z;
                ++counter;
              }
            }
          }
          const float inv_counter = 1.0f / static_cast<float> (counter);
          center_x *= inv_counter;
          center_y *= inv_counter;
          center_z *= inv_counter;

          pcl::BoundingBoxXYZ template_bounding_box = bounding_boxes_[detection.template_id];
          detection.bounding_box = template_bounding_box;
          detection.bounding_box.x += center_x;
          detection.bounding_box.y += center_y;
          detection.bounding_box.z += center_z;

          detections.push_back (detection);
          detections_.push_back (detection);
        }
      }

      /** Computes and returns the point cloud of the specified detection. This is the template point cloud transformed to the detection coordinates. The detection ID refers to the last call of the method detect (...).
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        * \param[out] cloud The storage for the transformed points.
        */
      inline void
      computeTransformedTemplatePoints (const size_t detection_id,
                                        pcl::PointCloud<pcl::PointXYZRGB> & cloud)
      {
        if (detection_id >= detections_.size ())
          std::cerr << "ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds" << std::endl;

        const size_t template_id = detections_[detection_id].template_id;
        pcl::PointCloud<pcl::PointXYZRGB> & template_point_cloud = template_point_clouds_[template_id];

        const pcl::BoundingBoxXYZ & template_bounding_box = bounding_boxes_[template_id];
        const pcl::BoundingBoxXYZ & detection_bounding_box = detections_[detection_id].bounding_box;

        const float translation_x = detection_bounding_box.x - template_bounding_box.x;
        const float translation_y = detection_bounding_box.y - template_bounding_box.y;
        const float translation_z = detection_bounding_box.z - template_bounding_box.z;

        const size_t nr_points = template_point_cloud.size ();
        cloud.resize (nr_points);
        for (size_t point_index = 0; point_index < nr_points; ++point_index)
        {
          pcl::PointXYZRGB point = template_point_cloud.points[point_index];

          point.x += translation_x;
          point.y += translation_y;
          point.z += translation_z;

          cloud.points[point_index] = point;
        }
      }

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

      /** LINEMOD instance */
      pcl::LINEMOD linemod_;
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

      /** Detections from last call of method detect (...) */
      std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> detections_;

  };

}

#endif  
