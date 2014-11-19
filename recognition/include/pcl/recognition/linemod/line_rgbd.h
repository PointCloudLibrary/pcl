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
    /** \brief Constructor. */
    BoundingBoxXYZ () : x (0.0f), y (0.0f), z (0.0f), width (0.0f), height (0.0f), depth (0.0f) {}

    /** \brief X-coordinate of the upper left front point */
    float x;
    /** \brief Y-coordinate of the upper left front point */
    float y;
    /** \brief Z-coordinate of the upper left front point */
    float z;

    /** \brief Width of the bounding box */
    float width;
    /** \brief Height of the bounding box */
    float height;
    /** \brief Depth of the bounding box */
    float depth;
  };

  /** \brief High-level class for template matching using the LINEMOD approach based on RGB and Depth data.
    * \author Stefan Holzer
    */
  template <typename PointXYZT, typename PointRGBT=PointXYZT>
  class PCL_EXPORTS LineRGBD
  {
    public:

      /** \brief A LineRGBD detection. */
      struct Detection
      {
        /** \brief Constructor. */
        Detection () : template_id (0), object_id (0), detection_id (0), response (0.0f), bounding_box () {}

        /** \brief The ID of the template. */
        size_t template_id;
        /** \brief The ID of the object corresponding to the template. */
        size_t object_id;
        /** \brief The ID of this detection. This is only valid for the last call of the method detect (...). */
        size_t detection_id;
        /** \brief The response of this detection. Responses are between 0 and 1, where 1 is best. */
        float response;
        /** \brief The 3D bounding box of the detection. */
        BoundingBoxXYZ bounding_box;
        /** \brief The 2D template region of the detection. */
        RegionXY region;
      };

      /** \brief Constructor */
      LineRGBD ()
        : intersection_volume_threshold_ (1.0f)
        , linemod_ ()
        , color_gradient_mod_ ()
        , surface_normal_mod_ ()
        , cloud_xyz_ ()
        , cloud_rgb_ ()
        , template_point_clouds_ ()
        , bounding_boxes_ ()
        , object_ids_ ()
        , detections_ ()
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
        * \param object_id
        *
        * \return true, if the operation was successful, false otherwise.
        */
      bool
      loadTemplates (const std::string &file_name, size_t object_id = 0);

      bool
      addTemplate (const SparseQuantizedMultiModTemplate & sqmmt, pcl::PointCloud<pcl::PointXYZRGBA> & cloud, size_t object_id = 0);

      /** \brief Sets the threshold for the detection responses. Responses are between 0 and 1, where 1 is a best. 
        * \param[in] threshold The threshold used to decide where a template is detected.
        */
      inline void
      setDetectionThreshold (float threshold)
      {
        linemod_.setDetectionThreshold (threshold);
      }

      /** \brief Sets the threshold on the magnitude of color gradients. Color gradients with a magnitude below 
        *        this threshold are not considered in the detection process.
        * \param[in] threshold The threshold on the magnitude of color gradients.
        */
      inline void
      setGradientMagnitudeThreshold (const float threshold)
      {
        color_gradient_mod_.setGradientMagnitudeThreshold (threshold);
      }

      /** \brief Sets the threshold for the decision whether two detections of the same template are merged or not. 
        *        If ratio between the intersection of the bounding boxes of two detections and the original bounding 
        *        boxes is larger than the specified threshold then they are merged. If detection A overlaps with 
        *        detection B and B with C than A, B, and C are merged. Threshold has to be between 0 and 1.
        * \param[in] threshold The threshold on the ratio between the intersection bounding box and the original 
        *                      bounding box.
        */
      inline void
      setIntersectionVolumeThreshold (const float threshold = 1.0f)
      {
        intersection_volume_threshold_ = threshold;
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

      /** \brief Creates a template from the specified data and adds it to the matching queue. 
        * \param cloud
        * \param object_id
        * \param[in] mask_xyz the mask that determine which parts of the xyz-modality are used for creating the template.
        * \param[in] mask_rgb the mask that determine which parts of the rgb-modality are used for creating the template.
        * \param[in] region the region which will be associated with the template (can be larger than the actual modality-maps).
        */
      int 
      createAndAddTemplate (
        pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
        const size_t object_id,
        const MaskMap & mask_xyz,
        const MaskMap & mask_rgb,
        const RegionXY & region);


      /** \brief Applies the detection process and fills the supplied vector with the detection instances. 
        * \param[out] detections The storage for the detection instances.
        */
      void 
      detect (std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections);

      /** \brief Applies the detection process in a semi-scale-invariant manner. This is done by acutally
        *        scaling the template to different sizes.
        */
      void
      detectSemiScaleInvariant (std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections,
                                float min_scale = 0.6944444f,
                                float max_scale = 1.44f,
                                float scale_multiplier = 1.2f);

      /** \brief Computes and returns the point cloud of the specified detection. This is the template point 
        *        cloud transformed to the detection coordinates. The detection ID refers to the last call of 
        *        the method detect (...).
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        * \param[out] cloud The storage for the transformed points.
        */
      void
      computeTransformedTemplatePoints (const size_t detection_id,
                                        pcl::PointCloud<pcl::PointXYZRGBA> & cloud);

      /** \brief Finds the indices of the points in the input cloud which correspond to the specified detection. 
        *        The detection ID refers to the last call of the method detect (...).
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        */
      inline std::vector<size_t>
      findObjectPointIndices (const size_t detection_id)
      {
        if (detection_id >= detections_.size ())
          PCL_ERROR ("ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds\n");

        // TODO: compute transform from detection
        // TODO: transform template points
        std::vector<size_t> vec;
        return (vec);
      }


    protected:

      /** \brief Aligns the template points with the points found at the detection position of the specified detection. 
        *        The detection ID refers to the last call of the method detect (...). 
        * \param[in] detection_id The ID of the detection (according to the last call of the method detect (...)).
        */
      inline std::vector<size_t>
      alignTemplatePoints (const size_t detection_id)
      {
        if (detection_id >= detections_.size ())
          PCL_ERROR ("ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds\n");

        // TODO: compute transform from detection
        // TODO: transform template points
        std::vector<size_t> vec;
        return (vec);
      }

      /** \brief Refines the found detections along the depth. */
      void
      refineDetectionsAlongDepth ();

      /** \brief Applies projective ICP on detections to find their correct position in depth. */
      void
      applyProjectiveDepthICPOnDetections ();

      /** \brief Checks for overlapping detections, removes them and keeps only the strongest one. */
      void
      removeOverlappingDetections ();

      /** \brief Computes the volume of the intersection between two bounding boxes.
        * \param[in] box1 First bounding box.
        * \param[in] box2 Second bounding box.
        */
      static float
      computeBoundingBoxIntersectionVolume (const BoundingBoxXYZ &box1, const BoundingBoxXYZ &box2);

    private:
      /** \brief Read another LTM header chunk. */
      bool 
      readLTMHeader (int fd, pcl::io::TARHeader &header);

      /** \brief Intersection volume threshold. */
      float intersection_volume_threshold_;

      /** \brief LINEMOD instance. */
      public: pcl::LINEMOD linemod_;
      /** \brief Color gradient modality. */
      pcl::ColorGradientModality<PointRGBT> color_gradient_mod_;
      /** \brief Surface normal modality. */
      pcl::SurfaceNormalModality<PointXYZT> surface_normal_mod_;

      /** \brief XYZ point cloud. */
      typename pcl::PointCloud<PointXYZT>::ConstPtr cloud_xyz_;
      /** \brief RGB point cloud. */
      typename pcl::PointCloud<PointRGBT>::ConstPtr cloud_rgb_;

      /** \brief Point clouds corresponding to the templates. */
      pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType template_point_clouds_;
      /** \brief Bounding boxes corresonding to the templates. */
      std::vector<pcl::BoundingBoxXYZ> bounding_boxes_;
      /** \brief Object IDs corresponding to the templates. */
      std::vector<size_t> object_ids_;

      /** \brief Detections from last call of method detect (...). */
      std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> detections_; 
  };

}

#include <pcl/recognition/impl/linemod/line_rgbd.hpp>

#endif  
