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

#ifndef PCL_INTEGRALIMAGE_BASED_NORMAL_ESTIMATOR_H_
#define PCL_INTEGRALIMAGE_BASED_NORMAL_ESTIMATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/integral_image2D.h>

#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic ignored "-Weffc++"
#endif
namespace pcl
{
  /** \brief Surface normal estimation on organized data using integral images.
    * \author Stefan Holzer
    */
  template <typename PointInT, typename PointOutT>
  class IntegralImageNormalEstimation: public Feature<PointInT, PointOutT>
  {
    using Feature<PointInT, PointOutT>::input_;
    using Feature<PointInT, PointOutT>::feature_name_;
    using Feature<PointInT, PointOutT>::tree_;
    using Feature<PointInT, PointOutT>::k_;

    public:

      enum NormalEstimationMethod
      {
        COVARIANCE_MATRIX,
        AVERAGE_3D_GRADIENT,
        AVERAGE_DEPTH_CHANGE,
        SIMPLE_3D_GRADIENT
      };

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Constructor */
      IntegralImageNormalEstimation ()
        : normal_estimation_method_(AVERAGE_3D_GRADIENT)
        , rect_width_ (0), rect_width_2_ (0), rect_width_4_ (0)
        , rect_height_ (0), rect_height_2_ (0), rect_height_4_ (0)
        , distance_threshold_ (0)
        , integral_image_DX_ (false)
        , integral_image_DY_ (false)
        , integral_image_depth_ (false)
        , integral_image_XYZ_ (true)
        , diff_x_ (NULL)
        , diff_y_ (NULL)
        , depth_data_ (NULL)
        , distance_map_ (NULL)
        , use_depth_dependent_smoothing_ (false)
        , max_depth_change_factor_ (20.0f*0.001f)
        , normal_smoothing_size_ (10.0f)
        , init_covariance_matrix_ (false)
        , init_average_3d_gradient_ (false)
        , init_simple_3d_gradient_ (false)
        , init_depth_change_ (false)
      {
        feature_name_ = "IntegralImagesNormalEstimation";
        tree_.reset ();
        k_ = 1;
      }

      /** \brief Destructor **/
      virtual ~IntegralImageNormalEstimation ();

      /** \brief Set the regions size which is considered for normal estimation.
        * \param[in] width the width of the search rectangle
        * \param[in] height the height of the search rectangle
        */
      void
      setRectSize (const int width, const int height);

      /** \brief Computes the normal at the specified position.
        * \param[in] pos_x x position (pixel)
        * \param[in] pos_y y position (pixel)
        * \param[in] point_index the position index of the point
        * \param[out] normal the output estimated normal
        */
      void
      computePointNormal (const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal);

      /** \brief The depth change threshold for computing object borders
        * \param[in] max_depth_change_factor the depth change threshold for computing object borders based on
        * depth changes
        */
      void
      setMaxDepthChangeFactor (float max_depth_change_factor)
      {
        max_depth_change_factor_ = max_depth_change_factor;
      }

      /** \brief Set the normal smoothing size
        * \param[in] normal_smoothing_size factor which influences the size of the area used to smooth normals
        * (depth dependent if useDepthDependentSmoothing is true)
        */
      void
      setNormalSmoothingSize (float normal_smoothing_size)
      {
        normal_smoothing_size_ = normal_smoothing_size;
      }

      /** \brief Set the normal estimation method. The current implemented algorithms are:
        * <ul>
        *   <li><b>COVARIANCE_MATRIX</b> - creates 9 integral images to compute the normal for a specific point
        *   from the covariance matrix of its local neighborhood.</li>
        *   <li><b>AVERAGE_3D_GRADIENT</b> - creates 6 integral images to compute smoothed versions of
        *   horizontal and vertical 3D gradients and computes the normals using the cross-product between these
        *   two gradients.
        *   <li><b>AVERAGE_DEPTH_CHANGE</b> -  creates only a single integral image and computes the normals
        *   from the average depth changes.
        * </ul>
        * \param[in] normal_estimation_method the method used for normal estimation
        */
      void
      setNormalEstimationMethod (NormalEstimationMethod normal_estimation_method)
      {
        normal_estimation_method_ = normal_estimation_method;
      }

      /** \brief Set whether to use depth depending smoothing or not
        * \param[in] use_depth_dependent_smoothing decides whether the smoothing is depth dependent
        */
      void
      setDepthDependentSmoothing (bool use_depth_dependent_smoothing)
      {
        use_depth_dependent_smoothing_ = use_depth_dependent_smoothing;
      }

       /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
         * \param[in] cloud the const boost shared pointer to a PointCloud message
         */
      virtual inline void
      setInputCloud (const typename PointCloudIn::ConstPtr &cloud)
      {
        input_ = cloud;
        if (!cloud->isOrganized ())
        {
          PCL_ERROR ("[pcl::IntegralImageNormalEstimation::setInputCloud] Input dataset is not organized (height = 1).\n");
          return;
        }

        init_covariance_matrix_ = init_average_3d_gradient_ = init_depth_change_ = false;
        
        if (use_sensor_origin_)
        {
          vpx_ = input_->sensor_origin_.coeff (0);
          vpy_ = input_->sensor_origin_.coeff (1);
          vpz_ = input_->sensor_origin_.coeff (2);
        }

        // Initialize the correct data structure based on the normal estimation method chosen
        initData ();
      }

      /** \brief Returns a pointer to the distance map which was computed internally
        */
      inline float*
      getDistanceMap ()
      {
        return (distance_map_);
      }

      /** \brief Set the viewpoint.
        * \param vpx the X coordinate of the viewpoint
        * \param vpy the Y coordinate of the viewpoint
        * \param vpz the Z coordinate of the viewpoint
        */
      inline void
      setViewPoint (float vpx, float vpy, float vpz)
      {
        vpx_ = vpx;
        vpy_ = vpy;
        vpz_ = vpz;
        use_sensor_origin_ = false;
      }

      /** \brief Get the viewpoint.
        * \param [out] vpx x-coordinate of the view point
        * \param [out] vpy y-coordinate of the view point
        * \param [out] vpz z-coordinate of the view point
        * \note this method returns the currently used viewpoint for normal flipping.
        * If the viewpoint is set manually using the setViewPoint method, this method will return the set view point coordinates.
        * If an input cloud is set, it will return the sensor origin otherwise it will return the origin (0, 0, 0)
        */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz)
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

      /** \brief sets whether the sensor origin or a user given viewpoint should be used. After this method, the 
        * normal estimation method uses the sensor origin of the input cloud.
        * to use a user defined view point, use the method setViewPoint
        */
      inline void
      useSensorOriginAsViewPoint ()
      {
        use_sensor_origin_ = true;
        if (input_)
        {
          vpx_ = input_->sensor_origin_.coeff (0);
          vpy_ = input_->sensor_origin_.coeff (1);
          vpz_ = input_->sensor_origin_.coeff (2);
        }
        else
        {
          vpx_ = 0;
          vpy_ = 0;
          vpz_ = 0;
        }
      }
      
    protected:

      /** \brief Computes the normal for the complete cloud.
        * \param[out] output the resultant normals
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Initialize the data structures, based on the normal estimation method chosen. */
      void
      initData ();

    private:
      /** \brief The normal estimation method to use. Currently, 3 implementations are provided:
        *
        * - COVARIANCE_MATRIX
        * - AVERAGE_3D_GRADIENT
        * - AVERAGE_DEPTH_CHANGE
        */
      NormalEstimationMethod normal_estimation_method_;

      /** The width of the neighborhood region used for computing the normal. */
      int rect_width_;
      int rect_width_2_;
      int rect_width_4_;
      /** The height of the neighborhood region used for computing the normal. */
      int rect_height_;
      int rect_height_2_;
      int rect_height_4_;

      /** the threshold used to detect depth discontinuities */
      float distance_threshold_;

      /** integral image in x-direction */
      IntegralImage2D<float, 3> integral_image_DX_;
      /** integral image in y-direction */
      IntegralImage2D<float, 3> integral_image_DY_;
      /** integral image */
      IntegralImage2D<float, 1> integral_image_depth_;
      /** integral image xyz */
      IntegralImage2D<float, 3> integral_image_XYZ_;

      /** derivatives in x-direction */
      float *diff_x_;
      /** derivatives in y-direction */
      float *diff_y_;

      /** depth data */
      float *depth_data_;

      /** distance map */
      float *distance_map_;

      /** \brief Smooth data based on depth (true/false). */
      bool use_depth_dependent_smoothing_;

      /** \brief Threshold for detecting depth discontinuities */
      float max_depth_change_factor_;

      /** \brief */
      float normal_smoothing_size_;

      /** \brief True when a dataset has been received and the covariance_matrix data has been initialized. */
      bool init_covariance_matrix_;

      /** \brief True when a dataset has been received and the average 3d gradient data has been initialized. */
      bool init_average_3d_gradient_;

      /** \brief True when a dataset has been received and the simple 3d gradient data has been initialized. */
      bool init_simple_3d_gradient_;

      /** \brief True when a dataset has been received and the depth change data has been initialized. */
      bool init_depth_change_;

      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
        * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
      float vpx_, vpy_, vpz_;

      /** whether the sensor origin of the input cloud or a user given viewpoint should be used.*/
      bool use_sensor_origin_;
      
      /** \brief This method should get called before starting the actual computation. */
      bool
      initCompute ();

      /** \brief Internal initialization method for COVARIANCE_MATRIX estimation. */
      void
      initCovarianceMatrixMethod ();

      /** \brief Internal initialization method for AVERAGE_3D_GRADIENT estimation. */
      void
      initAverage3DGradientMethod ();

      /** \brief Internal initialization method for AVERAGE_DEPTH_CHANGE estimation. */
      void
      initAverageDepthChangeMethod ();

      /** \brief Internal initialization method for SIMPLE_3D_GRADIENT estimation. */
      void
      initSimple3DGradientMethod ();

    private:
      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic warning "-Weffc++"
#endif


#endif

