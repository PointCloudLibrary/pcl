/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_
#define PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_

#include <pcl/apps/optronic_viewer/qt.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <fz_api.h>

#include <string>
#include <vector>

namespace pcl
{
  namespace apps
  {
    namespace optronic_viewer
    {
      
      /////////////////////////////////////////////////////////////////////////
      /** \brief Interface for a class that implements a filter for a point
       *         cloud.
       */
      class CloudFilter
      {
      public:
        virtual ~CloudFilter () {}

        /** \brief Applies the filter on the input and stores the result in the
         *         output cloud.
         */
        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out) = 0;

        /** \brief Returns the name of the filter. */
        virtual std::string
        getName ()
        {
          return name_;
        }

        /** \brief Sets the name of the filter. */
        virtual void
        setName (std::string & name)
        {
          name_ = name;
        }

        /** \brief Returns a Qt page which allows to configure the filter. */
        virtual QWizardPage *
        getParameterPage () = 0;

      protected:
        /** \brief The name of the filter. */
        std::string name_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Factory class to create a filter. */
      class CloudFilterFactory
      {
      public:
        virtual ~CloudFilterFactory () {}

        /** \brief Returns the name of the filter type. */
        virtual std::string
        getName ()
        {
          return filter_type_name_;
        }

        /** \brief Creates a filter object. */
        virtual CloudFilter*
        create () = 0;

      protected:
        /** \brief Constructor which sets the filter type name. */
        CloudFilterFactory (std::string name)
          : filter_type_name_ (name)
        {}

      protected:
        /** \brief Filter type name */
        std::string filter_type_name_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Helper class for the factory to simplify implementation of
       *         new cloud filters. This class makes the implementation of a 
       *         separate factory class obsolete, e.g.:
       *               
       *         Header:
       *         char g_voxel_grid_cf_name[];
       *         typedef CloudFilterFactory2<VoxelGridCF, g_voxel_grid_cf_name> VoxelGridCFF2;
       *         
       *         Source:
       *         extern char pcl::apps::optronic_viewer::g_voxel_grid_cf_name[] = "VoxelGrid Filter";
       */
      template <class T, char const *name>
      class CloudFilterFactory2
        : public CloudFilterFactory
      {
      public:
        /** \brief Creates a new cloud factory with the name specified in
         *         the template parameter.
         */
        CloudFilterFactory2 () : CloudFilterFactory (name) {}
        virtual ~CloudFilterFactory2 () {}

        /** \brief Creates a new filter object. */
        virtual CloudFilter*
        create ()
        {
          return (new T ());
        }
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a voxel grid filter. Divides the space in voxels
       *         and takes a point per voxel.
       */
      class VoxelGridCF
        : public CloudFilter
      {
      public:
        VoxelGridCF ();
        virtual ~VoxelGridCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);

        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        // filter parameters
        float voxel_grid_size_x_;
        float voxel_grid_size_y_;
        float voxel_grid_size_z_;

        QLabel * voxel_grid_size_x_label_;
        QLabel * voxel_grid_size_y_label_;
        QLabel * voxel_grid_size_z_label_;
        QLineEdit * voxel_grid_size_x_line_edit_;
        QLineEdit * voxel_grid_size_y_line_edit_;
        QLineEdit * voxel_grid_size_z_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a pass-through filter. Removes all points that are
       *         out of a specified range for a specified component (e.g. x, y,
       *         or z).
       */
      class PassThroughCF
        : public CloudFilter
      {
      public:
        PassThroughCF ();
        virtual ~PassThroughCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        std::string filter_field_name_;
        float filter_limits_min_;
        float filter_limits_max_;

        QLabel * filter_field_name_label_;
        QLabel * filter_limits_min_label_;
        QLabel * filter_limits_max_label_;
        QLineEdit * filter_field_name_line_edit_;
        QLineEdit * filter_limits_min_line_edit_;
        QLineEdit * filter_limits_max_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a radius-outlier filter. Removes all points that
       *         have less than a specified number of points as neighbors
       *         (within a specified radius).
       */
      class RadiusOutlierCF
        : public CloudFilter
      {
      public:
        RadiusOutlierCF ();
        virtual ~RadiusOutlierCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        float search_radius_;
        int min_neighbors_in_radius_;

        QLabel * search_radius_label_;
        QLabel * min_neighbors_in_radius_label_;
        QLineEdit * search_radius_line_edit_;
        QLineEdit * min_neighbors_in_radius_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for the fast-bilateral filter. Applies the fast-
       *         bilateral filter on a cloud for smoothing.
       */
      class FastBilateralCF
        : public CloudFilter
      {
      public:
        FastBilateralCF ();
        virtual ~FastBilateralCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        float sigma_s_;
        float sigma_r_;

        QLabel * sigma_s_label_;
        QLabel * sigma_r_label_;
        QLineEdit * sigma_s_line_edit_;
        QLineEdit * sigma_r_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a Median filter. Applies the Median filter on a
       *         cloud.
       */
      class MedianCF
        : public CloudFilter
      {
      public:
        MedianCF ();
        virtual ~MedianCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        float max_allowed_movement_;
        int window_size_;

        QLabel * max_allowed_movement_label_;
        QLabel * window_size_label_;
        QLineEdit * max_allowed_movement_line_edit_;
        QLineEdit * window_size_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a random sample filter. Selects a random sample
       *         of points from the input cloud.
       */
      class RandomSampleCF
        : public CloudFilter
      {
      public:
        RandomSampleCF ();
        virtual ~RandomSampleCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        int seed_;
        int sample_;

        QLabel * seed_label_;
        QLabel * sample_label_;
        QLineEdit * seed_line_edit_;
        QLineEdit * sample_line_edit_;
        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };

      /////////////////////////////////////////////////////////////////////////
      /** \brief Wrapper for a filter that finds the dominant plane in the
       *         cloud and either keeps only the plane or everything else. 
       */
      class PlaneCF
        : public CloudFilter
      {
      public:
        PlaneCF ();
        virtual ~PlaneCF ();

        virtual void
        filter (
          pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);
        
        virtual QWizardPage *
        getParameterPage ()
        {
          return (filter_selection_page_);
        }

      protected:
        double threshold_;
        int max_iterations_;

        double refinement_sigma_;
        int max_refinement_iterations_;

        bool return_negative_;

        double cluster_tolerance_;
        int min_cluster_size_;

        QLabel * threshold_label_;
        QLabel * max_iterations_label_;
        QLabel * refinement_sigma_label_;
        QLabel * max_refinement_iterations_label_;
        QLabel * return_negative_label_;
        QLabel * cluster_tolerance_label_;
        QLabel * min_cluster_size_label_;

        QLineEdit * threshold_line_edit_;
        QLineEdit * max_iterations_line_edit_;
        QLineEdit * refinement_sigma_line_edit_;
        QLineEdit * max_refinement_iterations_line_edit_;
        QCheckBox * return_negative_check_box_;
        QLineEdit * cluster_tolerance_line_edit_;
        QLineEdit * min_cluster_size_line_edit_;

        QVBoxLayout * main_layout_;
        QWizardPage * filter_selection_page_;
      };


      /////////////////////////////////////////////////////////////////////////
      char g_voxel_grid_cf_name[];
      char g_passthrough_cf_name[];
      char g_radius_outlier_cf_name[];
      char g_fast_bilateral_cf_name[];
      char g_median_cf_name[];
      char g_random_sample_cf_name[];
      char g_plane_cf_name[];

      /////////////////////////////////////////////////////////////////////////
      typedef CloudFilterFactory2< VoxelGridCF,      g_voxel_grid_cf_name>      VoxelGridCFF2;
      typedef CloudFilterFactory2< PassThroughCF,    g_passthrough_cf_name>     PassThroughCFF2;
      typedef CloudFilterFactory2< RadiusOutlierCF,  g_radius_outlier_cf_name>  RadiusOutlierCFF2;
      typedef CloudFilterFactory2< FastBilateralCF,  g_fast_bilateral_cf_name>  FastBilateralCFF2;
      typedef CloudFilterFactory2< MedianCF,         g_median_cf_name>          MedianCFF2;
      typedef CloudFilterFactory2< RandomSampleCF,   g_random_sample_cf_name>   RandomSampleCFF2;
      typedef CloudFilterFactory2< PlaneCF,          g_plane_cf_name>           PlaneCFF2;

    }
  }
}

#endif // PCL_APPS_OPTRONIC_VIEWER_CLOUD_FILTER_H_
