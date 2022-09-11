/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#pragma once

#if defined __GNUC__
#pragma GCC system_header
#endif

// PCL includes
#include <pcl/pcl_base.h> // for UNAVAILABLE
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>

namespace pcl
{
  namespace visualization
  {
    /** \brief Base handler class for PointCloud geometry.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandler
    {
      public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandler<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandler<PointT> >;

        /** \brief Constructor. */
        PointCloudGeometryHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false),
          field_x_idx_ (UNAVAILABLE), field_y_idx_ (UNAVAILABLE), field_z_idx_ (UNAVAILABLE),
          fields_ ()
        {}

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandler() = default;

        /** \brief Abstract getName method.
          * \return the name of the class/object.
          */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const  = 0;

        /** \brief Checl if this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Obtain the actual point geometry for the input dataset in VTK format.
          * \param[out] points the resultant geometry
          */
        virtual void
        getGeometry (vtkSmartPointer<vtkPoints> &points) const = 0;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          cloud_ = cloud;
        }

     protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the X data. */
        index_t field_x_idx_;

        /** \brief The index of the field holding the Y data. */
        index_t field_y_idx_;

        /** \brief The index of the field holding the Z data. */
        index_t field_z_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<pcl::PCLPointField> fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. Given an input dataset, all XYZ
      * data present in fields "x", "y", and "z" is extracted and displayed on screen.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerXYZ : public PointCloudGeometryHandler<PointT>
    {
      public:
        using PointCloud = typename PointCloudGeometryHandler<PointT>::PointCloud;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandlerXYZ<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandlerXYZ<PointT> >;

        /** \brief Constructor. */
        PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudGeometryHandlerXYZ"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("xyz"); }

        /** \brief Obtain the actual point geometry for the input dataset in VTK format.
          * \param[out] points the resultant geometry
          */
        virtual void
        getGeometry (vtkSmartPointer<vtkPoints> &points) const;

      private:
        // Members derived from the base class
        using PointCloudGeometryHandler<PointT>::cloud_;
        using PointCloudGeometryHandler<PointT>::capable_;
        using PointCloudGeometryHandler<PointT>::field_x_idx_;
        using PointCloudGeometryHandler<PointT>::field_y_idx_;
        using PointCloudGeometryHandler<PointT>::field_z_idx_;
        using PointCloudGeometryHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Surface normal handler class for PointCloud geometry. Given an input
      * dataset, all data present in fields "normal_x", "normal_y", and "normal_z" is
      * extracted and displayed on screen as XYZ data.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerSurfaceNormal : public PointCloudGeometryHandler<PointT>
    {
      public:
        using PointCloud = typename PointCloudGeometryHandler<PointT>::PointCloud;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandlerSurfaceNormal<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandlerSurfaceNormal<PointT> >;

        /** \brief Constructor. */
        PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudGeometryHandlerSurfaceNormal"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("normal_xyz"); }

        /** \brief Obtain the actual point geometry for the input dataset in VTK format.
          * \param[out] points the resultant geometry
          */
        virtual void
        getGeometry (vtkSmartPointer<vtkPoints> &points) const;

      private:
        // Members derived from the base class
        using PointCloudGeometryHandler<PointT>::cloud_;
        using PointCloudGeometryHandler<PointT>::capable_;
        using PointCloudGeometryHandler<PointT>::field_x_idx_;
        using PointCloudGeometryHandler<PointT>::field_y_idx_;
        using PointCloudGeometryHandler<PointT>::field_z_idx_;
        using PointCloudGeometryHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Custom handler class for PointCloud geometry. Given an input dataset and
      * three user defined fields, all data present in them is extracted and displayed on
      * screen as XYZ data.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerCustom : public PointCloudGeometryHandler<PointT>
    {
      public:
        using PointCloud = typename PointCloudGeometryHandler<PointT>::PointCloud;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandlerCustom<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandlerCustom<PointT> >;

        /** \brief Constructor. */
        PointCloudGeometryHandlerCustom (const PointCloudConstPtr &cloud,
                                         const std::string &x_field_name,
                                         const std::string &y_field_name,
                                         const std::string &z_field_name)
          : pcl::visualization::PointCloudGeometryHandler<PointT>::PointCloudGeometryHandler (cloud)
        {
          field_x_idx_ = pcl::getFieldIndex<PointT> (x_field_name, fields_);
          if (field_x_idx_ == UNAVAILABLE)
            return;
          field_y_idx_ = pcl::getFieldIndex<PointT> (y_field_name, fields_);
          if (field_y_idx_ == UNAVAILABLE)
            return;
          field_z_idx_ = pcl::getFieldIndex<PointT> (z_field_name, fields_);
          if (field_z_idx_ == UNAVAILABLE)
            return;
          field_name_ = x_field_name + y_field_name + z_field_name;
          capable_ = true;
        }

        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudGeometryHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

        /** \brief Obtain the actual point geometry for the input dataset in VTK format.
          * \param[out] points the resultant geometry
          */
        virtual void
        getGeometry (vtkSmartPointer<vtkPoints> &points) const
        {
          if (!capable_)
            return;

          if (!points)
            points = vtkSmartPointer<vtkPoints>::New ();
          points->SetDataTypeToFloat ();
          points->SetNumberOfPoints (cloud_->size ());

          float data;
          // Add all points
          double p[3];
          for (vtkIdType i = 0; i < static_cast<vtkIdType> (cloud_->size ()); ++i)
          {
            // Copy the value at the specified field
            const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&(*cloud_)[i]);
            memcpy (&data, pt_data + fields_[field_x_idx_].offset, sizeof (float));
            p[0] = data;

            memcpy (&data, pt_data + fields_[field_y_idx_].offset, sizeof (float));
            p[1] = data;

            memcpy (&data, pt_data + fields_[field_z_idx_].offset, sizeof (float));
            p[2] = data;

            points->SetPoint (i, p);
          }
        }

      private:
        // Members derived from the base class
        using PointCloudGeometryHandler<PointT>::cloud_;
        using PointCloudGeometryHandler<PointT>::capable_;
        using PointCloudGeometryHandler<PointT>::field_x_idx_;
        using PointCloudGeometryHandler<PointT>::field_y_idx_;
        using PointCloudGeometryHandler<PointT>::field_z_idx_;
        using PointCloudGeometryHandler<PointT>::fields_;

        /** \brief Name of the field used to create the geometry handler. */
        std::string field_name_;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base handler class for PointCloud geometry.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandler<pcl::PCLPointCloud2>
    {
      public:
        using PointCloud = pcl::PCLPointCloud2;
        using PointCloudPtr = PointCloud::Ptr;
        using PointCloudConstPtr = PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandler<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandler<PointCloud> >;

        /** \brief Constructor. */
        PointCloudGeometryHandler (const PointCloudConstPtr &cloud, const Eigen::Vector4f & = Eigen::Vector4f::Zero ())
          : cloud_ (cloud)
          , capable_ (false)
          , field_x_idx_ (UNAVAILABLE)
          , field_y_idx_ (UNAVAILABLE)
          , field_z_idx_ (UNAVAILABLE)
          , fields_ (cloud_->fields)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandler() = default;

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const  = 0;

        /** \brief Check if this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Obtain the actual point geometry for the input dataset in VTK format.
          * \param[out] points the resultant geometry
          */
        virtual void
        getGeometry (vtkSmartPointer<vtkPoints> &points) const;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          cloud_ = cloud;
        }

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the X data. */
        index_t field_x_idx_;

        /** \brief The index of the field holding the Y data. */
        index_t field_y_idx_;

        /** \brief The index of the field holding the Z data. */
        index_t field_z_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<pcl::PCLPointField> fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. Given an input dataset, all XYZ
      * data present in fields "x", "y", and "z" is extracted and displayed on screen.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> : public PointCloudGeometryHandler<pcl::PCLPointCloud2>
    {
      public:
        using PointCloud = PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloud;
        using PointCloudPtr = PointCloud::Ptr;
        using PointCloudConstPtr = PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandlerXYZ<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandlerXYZ<PointCloud> >;

        /** \brief Constructor. */
        PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual std::string 
        getName () const { return ("PointCloudGeometryHandlerXYZ"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("xyz"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Surface normal handler class for PointCloud geometry. Given an input
      * dataset, all data present in fields "normal_x", "normal_y", and "normal_z" is
      * extracted and displayed on screen as XYZ data.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2> : public PointCloudGeometryHandler<pcl::PCLPointCloud2>
    {
      public:
        using PointCloud = PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloud;
        using PointCloudPtr = PointCloud::Ptr;
        using PointCloudConstPtr = PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudGeometryHandlerSurfaceNormal<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudGeometryHandlerSurfaceNormal<PointCloud> >;

        /** \brief Constructor. */
        PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudGeometryHandlerSurfaceNormal"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("normal_xyz"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Custom handler class for PointCloud geometry. Given an input dataset and
      * three user defined fields, all data present in them is extracted and displayed on
      * screen as XYZ data.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2> : public PointCloudGeometryHandler<pcl::PCLPointCloud2>
    {
      public:
        using PointCloud = PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloud;
        using PointCloudPtr = PointCloud::Ptr;
        using PointCloudConstPtr = PointCloud::ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerCustom (const PointCloudConstPtr &cloud,
                                         const std::string &x_field_name,
                                         const std::string &y_field_name,
                                         const std::string &z_field_name);

        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudGeometryHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

      private:
        /** \brief Name of the field used to create the geometry handler. */
        std::string field_name_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#endif
