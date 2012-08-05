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
 * $Id$
 *
 */
#ifndef PCL_POINT_CLOUD_HANDLERS_H_
#define PCL_POINT_CLOUD_HANDLERS_H_

#include <pcl/visualization/common/common.h>
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
// VTK includes
#include <pcl/visualization/vtk.h>

namespace pcl
{
  namespace visualization
  {
    /** \brief Base handler class for PointCloud geometry.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandler
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef typename boost::shared_ptr<PointCloudGeometryHandler<PointT> > Ptr;
        typedef typename boost::shared_ptr<const PointCloudGeometryHandler<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false),
          field_x_idx_ (-1), field_y_idx_ (-1), field_z_idx_ (-1),
          fields_ ()
        {}

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandler () {}

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

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the X data. */
        int field_x_idx_;

        /** \brief The index of the field holding the Y data. */
        int field_y_idx_;

        /** \brief The index of the field holding the Z data. */
        int field_z_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<sensor_msgs::PointField> fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. Given an input dataset, all XYZ
      * data present in fields "x", "y", and "z" is extracted and displayed on screen.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerXYZ : public PointCloudGeometryHandler<PointT>
    {
      public:
        typedef typename PointCloudGeometryHandler<PointT>::PointCloud PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef typename boost::shared_ptr<PointCloudGeometryHandlerXYZ<PointT> > Ptr;
        typedef typename boost::shared_ptr<const PointCloudGeometryHandlerXYZ<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud);

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandlerXYZ () {};

        /** \brief Class getName method. */
        virtual inline std::string
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
      * extracted and dislayed on screen as XYZ data.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerSurfaceNormal : public PointCloudGeometryHandler<PointT>
    {
      public:
        typedef typename PointCloudGeometryHandler<PointT>::PointCloud PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef typename boost::shared_ptr<PointCloudGeometryHandlerSurfaceNormal<PointT> > Ptr;
        typedef typename boost::shared_ptr<const PointCloudGeometryHandlerSurfaceNormal<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual inline std::string
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
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudGeometryHandlerCustom : public PointCloudGeometryHandler<PointT>
    {
      public:
        typedef typename PointCloudGeometryHandler<PointT>::PointCloud PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef typename boost::shared_ptr<PointCloudGeometryHandlerCustom<PointT> > Ptr;
        typedef typename boost::shared_ptr<const PointCloudGeometryHandlerCustom<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerCustom (const PointCloudConstPtr &cloud,
                                         const std::string &x_field_name,
                                         const std::string &y_field_name,
                                         const std::string &z_field_name);

        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudGeometryHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

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

        /** \brief Name of the field used to create the geometry handler. */
        std::string field_name_;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base handler class for PointCloud geometry.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandler<sensor_msgs::PointCloud2>
    {
      public:
        typedef sensor_msgs::PointCloud2 PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudGeometryHandler<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudGeometryHandler<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandler (const PointCloudConstPtr &cloud, const Eigen::Vector4f & = Eigen::Vector4f::Zero ())
          : cloud_ (cloud)
          , capable_ (false)
          , field_x_idx_ (-1)
          , field_y_idx_ (-1)
          , field_z_idx_ (-1)
          , fields_ (cloud_->fields)
        {
        }

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandler () {}

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

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the X data. */
        int field_x_idx_;

        /** \brief The index of the field holding the Y data. */
        int field_y_idx_;

        /** \brief The index of the field holding the Z data. */
        int field_z_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<sensor_msgs::PointField> fields_;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. Given an input dataset, all XYZ
      * data present in fields "x", "y", and "z" is extracted and displayed on screen.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> : public PointCloudGeometryHandler<sensor_msgs::PointCloud2>
    {
      public:
        typedef PointCloudGeometryHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudGeometryHandlerXYZ<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudGeometryHandlerXYZ<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud);

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandlerXYZ () {}

        /** \brief Class getName method. */
        virtual inline
        std::string getName () const { return ("PointCloudGeometryHandlerXYZ"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("xyz"); }
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief Surface normal handler class for PointCloud geometry. Given an input
      * dataset, all data present in fields "normal_x", "normal_y", and "normal_z" is
      * extracted and dislayed on screen as XYZ data.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerSurfaceNormal<sensor_msgs::PointCloud2> : public PointCloudGeometryHandler<sensor_msgs::PointCloud2>
    {
      public:
        typedef PointCloudGeometryHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudGeometryHandlerSurfaceNormal<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudGeometryHandlerSurfaceNormal<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud);

        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudGeometryHandlerSurfaceNormal"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("normal_xyz"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Custom handler class for PointCloud geometry. Given an input dataset and
      * three user defined fields, all data present in them is extracted and displayed on
      * screen as XYZ data.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudGeometryHandlerCustom<sensor_msgs::PointCloud2> : public PointCloudGeometryHandler<sensor_msgs::PointCloud2>
    {
      public:
        typedef PointCloudGeometryHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        /** \brief Constructor. */
        PointCloudGeometryHandlerCustom (const PointCloudConstPtr &cloud,
                                         const std::string &x_field_name,
                                         const std::string &y_field_name,
                                         const std::string &z_field_name);

        /** \brief Destructor. */
        virtual ~PointCloudGeometryHandlerCustom () {}

        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudGeometryHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

      private:
        /** \brief Name of the field used to create the geometry handler. */
        std::string field_name_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base Handler class for PointCloud colors.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandler
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudColorHandler<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandler<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false), field_idx_ (-1), fields_ ()
        {}

        /** \brief Destructor. */
        virtual ~PointCloudColorHandler () {}

        /** \brief Check if this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const = 0;

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the data that represents the color. */
        int field_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<sensor_msgs::PointField> fields_;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors (i.e., R, G, B will be randomly chosen)
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRandom : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRandom<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRandom<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<PointT> (cloud)
        {
          capable_ = true;
        }

        /** \brief Abstract getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
    };

    ///////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. The color at each point will be drawn
      * as the use given R, G, B values.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerCustom : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerCustom<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerCustom<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerCustom (const PointCloudConstPtr &cloud,
                                      double r, double g, double b)
          : PointCloudColorHandler<PointT> (cloud)
          , r_ (r)
          , g_ (g)
          , b_ (b)
        {
          capable_ = true;
        }

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerCustom () {};

        /** \brief Abstract getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;

        /** \brief Internal R, G, B holding the values given by the user. */
        double r_, g_, b_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields as the color at each point.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRGBField : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud);

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerRGBField () {}

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgb"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerRGBField"); }

      private:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields of additional cloud as the color at each point.
      * \ingroup visualization
      */

    template <typename PointT>
    class PointCloudColorHandlerRGBCloud : public PointCloudColorHandler<PointT>
    {
      using PointCloudColorHandler<PointT>::capable_;
      using PointCloudColorHandler<PointT>::cloud_;

      typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename pcl::PointCloud<RGB>::ConstPtr RgbCloudConstPtr;

    public:
      typedef boost::shared_ptr<PointCloudColorHandlerRGBCloud<PointT> > Ptr;
      typedef boost::shared_ptr<const PointCloudColorHandlerRGBCloud<PointT> > ConstPtr;
      
      /** \brief Constructor. */
      PointCloudColorHandlerRGBCloud (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors);
            
      virtual void getColor (vtkSmartPointer<vtkDataArray> &scalars) const;  
    private:
      virtual std::string getFieldName () const { return ("additional rgb"); }
      virtual inline std::string getName () const { return ("PointCloudColorHandlerRGBCloud"); }
      RgbCloudConstPtr rgb_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief HSV handler class for colors. Uses the data present in the "h", "s", "v"
      * fields as the color at each point.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerHSVField : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerHSVField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerHSVField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("hsv"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerHSVField"); }

        /** \brief The field index for "S". */
        int s_field_idx_;

        /** \brief The field index for "V". */
        int v_field_idx_;
      private:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Generic field handler class for colors. Uses an user given field to extract
      * 1D data and display the color at each point using a min-max lookup table.
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerGenericField : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerGenericField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerGenericField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
                                            const std::string &field_name);

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerGenericField () {}

        /** \brief Get the name of the field used. */
        virtual std::string getFieldName () const { return (field_name_); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Class getName method. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerGenericField"); }

      private:
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;

        /** \brief Name of the field used to create the color handler. */
        std::string field_name_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base Handler class for PointCloud colors.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      public:
        typedef sensor_msgs::PointCloud2 PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudColorHandler<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandler<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false), field_idx_ ()
        {}
        
        /** \brief Destructor. */
        virtual ~PointCloudColorHandler () {}

        /** \brief Return whether this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const = 0;

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the data that represents the color. */
        int field_idx_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors (i.e., R, G, B will be randomly chosen)
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerRandom<sensor_msgs::PointCloud2> : public PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      typedef PointCloudColorHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRandom<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRandom<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<sensor_msgs::PointCloud2> (cloud)
        {
          capable_ = true;
        }

        /** \brief Get the name of the class. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. The color at each point will be drawn
      * as the use given R, G, B values.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerCustom<sensor_msgs::PointCloud2> : public PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      typedef PointCloudColorHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        /** \brief Constructor. */
        PointCloudColorHandlerCustom (const PointCloudConstPtr &cloud,
                                      double r, double g, double b) :
          PointCloudColorHandler<sensor_msgs::PointCloud2> (cloud),
          r_ (r), g_ (g), b_ (b)
        {
          capable_ = true;
        }

        /** \brief Get the name of the class. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Internal R, G, B holding the values given by the user. */
        double r_, g_, b_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields as the color at each point.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> : public PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      typedef PointCloudColorHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud);

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerRGBField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgb"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief HSV handler class for colors. Uses the data present in the "h", "s", "v"
      * fields as the color at each point.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerHSVField<sensor_msgs::PointCloud2> : public PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      typedef PointCloudColorHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerHSVField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerHSVField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerHSVField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("hsv"); }

        /** \brief The field index for "S". */
        int s_field_idx_;

        /** \brief The field index for "V". */
        int v_field_idx_;
     };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Generic field handler class for colors. Uses an user given field to extract
      * 1D data and display the color at each point using a min-max lookup table.
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> : public PointCloudColorHandler<sensor_msgs::PointCloud2>
    {
      typedef PointCloudColorHandler<sensor_msgs::PointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerGenericField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerGenericField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
                                            const std::string &field_name);

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          */
        virtual void
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual inline std::string
        getName () const { return ("PointCloudColorHandlerGenericField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

      private:
        /** \brief Name of the field used to create the color handler. */
        std::string field_name_;
    };

  }
}

#include <pcl/visualization/impl/point_cloud_handlers.hpp>

#endif
