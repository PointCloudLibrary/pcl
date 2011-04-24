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
 * $Id$
 *
 */
#ifndef PCL_POINT_CLOUD_HANDLERS_H_
#define PCL_POINT_CLOUD_HANDLERS_H_

#include <pcl/visualization/common/common.h>
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

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
          cloud_ (cloud), capable_ (false)
        {}

        /** \brief Abstract getName method. */
        virtual std::string 
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string 
        getFieldName () const  = 0;

        /** \brief Return whether this handler is capable of handling the input data or not. */
        inline bool 
        isCapable () const { return (capable_); }

        /** \brief Obtain the actual point geometry for the input dataset as a vtk pointset.
          * \param points the resultant geometry 
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. 
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

        /** \brief Class getName method. */
        virtual inline std::string 
        getName () const { return ("PointCloudGeometryHandlerXYZ"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return ("xyz"); }

        /** \brief Obtain the actual point geometry for the input dataset as a vtk pointset.
          * \param points the resultant geometry 
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Surface normal handler class for PointCloud geometry. 
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

        /** \brief Obtain the actual point geometry for the input dataset as a vtk pointset.
          * \param points the resultant geometry 
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Custom handler class for PointCloud geometry. 
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

        /** \brief Obtain the actual point geometry for the input dataset as a vtk pointset.
          * \param points the resultant geometry 
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

    //////////////////////////////////////////////////////////////////////////////////////////
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
        PointCloudGeometryHandler (const PointCloudConstPtr &cloud) : 
          cloud_ (cloud), capable_ (false)
        {
          fields_ = cloud_->fields;
        }

        /** \brief Abstract getName method. */
        virtual std::string 
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string 
        getFieldName () const  = 0;

        /** \brief Return whether this handler is capable of handling the input data or not. */
        inline bool 
        isCapable () const { return (capable_); }

        /** \brief Obtain the actual point geometry for the input dataset as a vtk pointset.
          * \param points the resultant geometry 
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief XYZ handler class for PointCloud geometry. 
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

        /** \brief Class getName method. */
        virtual inline 
        std::string getName () const { return ("PointCloudGeometryHandlerXYZ"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return ("xyz"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Surface normal handler class for PointCloud geometry. 
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Custom handler class for PointCloud geometry. 
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

    //////////////////////////////////////////////////////////////////////////////////////////
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
          cloud_ (cloud), capable_ (false)
        {}

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
          * \param scalars the resultant scalars containing the color for the input dataset
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
   
    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors. 
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
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. 
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
        { 
          capable_ = true; 
          r_ = r;
          g_ = g;
          b_ = b;
        }

        /** \brief Abstract getName method. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. 
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

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return ("rgb"); }

      protected:
        /** \brief Class getName method. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerRGBField"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      private:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Generic field handler class for colors. 
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

        /** \brief Get the name of the field used. */
        virtual std::string getFieldName () const { return (field_name_); }

      protected:
        /** \brief Class getName method. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerGenericField"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

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
          cloud_ (cloud), capable_ (false)
        {}

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
          * \param scalars the resultant scalars containing the color for the input dataset
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

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors. 
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
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. 
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
          PointCloudColorHandler<sensor_msgs::PointCloud2> (cloud)
        { 
          capable_ = true; 
          r_ = r;
          g_ = g;
          b_ = b;
        }

        /** \brief Get the name of the class. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Internal R, G, B holding the values given by the user. */
        double r_, g_, b_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. 
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

      protected:
        /** \brief Get the name of the class. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerRGBField"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return ("rgb"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Generic field handler class for colors. 
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

      protected:
        /** \brief Get the name of the class. */
        virtual inline std::string 
        getName () const { return ("PointCloudColorHandlerGenericField"); }

        /** \brief Get the name of the field used. */
        virtual std::string 
        getFieldName () const { return (field_name_); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param scalars the resultant scalars containing the color for the input dataset
          */
        virtual void 
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      private:
        /** \brief Name of the field used to create the color handler. */
        std::string field_name_;
    };

  }
}

#include "pcl/visualization/impl/point_cloud_handlers.hpp"

#endif
