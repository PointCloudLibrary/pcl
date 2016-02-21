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
#ifndef PCL_POINT_CLOUD_COLOR_HANDLERS_H_
#define PCL_POINT_CLOUD_COLOR_HANDLERS_H_

#if defined __GNUC__
#pragma GCC system_header
#endif

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/common/common.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>

namespace pcl
{
  namespace visualization
  {
    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base Handler class for PointCloud colors.
      * \author Radu B. Rusu 
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
        PointCloudColorHandler () :
          cloud_ (), capable_ (false), field_idx_ (-1), fields_ ()
        {}

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
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
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

        /** \brief The index of the field holding the data that represents the color. */
        int field_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<pcl::PCLPointField> fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors (i.e., R, G, B will be randomly chosen)
      * \author Radu B. Rusu 
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
        PointCloudColorHandlerRandom () :
          PointCloudColorHandler<PointT> ()
        {
          capable_ = true;
        }

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<PointT> (cloud)
        {
          capable_ = true;
        }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. The color at each point will be drawn
      * as the use given R, G, B values.
      * \author Radu B. Rusu 
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
        PointCloudColorHandlerCustom (double r, double g, double b)
          : PointCloudColorHandler<PointT> ()
          , r_ (r)
          , g_ (g)
          , b_ (b)
        {
          capable_ = true;
        }

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
        virtual std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
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
      * \author Radu B. Rusu 
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
        PointCloudColorHandlerRGBField ()
        {
          capable_ = false;
        }

        /** \brief Constructor. */
        PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud)
          : PointCloudColorHandler<PointT> (cloud)
        {
          setInputCloud (cloud);
        }

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerRGBField () {}

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgb"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRGBField"); }

      private:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
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
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerHSVField () {}

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("hsv"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Class getName method. */
        virtual std::string
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
      * \author Radu B. Rusu 
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
        PointCloudColorHandlerGenericField (const std::string &field_name)
          : field_name_ (field_name)
        {
          capable_ = false;
        }

        /** \brief Constructor. */
        PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
                                            const std::string &field_name)
          : PointCloudColorHandler<PointT> (cloud)
          , field_name_ (field_name)
        {
          setInputCloud (cloud);
        }

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerGenericField () {}

        /** \brief Get the name of the field used. */
        virtual std::string getFieldName () const { return (field_name_); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerGenericField"); }

      private:
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;

        /** \brief Name of the field used to create the color handler. */
        std::string field_name_;
    };


    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGBA handler class for colors. Uses the data present in the "rgba" field as
      * the color at each point. Transparency is handled.
      * \author Nizar Sallem
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRGBAField : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBAField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBAField<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBAField ()
        {
          capable_ = false;
        }

        /** \brief Constructor. */
        PointCloudColorHandlerRGBAField (const PointCloudConstPtr &cloud)
          : PointCloudColorHandler<PointT> (cloud)
        {
          setInputCloud (cloud);
        }

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerRGBAField () {}

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgba"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRGBAField"); }

        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Label field handler class for colors. Paints the points according to their
      * labels, assigning a unique color from a predefined color lookup table to each label.
      * \author Sergey Alexandrov
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerLabelField : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerLabelField<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerLabelField<PointT> > ConstPtr;

        /** \brief Constructor.
          * \param[in] static_mapping Use a static colormapping from label_id to color (default true) */
        PointCloudColorHandlerLabelField (const bool static_mapping = true)
          : PointCloudColorHandler<PointT> ()
        {
          capable_ = false;
          static_mapping_ = static_mapping;
        }

        /** \brief Constructor.
          * \param[in] static_mapping Use a static colormapping from label_id to color (default true) */
        PointCloudColorHandlerLabelField (const PointCloudConstPtr &cloud,
                                          const bool static_mapping = true)
          : PointCloudColorHandler<PointT> (cloud)
        {
          setInputCloud (cloud);
          static_mapping_ = static_mapping;
        }

        /** \brief Destructor. */
        virtual ~PointCloudColorHandlerLabelField () {}

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("label"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerLabelField"); }

      private:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
        using PointCloudColorHandler<PointT>::field_idx_;
        using PointCloudColorHandler<PointT>::fields_;
        bool static_mapping_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Base Handler class for PointCloud colors.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      public:
        typedef pcl::PCLPointCloud2 PointCloud;
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
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;

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

        /** \brief The index of the field holding the data that represents the color. */
        int field_idx_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors (i.e., R, G, B will be randomly chosen)
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerRandom<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRandom<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRandom<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<pcl::PCLPointCloud2> (cloud)
        {
          capable_ = true;
        }
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerRandom () {}

        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for predefined user colors. The color at each point will be drawn
      * as the use given R, G, B values.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerCustom<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        /** \brief Constructor. */
        PointCloudColorHandlerCustom (const PointCloudConstPtr &cloud,
                                      double r, double g, double b) :
          PointCloudColorHandler<pcl::PCLPointCloud2> (cloud),
          r_ (r), g_ (g), b_ (b)
        {
          capable_ = true;
        }
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerCustom () {}

        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Internal R, G, B holding the values given by the user. */
        double r_, g_, b_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields as the color at each point.
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud);
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerRGBField () {}

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual std::string
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
    class PCL_EXPORTS PointCloudColorHandlerHSVField<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerHSVField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerHSVField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerHSVField () {}

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual std::string
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
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerGenericField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerGenericField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
                                            const std::string &field_name);
      
        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerGenericField () {}

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerGenericField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (field_name_); }

      private:
        /** \brief Name of the field used to create the color handler. */
        std::string field_name_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGBA handler class for colors. Uses the data present in the "rgba" field as
      * the color at each point. Transparency is handled.
      * \author Nizar Sallem
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerRGBAField<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBAField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBAField<PointCloud> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBAField (const PointCloudConstPtr &cloud);

        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerRGBAField () {}

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRGBAField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgba"); }
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Label field handler class for colors. Paints the points according to their
      * labels, assigning a unique color from a predefined color lookup table to each label.
      * \author Sergey Alexandrov
      * \ingroup visualization
      */
    template <>
    class PCL_EXPORTS PointCloudColorHandlerLabelField<pcl::PCLPointCloud2> : public PointCloudColorHandler<pcl::PCLPointCloud2>
    {
      typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerLabelField<PointCloud> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerLabelField<PointCloud> > ConstPtr;

        /** \brief Constructor.
          * \param[in] static_mapping Use a static colormapping from label_id to color (default true) */
        PointCloudColorHandlerLabelField (const PointCloudConstPtr &cloud,
                                          const bool static_mapping = true);

        /** \brief Empty destructor */
        virtual ~PointCloudColorHandlerLabelField () {}

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerLabelField"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("label"); }
    private:
        bool static_mapping_;
    };

  }
}

#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>

#endif      // PCL_POINT_CLOUD_COLOR_HANDLERS_H_

