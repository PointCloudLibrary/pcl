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
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h> // for PCLPointCloud2
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
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudColorHandler<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandler<PointT> >;

        /** \brief Constructor. */
        PointCloudColorHandler () :
          cloud_ (), capable_ (false), field_idx_ (-1), fields_ ()
        {}

        /** \brief Constructor. */
        PointCloudColorHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false), field_idx_ (-1), fields_ ()
        {}

        /** \brief Destructor. */
        virtual ~PointCloudColorHandler() = default;

        /** \brief Check if this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const = 0;

        /** Obtain the actual color for the input dataset as a VTK data array.
          * Deriving handlers should override this method.
          * \return smart pointer to VTK array if the operation was successful (the
          * handler is capable and the input cloud was given), a null pointer otherwise */
        virtual vtkSmartPointer<vtkDataArray>
        getColor () const = 0;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRandom<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRandom<PointT> >;

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

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerCustom<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerCustom<PointT> >;

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

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRGBField<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRGBField<PointT> >;

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

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgb"); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerHSVField<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerHSVField<PointT> >;

        /** \brief Constructor. */
        PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("hsv"); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerGenericField<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerGenericField<PointT> >;

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

        /** \brief Get the name of the field used. */
        virtual std::string getFieldName () const { return (field_name_); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRGBAField<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRGBAField<PointT> >;

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

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("rgba"); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRGBAField"); }

      private:
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
      using PointCloud = typename PointCloudColorHandler<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerLabelField<PointT> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerLabelField<PointT> >;

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

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("label"); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

        using PointCloudColorHandler<PointT>::getColor;

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
        using PointCloud = pcl::PCLPointCloud2;
        using PointCloudPtr = PointCloud::Ptr;
        using PointCloudConstPtr = PointCloud::ConstPtr;

        using Ptr = shared_ptr<PointCloudColorHandler<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandler<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false), field_idx_ ()
        {}
        
        /** \brief Destructor. */
        virtual ~PointCloudColorHandler() = default;

        /** \brief Return whether this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const = 0;

        /** Obtain the actual color for the input dataset as a VTK data array.
          * Deriving handlers should override this method. The default implementation is
          * provided only for backwards compatibility with handlers that were written
          * before PCL 1.10.0 and will be removed in future.
          * \return smart pointer to VTK array if the operation was successful (the
          * handler is capable and the input cloud was given), a null pointer otherwise */
        virtual vtkSmartPointer<vtkDataArray>
          getColor() const = 0;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRandom<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRandom<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<pcl::PCLPointCloud2> (cloud)
        {
          capable_ = true;
        }

        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;
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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        /** \brief Constructor. */
        PointCloudColorHandlerCustom (const PointCloudConstPtr &cloud,
                                      double r, double g, double b) :
          PointCloudColorHandler<pcl::PCLPointCloud2> (cloud),
          r_ (r), g_ (g), b_ (b)
        {
          capable_ = true;
        }

        /** \brief Get the name of the class. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerCustom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return (""); }

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRGBField<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRGBField<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud);

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerHSVField<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerHSVField<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerGenericField<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerGenericField<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
                                            const std::string &field_name);

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerRGBAField<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerRGBAField<PointCloud> >;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBAField (const PointCloudConstPtr &cloud);

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
      using PointCloud = PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
      using PointCloudPtr = PointCloud::Ptr;
      using PointCloudConstPtr = PointCloud::ConstPtr;

      public:
        using Ptr = shared_ptr<PointCloudColorHandlerLabelField<PointCloud> >;
        using ConstPtr = shared_ptr<const PointCloudColorHandlerLabelField<PointCloud> >;

        /** \brief Constructor.
          * \param[in] static_mapping Use a static colormapping from label_id to color (default true) */
        PointCloudColorHandlerLabelField (const PointCloudConstPtr &cloud,
                                          const bool static_mapping = true);

        vtkSmartPointer<vtkDataArray>
        getColor () const override;

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
