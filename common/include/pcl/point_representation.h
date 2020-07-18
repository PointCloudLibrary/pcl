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
 * $Id$
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/for_each_type.h>

namespace pcl
{
  /** \brief @b PointRepresentation provides a set of methods for converting a point structs/object into an
    * n-dimensional vector.
    * \note This is an abstract class.  Subclasses must set nr_dimensions_ to the appropriate value in the constructor
    * and provide an implementation of the pure virtual copyToFloatArray method.
    * \author Michael Dixon
    */
  template <typename PointT>
  class PointRepresentation
  {
    protected:
      /** \brief The number of dimensions in this point's vector (i.e. the "k" in "k-D") */
      int nr_dimensions_ = 0;
      /** \brief A vector containing the rescale factor to apply to each dimension. */
      std::vector<float> alpha_;
      /** \brief Indicates whether this point representation is trivial. It is trivial if and only if the following
       *  conditions hold:
       *  - the relevant data consists only of float values
       *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
       *  - sizeof(PointT) is a multiple of sizeof(float)
       *  In short, a trivial point representation converts the input point to a float array that is the same as if
       *  the point was reinterpret_casted to a float array of length nr_dimensions_ . This value says that this
       *  representation can be trivial; it is only trivial if setRescaleValues() has not been set.
       */
      bool trivial_ = false;

    public:
      using Ptr = shared_ptr<PointRepresentation<PointT> >;
      using ConstPtr = shared_ptr<const PointRepresentation<PointT> >;

      /** \brief Empty destructor */
      virtual ~PointRepresentation () = default;
      //TODO: check if copy and move constructors / assignment operators are needed

      /** \brief Copy point data from input point to a float array. This method must be overridden in all subclasses.
       *  \param[in] p The input point
       *  \param[out] out A pointer to a float array.
       */
      virtual void copyToFloatArray (const PointT &p, float *out) const = 0;

      /** \brief Returns whether this point representation is trivial. It is trivial if and only if the following
       *  conditions hold:
       *  - the relevant data consists only of float values
       *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
       *  - sizeof(PointT) is a multiple of sizeof(float)
       *  In short, a trivial point representation converts the input point to a float array that is the same as if
       *  the point was reinterpret_casted to a float array of length nr_dimensions_ . */
      inline bool isTrivial() const { return trivial_ && alpha_.empty (); }

      /** \brief Verify that the input point is valid.
       *  \param p The point to validate
       */
      virtual bool
      isValid (const PointT &p) const
      {
        bool is_valid = true;

        if (trivial_)
        {
          const float* temp = reinterpret_cast<const float*>(&p);

          for (int i = 0; i < nr_dimensions_; ++i)
          {
            if (!std::isfinite (temp[i]))
            {
              is_valid = false;
              break;
            }
          }
        }
        else
        {
          float *temp = new float[nr_dimensions_];
          copyToFloatArray (p, temp);

          for (int i = 0; i < nr_dimensions_; ++i)
          {
            if (!std::isfinite (temp[i]))
            {
              is_valid = false;
              break;
            }
          }
          delete [] temp;
        }
        return (is_valid);
      }

      /** \brief Convert input point into a vector representation, rescaling by \a alpha.
        * \param[in] p the input point
        * \param[out] out The output vector.  Can be of any type that implements the [] operator.
        */
      template <typename OutputType> void
      vectorize (const PointT &p, OutputType &out) const
      {
        float *temp = new float[nr_dimensions_];
        copyToFloatArray (p, temp);
        if (alpha_.empty ())
        {
          for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = temp[i];
        }
        else
        {
          for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = temp[i] * alpha_[i];
        }
        delete [] temp;
      }

      /** \brief Set the rescale values to use when vectorizing points
        * \param[in] rescale_array The array/vector of rescale values.  Can be of any type that implements the [] operator.
        */
      void
      setRescaleValues (const float *rescale_array)
      {
        alpha_.resize (nr_dimensions_);
        std::copy_n(rescale_array, nr_dimensions_, alpha_.begin());
      }

      /** \brief Return the number of dimensions in the point's vector representation. */
      inline int getNumberOfDimensions () const { return (nr_dimensions_); }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b DefaultPointRepresentation extends PointRepresentation to define default behavior for common point types.
   */
  template <typename PointDefault>
  class DefaultPointRepresentation : public PointRepresentation <PointDefault>
  {
    using PointRepresentation <PointDefault>::nr_dimensions_;
    using PointRepresentation <PointDefault>::trivial_;

    public:
      // Boost shared pointers
      using Ptr = shared_ptr<DefaultPointRepresentation<PointDefault> >;
      using ConstPtr = shared_ptr<const DefaultPointRepresentation<PointDefault> >;

      DefaultPointRepresentation ()
      {
        // If point type is unknown, assume it's a struct/array of floats, and compute the number of dimensions
        nr_dimensions_ = sizeof (PointDefault) / sizeof (float);
        // Limit the default representation to the first 3 elements
        if (nr_dimensions_ > 3) nr_dimensions_ = 3;

        trivial_ = true;
      }

      ~DefaultPointRepresentation () {}

      inline Ptr
      makeShared () const
      {
        return (Ptr (new DefaultPointRepresentation<PointDefault> (*this)));
      }

      void
      copyToFloatArray (const PointDefault &p, float * out) const override
      {
        // If point type is unknown, treat it as a struct/array of floats
        const float* ptr = reinterpret_cast<const float*> (&p);
        std::copy_n(ptr, nr_dimensions_, out);
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b DefaulFeatureRepresentation extends PointRepresentation and is intended to be used when defining the
    * default behavior for feature descriptor types (i.e., copy each element of each field into a float array).
    */
  template <typename PointDefault>
  class DefaultFeatureRepresentation : public PointRepresentation <PointDefault>
  {
    protected:
      using PointRepresentation <PointDefault>::nr_dimensions_;

    private:
      struct IncrementFunctor
      {
        IncrementFunctor (int &n) : n_ (n)
        {
          n_ = 0;
        }

        template<typename Key> inline void operator () ()
        {
          n_ += pcl::traits::datatype<PointDefault, Key>::size;
        }

      private:
        int &n_;
      };

    struct NdCopyPointFunctor
    {
      using Pod = typename traits::POD<PointDefault>::type;

      NdCopyPointFunctor (const PointDefault &p1, float * p2)
        : p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

      template<typename Key> inline void operator() ()
      {
        using FieldT = typename pcl::traits::datatype<PointDefault, Key>::type;
        const int NrDims = pcl::traits::datatype<PointDefault, Key>::size;
        Helper<Key, FieldT, NrDims>::copyPoint (p1_, p2_, f_idx_);
      }

      // Copy helper for scalar fields
      template <typename Key, typename FieldT, int NrDims>
      struct Helper
      {
        static void copyPoint (const Pod &p1, float * p2, int &f_idx)
        {
          const std::uint8_t * data_ptr = reinterpret_cast<const std::uint8_t *> (&p1) +
            pcl::traits::offset<PointDefault, Key>::value;
          p2[f_idx++] = *reinterpret_cast<const FieldT*> (data_ptr);
        }
      };
      // Copy helper for array fields
      template <typename Key, typename FieldT, int NrDims>
      struct Helper<Key, FieldT[NrDims], NrDims>
      {
        static void copyPoint (const Pod &p1, float * p2, int &f_idx)
        {
          const std::uint8_t * data_ptr = reinterpret_cast<const std::uint8_t *> (&p1) +
            pcl::traits::offset<PointDefault, Key>::value;
          int nr_dims = NrDims;
          const FieldT * array = reinterpret_cast<const FieldT *> (data_ptr);
          for (int i = 0; i < nr_dims; ++i)
          {
            p2[f_idx++] = array[i];
          }
        }
      };

    private:
      const Pod &p1_;
      float * p2_;
      int f_idx_;
    };

    public:
      // Boost shared pointers
      using Ptr = shared_ptr<DefaultFeatureRepresentation<PointDefault>>;
      using ConstPtr = shared_ptr<const DefaultFeatureRepresentation<PointDefault>>;
      using FieldList = typename pcl::traits::fieldList<PointDefault>::type;

      DefaultFeatureRepresentation ()
      {
        nr_dimensions_ = 0; // zero-out the nr_dimensions_ before it gets incremented
        pcl::for_each_type <FieldList> (IncrementFunctor (nr_dimensions_));
      }

      inline Ptr
      makeShared () const
      {
        return (Ptr (new DefaultFeatureRepresentation<PointDefault> (*this)));
      }

      void
      copyToFloatArray (const PointDefault &p, float * out) const override
      {
        pcl::for_each_type <FieldList> (NdCopyPointFunctor (p, out));
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PointXYZ> : public  PointRepresentation <PointXYZ>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 3;
        trivial_ = true;
      }

      void
      copyToFloatArray (const PointXYZ &p, float * out) const override
      {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PointXYZI> : public  PointRepresentation <PointXYZI>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 3;
        trivial_ = true;
      }

      void
      copyToFloatArray (const PointXYZI &p, float * out) const override
      {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        // By default, p.intensity is not part of the PointXYZI vectorization
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PointNormal> : public  PointRepresentation <PointNormal>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 3;
        trivial_ = true;
      }

      void
      copyToFloatArray (const PointNormal &p, float * out) const override
      {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PFHSignature125> : public DefaultFeatureRepresentation <PFHSignature125>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PFHRGBSignature250> : public DefaultFeatureRepresentation <PFHRGBSignature250>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <PPFSignature> : public DefaultFeatureRepresentation <PPFSignature>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 4;
        trivial_ = true;
      }

      void
      copyToFloatArray (const PPFSignature &p, float * out) const override
      {
        out[0] = p.f1;
        out[1] = p.f2;
        out[2] = p.f3;
        out[3] = p.f4;
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <FPFHSignature33> : public DefaultFeatureRepresentation <FPFHSignature33>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <VFHSignature308> : public DefaultFeatureRepresentation <VFHSignature308>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <GASDSignature512> : public DefaultFeatureRepresentation <GASDSignature512>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <GASDSignature984> : public DefaultFeatureRepresentation <GASDSignature984>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <GASDSignature7992> : public DefaultFeatureRepresentation <GASDSignature7992>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation <Narf36> : public PointRepresentation <Narf36>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 36;
        trivial_=false;
      }

      void
      copyToFloatArray (const Narf36 &p, float * out) const override
      {
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = p.descriptor[i];
      }
  };
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation<NormalBasedSignature12> : public DefaultFeatureRepresentation <NormalBasedSignature12>
  {};

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation<ShapeContext1980> : public PointRepresentation<ShapeContext1980>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 1980;
      }

      void
      copyToFloatArray (const ShapeContext1980 &p, float * out) const override
      {
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = p.descriptor[i];
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation<UniqueShapeContext1960> : public PointRepresentation<UniqueShapeContext1960>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 1960;
      }

      void
      copyToFloatArray (const UniqueShapeContext1960 &p, float * out) const override
      {
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = p.descriptor[i];
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation<SHOT352> : public PointRepresentation<SHOT352>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 352;
      }

      void
      copyToFloatArray (const SHOT352 &p, float * out) const override
      {
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = p.descriptor[i];
      }
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class DefaultPointRepresentation<SHOT1344> : public PointRepresentation<SHOT1344>
  {
    public:
      DefaultPointRepresentation ()
      {
        nr_dimensions_ = 1344;
      }

      void
      copyToFloatArray (const SHOT1344 &p, float * out) const override
      {
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = p.descriptor[i];
      }
  };


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b CustomPointRepresentation extends PointRepresentation to allow for sub-part selection on the point.
   */
  template <typename PointDefault>
  class CustomPointRepresentation : public PointRepresentation <PointDefault>
  {
    using PointRepresentation <PointDefault>::nr_dimensions_;

    public:
      // Boost shared pointers
      using Ptr = shared_ptr<CustomPointRepresentation<PointDefault> >;
      using ConstPtr = shared_ptr<const CustomPointRepresentation<PointDefault> >;

      /** \brief Constructor
        * \param[in] max_dim the maximum number of dimensions to use
        * \param[in] start_dim the starting dimension
        */
      CustomPointRepresentation (const int max_dim = 3, const int start_dim = 0)
        : max_dim_(max_dim), start_dim_(start_dim)
      {
        // If point type is unknown, assume it's a struct/array of floats, and compute the number of dimensions
        nr_dimensions_ = static_cast<int> (sizeof (PointDefault) / sizeof (float)) - start_dim_;
        // Limit the default representation to the first 3 elements
        if (nr_dimensions_ > max_dim_)
          nr_dimensions_ = max_dim_;
      }

      inline Ptr
      makeShared () const
      {
        return Ptr (new CustomPointRepresentation<PointDefault> (*this));
      }

      /** \brief Copy the point data into a float array
        * \param[in] p the input point
        * \param[out] out the resultant output array
        */
      virtual void
      copyToFloatArray (const PointDefault &p, float *out) const
      {
        // If point type is unknown, treat it as a struct/array of floats
        const float *ptr = (reinterpret_cast<const float*> (&p)) + start_dim_;
        std::copy_n(ptr, nr_dimensions_, out);
      }

    protected:
      /** \brief Use at most this many dimensions (i.e. the "k" in "k-D" is at most max_dim_) -- \note float fields are assumed */
      int max_dim_;
      /** \brief Use dimensions only starting with this one (i.e. the "k" in "k-D" is = dim - start_dim_) -- \note float fields are assumed */
      int start_dim_;
  };
}
