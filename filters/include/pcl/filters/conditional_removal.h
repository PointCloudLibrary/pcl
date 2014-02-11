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

#ifndef PCL_FILTER_FIELD_VAL_CONDITION_H_
#define PCL_FILTER_FIELD_VAL_CONDITION_H_
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////
  namespace ComparisonOps
  {
    /** \brief The kind of comparison operations that are possible within a 
      * comparison object
      */
    typedef enum
    {
      GT, GE, LT, LE, EQ
    } CompareOp;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief A datatype that enables type-correct comparisons. */
  template<typename PointT>
  class PointDataAtOffset
  {
    public:
      /** \brief Constructor. */
      PointDataAtOffset (uint8_t datatype, uint32_t offset) :
        datatype_ (datatype), offset_ (offset)
      {
      }

      /** \brief Compare function. 
        * \param p the point to compare
        * \param val the value to compare the point to
        */
      int
      compare (const PointT& p, const double& val);
    protected:
      /** \brief The type of data. */
      uint8_t datatype_;

      /** \brief The data offset. */
      uint32_t offset_;
    private:
      PointDataAtOffset () : datatype_ (), offset_ () {}
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief The (abstract) base class for the comparison object. */
  template<typename PointT>
  class ComparisonBase
  {
    public:
      typedef boost::shared_ptr< ComparisonBase<PointT> > Ptr;
      typedef boost::shared_ptr< const ComparisonBase<PointT> > ConstPtr;

      /** \brief Constructor. */
      ComparisonBase () : capable_ (false), field_name_ (), offset_ (), op_ () {}

      /** \brief Destructor. */
      virtual ~ComparisonBase () {}

      /** \brief Return if the comparison is capable. */
      inline bool
      isCapable () const
      {
        return (capable_);
      }

      /** \brief Evaluate function. */
      virtual bool
      evaluate (const PointT &point) const = 0;

    protected:
      /** \brief True if capable. */
      bool capable_;

      /** \brief Field name to compare data on. */
      std::string field_name_;

      /** \brief The data offset. */
      uint32_t offset_;

      /** \brief The comparison operator type. */
      ComparisonOps::CompareOp op_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief The field-based specialization of the comparison object. */
  template<typename PointT>
  class FieldComparison : public ComparisonBase<PointT>
  {
    using ComparisonBase<PointT>::field_name_;
    using ComparisonBase<PointT>::op_;
    using ComparisonBase<PointT>::capable_;

    public:
      typedef boost::shared_ptr< FieldComparison<PointT> > Ptr;
      typedef boost::shared_ptr< const FieldComparison<PointT> > ConstPtr;


      /** \brief Construct a FieldComparison
        * \param field_name the name of the field that contains the data we want to compare
        * \param op the operator to use when making the comparison
        * \param compare_val the constant value to compare the field value too
        */
      FieldComparison (std::string field_name, ComparisonOps::CompareOp op, double compare_val);

      /** \brief Copy constructor.
        * \param[in] src the field comparison object to copy into this
        */
      FieldComparison (const FieldComparison &src) 
        : ComparisonBase<PointT> ()
        , compare_val_ (src.compare_val_), point_data_ (src.point_data_)
      {
      }

      /** \brief Copy operator.
        * \param[in] src the field comparison object to copy into this
        */
      inline FieldComparison&
      operator = (const FieldComparison &src)
      {
        compare_val_ = src.compare_val_;
        point_data_  = src.point_data_;
        return (*this);
      }

      /** \brief Destructor. */
      virtual ~FieldComparison ();

      /** \brief Determine the result of this comparison.  
        * \param point the point to evaluate
        * \return the result of this comparison.
        */
      virtual bool
      evaluate (const PointT &point) const;

    protected:
      /** \brief All types (that we care about) can be represented as a double. */
      double compare_val_;

      /** \brief The point data to compare. */
      PointDataAtOffset<PointT>* point_data_;

    private:
      FieldComparison () :
        compare_val_ (), point_data_ ()
      {
      } // not allowed
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief A packed rgb specialization of the comparison object. */
  template<typename PointT>
  class PackedRGBComparison : public ComparisonBase<PointT>
  {
    using ComparisonBase<PointT>::capable_;
    using ComparisonBase<PointT>::op_;

    public:
      typedef boost::shared_ptr< PackedRGBComparison<PointT> > Ptr;
      typedef boost::shared_ptr< const PackedRGBComparison<PointT> > ConstPtr;

      /** \brief Construct a PackedRGBComparison
        * \param component_name either "r", "g" or "b"
        * \param op the operator to use when making the comparison
        * \param compare_val the constant value to compare the component value too
        */
      PackedRGBComparison (std::string component_name, ComparisonOps::CompareOp op, double compare_val);

      /** \brief Destructor. */
      virtual ~PackedRGBComparison () {}

      /** \brief Determine the result of this comparison.  
        * \param point the point to evaluate
        * \return the result of this comparison.
        */
      virtual bool
      evaluate (const PointT &point) const;

    protected:
      /** \brief The name of the component. */
      std::string component_name_;

      /** \brief The offset of the component */
      uint32_t component_offset_;

      /** \brief All types (that we care about) can be represented as a double. */
      double compare_val_;

    private:
      PackedRGBComparison () :
        component_name_ (), component_offset_ (), compare_val_ ()
      {
      } // not allowed

  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief A packed HSI specialization of the comparison object. */
  template<typename PointT>
  class PackedHSIComparison : public ComparisonBase<PointT>
  {
    using ComparisonBase<PointT>::capable_;
    using ComparisonBase<PointT>::op_;

    public:
      typedef boost::shared_ptr< PackedHSIComparison<PointT> > Ptr;
      typedef boost::shared_ptr< const PackedHSIComparison<PointT> > ConstPtr;
 
      /** \brief Construct a PackedHSIComparison 
        * \param component_name either "h", "s" or "i"
        * \param op the operator to use when making the comparison
        * \param compare_val the constant value to compare the component value too
        */
      PackedHSIComparison (std::string component_name, ComparisonOps::CompareOp op, double compare_val);

      /** \brief Destructor. */
      virtual ~PackedHSIComparison () {}

      /** \brief Determine the result of this comparison.  
        * \param point the point to evaluate
        * \return the result of this comparison.
        */
      virtual bool
      evaluate (const PointT &point) const;

      typedef enum
      {
        H, // -128 to 127 corresponds to -pi to pi
        S, // 0 to 255
        I  // 0 to 255
      } ComponentId;

    protected:
      /** \brief The name of the component. */
      std::string component_name_;

      /** \brief The ID of the component. */
      ComponentId component_id_;

      /** \brief All types (that we care about) can be represented as a double. */
      double compare_val_;

      /** \brief The offset of the component */
      uint32_t rgb_offset_;

    private:
      PackedHSIComparison () :
        component_name_ (), component_id_ (), compare_val_ (), rgb_offset_ ()
      {
      } // not allowed
  };
  
  //////////////////////////////////////////////////////////////////////////////////////////
  /**\brief A comparison whether the (x,y,z) components of a given point satisfy (p'Ap + 2v'p + c [OP] 0).
   * Here [OP] stands for the defined pcl::ComparisonOps, i.e. for GT, GE, LT, LE or EQ;
   * p = (x,y,z) is a point of the point cloud; A is 3x3 matrix; v is the 3x1 vector; c is a scalar.
   *  
   * One can also use TfQuadraticXYZComparison for simpler geometric shapes by defining the
   * quadratic parts (i.e. the matrix A) to be zero. By combining different instances of
   * TfQuadraticXYZComparison one can get more complex shapes. For example, to have a simple
   * cylinder (along the x-axis) of specific length one needs three comparisons combined as AND condition:
   *   1. The cylinder: A = [0 0 0, 0 1 0, 0 0 1]; v = [0, 0, 0]; c = radius²; OP = LT (meaning "<")
   *   2. X-min limit: A = 0; v = [1, 0, 0]; c = x_min; OP = GT
   *   3. X-max ...
   *
   * \author Julian Löchner
   */
  template<typename PointT>
  class TfQuadraticXYZComparison : public pcl::ComparisonBase<PointT>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     //needed whenever there is a fixed size Eigen:: vector or matrix in a class

      typedef boost::shared_ptr<TfQuadraticXYZComparison<PointT> > Ptr;
      typedef boost::shared_ptr<const TfQuadraticXYZComparison<PointT> > ConstPtr;

      /** \brief Constructor.
       */
      TfQuadraticXYZComparison ();
      
      /** \brief Empty destructor */
      virtual ~TfQuadraticXYZComparison () {}

      /** \brief Constructor.
       * \param op the operator "[OP]" of the comparison "p'Ap + 2v'p + c [OP] 0".
       * \param comparison_matrix the matrix "A" of the comparison "p'Ap + 2v'p + c [OP] 0".
       * \param comparison_vector the vector "v" of the comparison "p'Ap + 2v'p + c [OP] 0".
       * \param comparison_scalar the scalar "c" of the comparison "p'Ap + 2v'p + c [OP] 0".
       * \param comparison_transform the transformation of the comparison.
       */
      TfQuadraticXYZComparison (const pcl::ComparisonOps::CompareOp op, const Eigen::Matrix3f &comparison_matrix,
                                const Eigen::Vector3f &comparison_vector, const float &comparison_scalar,
                                const Eigen::Affine3f &comparison_transform = Eigen::Affine3f::Identity ());

      /** \brief set the operator "[OP]" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonOperator (const pcl::ComparisonOps::CompareOp op)
      {
        op_ = op;
      }

      /** \brief set the matrix "A" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonMatrix (const Eigen::Matrix3f &matrix)
      {
        //define comp_matr_ as an homogeneous matrix of the given matrix
        comp_matr_.block<3, 3> (0, 0) = matrix;
        comp_matr_.col (3) << 0, 0, 0, 1;
        comp_matr_.block<1, 3> (3, 0) << 0, 0, 0;
        tf_comp_matr_ = comp_matr_;
      }

      /** \brief set the matrix "A" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonMatrix (const Eigen::Matrix4f &homogeneousMatrix)
      {
        comp_matr_ = homogeneousMatrix;
        tf_comp_matr_ = comp_matr_;
      }

      /** \brief set the vector "v" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonVector (const Eigen::Vector3f &vector)
      {
        comp_vect_ = vector.homogeneous ();
        tf_comp_vect_ = comp_vect_;
      }

      /** \brief set the vector "v" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonVector (const Eigen::Vector4f &homogeneousVector)
      {
        comp_vect_ = homogeneousVector;
        tf_comp_vect_ = comp_vect_;
      }

      /** \brief set the scalar "c" of the comparison "p'Ap + 2v'p + c [OP] 0".
       */
      inline void
      setComparisonScalar (const float &scalar)
      {
        comp_scalar_ = scalar;
      }

      /** \brief transform the coordinate system of the comparison. If you think of
       * the transformation to be a translation and rotation of the comparison in the
       * same coordinate system, you have to provide the inverse transformation.
       * This function does not change the original definition of the comparison. Thus,
       * each call of this function will assume the original definition of the comparison
       * as starting point for the transformation.
       *
       * @param transform the transformation (rotation and translation) as an affine matrix.
       */
      inline void
      transformComparison (const Eigen::Matrix4f &transform)
      {
        tf_comp_matr_ = transform.transpose () * comp_matr_ * transform;
        tf_comp_vect_ = comp_vect_.transpose () * transform;
      }

      /** \brief transform the coordinate system of the comparison. If you think of
       * the transformation to be a translation and rotation of the comparison in the
       * same coordinate system, you have to provide the inverse transformation.
       * This function does not change the original definition of the comparison. Thus,
       * each call of this function will assume the original definition of the comparison
       * as starting point for the transformation.
       *
       * @param transform the transformation (rotation and translation) as an affine matrix.
       */
      inline void
      transformComparison (const Eigen::Affine3f &transform)
      {
        transformComparison (transform.matrix ());
      }

      /** \brief Determine the result of this comparison.
       * \param point the point to evaluate
       * \return the result of this comparison.
       */
      virtual bool
      evaluate (const PointT &point) const;

    protected:
      using pcl::ComparisonBase<PointT>::capable_;
      using pcl::ComparisonBase<PointT>::op_;

      Eigen::Matrix4f comp_matr_;
      Eigen::Vector4f comp_vect_;

      float comp_scalar_;

    private:
      Eigen::Matrix4f tf_comp_matr_;
      Eigen::Vector4f tf_comp_vect_;
  };
  
  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Base condition class. */
  template<typename PointT>
  class ConditionBase
  {
    public:
      typedef typename pcl::ComparisonBase<PointT> ComparisonBase;
      typedef typename ComparisonBase::Ptr ComparisonBasePtr;
      typedef typename ComparisonBase::ConstPtr ComparisonBaseConstPtr;

      typedef boost::shared_ptr<ConditionBase<PointT> > Ptr;
      typedef boost::shared_ptr<const ConditionBase<PointT> > ConstPtr;

      /** \brief Constructor. */
      ConditionBase () : capable_ (true), comparisons_ (), conditions_ ()
      {
      }

      /** \brief Destructor. */
      virtual ~ConditionBase ()
      {
        // comparisons are boost::shared_ptr.will take care of themselves
        comparisons_.clear ();

        // conditions are boost::shared_ptr. will take care of themselves
        conditions_.clear ();
      }

      /** \brief Add a new comparison
        * \param comparison the comparison operator to add
        */
      void
      addComparison (ComparisonBaseConstPtr comparison);

      /** \brief Add a nested condition to this condition.  
        * \param condition the nested condition to be added
        */
      void
      addCondition (Ptr condition);

      /** \brief Check if evaluation requirements are met. */
      inline bool
      isCapable () const
      {
        return (capable_);
      }

      /** \brief Determine if a point meets this condition.  
        * \return whether the point meets this condition.
        */
      virtual bool
      evaluate (const PointT &point) const = 0;

    protected:
      /** \brief True if capable. */
      bool capable_;

      /** \brief The collection of all comparisons that need to be verified. */
      std::vector<ComparisonBaseConstPtr> comparisons_;

      /** \brief The collection of all conditions that need to be verified. */
      std::vector<Ptr> conditions_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief AND condition. */
  template<typename PointT>
  class ConditionAnd : public ConditionBase<PointT>
  {
    using ConditionBase<PointT>::conditions_;
    using ConditionBase<PointT>::comparisons_;

    public:
      typedef boost::shared_ptr<ConditionAnd<PointT> > Ptr;
      typedef boost::shared_ptr<const ConditionAnd<PointT> > ConstPtr;

      /** \brief Constructor. */
      ConditionAnd () :
        ConditionBase<PointT> ()
      {
      }

      /** \brief Determine if a point meets this condition.  
        * \return whether the point meets this condition.
        *
        * The ConditionAnd evaluates to true when ALL
        * comparisons and nested conditions evaluate to true
        */
      virtual bool
      evaluate (const PointT &point) const;
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief OR condition. */
  template<typename PointT>
  class ConditionOr : public ConditionBase<PointT>
  {
    using ConditionBase<PointT>::conditions_;
    using ConditionBase<PointT>::comparisons_;

    public:
      typedef boost::shared_ptr<ConditionOr<PointT> > Ptr;
      typedef boost::shared_ptr<const ConditionOr<PointT> > ConstPtr;

      /** \brief Constructor. */
      ConditionOr () :
        ConditionBase<PointT> ()
      {
      }

      /** \brief Determine if a point meets this condition.  
        * \return whether the point meets this condition.
        *
        * The ConditionOr evaluates to true when ANY
        * comparisons or nested conditions evaluate to true
        */
      virtual bool
      evaluate (const PointT &point) const;
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConditionalRemoval filters data that satisfies certain conditions.
    *
    * A ConditionalRemoval must be provided a condition. There are two types of
    * conditions: ConditionAnd and ConditionOr. Conditions require one or more
    * comparisons and/or other conditions. A comparison has a name, a
    * comparison operator, and a value.
    *
    * An ConditionAnd will evaluate to true when ALL of its encapsulated
    * comparisons and conditions are true.
    *
    * An ConditionOr will evaluate to true when ANY of its encapsulated
    * comparisons and conditions are true.
    *
    * Depending on the derived type of the comparison, the name can correspond
    * to a PointCloud field name, or a color component in rgb color space or
    * hsi color space.
    *
    * Here is an example usage:
    *  // Build the condition
    *  pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    *  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, 2.0)));
    *  range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0)));
    *  // Build the filter
    *  pcl::ConditionalRemoval<PointT> range_filt;
    *  range_filt.setCondition (range_cond);
    *  range_filt.setKeepOrganized (false);
    *
    * \author Louis LeGrand, Intel Labs Seattle
    * \ingroup filters
    */
  template<typename PointT>
  class ConditionalRemoval : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    using Filter<PointT>::removed_indices_;
    using Filter<PointT>::extract_removed_indices_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      typedef typename pcl::ConditionBase<PointT> ConditionBase;
      typedef typename ConditionBase::Ptr ConditionBasePtr;
      typedef typename ConditionBase::ConstPtr ConditionBaseConstPtr;

      /** \brief the default constructor.  
        *
        * All ConditionalRemovals require a condition which can be set
        * using the setCondition method
        * \param extract_removed_indices extract filtered indices from indices vector
        */
      ConditionalRemoval (int extract_removed_indices = false) :
        Filter<PointT>::Filter (extract_removed_indices), capable_ (false), keep_organized_ (false), condition_ (),
        user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
        filter_name_ = "ConditionalRemoval";
      }

      /** \brief a constructor that includes the condition.  
        * \param condition the condition that each point must satisfy to avoid
        * being removed by the filter
        * \param extract_removed_indices extract filtered indices from indices vector
        */
      ConditionalRemoval (ConditionBasePtr condition, bool extract_removed_indices = false) :
        Filter<PointT>::Filter (extract_removed_indices), capable_ (false), keep_organized_ (false), condition_ (),
        user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
        filter_name_ = "ConditionalRemoval";
        setCondition (condition);
      }

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void
      setKeepOrganized (bool val)
      {
        keep_organized_ = val;
      }

      inline bool
      getKeepOrganized () const
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param val the user given value that the filtered point dimensions should be set to
        */
      inline void
      setUserFilterValue (float val)
      {
        user_filter_value_ = val;
      }

      /** \brief Set the condition that the filter will use.  
        * \param condition each point must satisfy this condition to avoid
        * being removed by the filter
        *
        * All ConditionalRemovals require a condition
        */
      void
      setCondition (ConditionBasePtr condition);

    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      /** \brief True if capable. */
      bool capable_;

      /** \brief Keep the structure of the data organized, by setting the
        * filtered points to the a user given value (NaN by default).
        */
      bool keep_organized_;

      /** \brief The condition to use for filtering */
      ConditionBasePtr condition_;

      /** \brief User given value to be set to any filtered point. Casted to
        * the correct field type. 
        */
      float user_filter_value_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/conditional_removal.hpp>
#endif

#endif 
