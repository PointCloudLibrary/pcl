/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_SIMULATION_SHAPE_GENERATOR_COMPLEX_H_
#define PCL_SIMULATION_SHAPE_GENERATOR_COMPLEX_H_

#include <pcl/simulation/shape_generator_base.h>

namespace pcl
{
  namespace simulation
  {
    /** \brief A class which allows the creation of complex objects by assembling multiple shapes.
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \note Use the constructor to combine multipe shapes represented by a vector of pointers or two seperate pointer to the abstract base class GeometricShapeBase deleting points which are in the overlap.
      * \note The MultiShape's center will equal to (0,0,0), since we are combining multiple shapes with different centers.
      * \ingroup simulation
      */
    class MultiShape : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<MultiShape> Ptr;

        /** \brief Option how the labels of the parts are treated.
          * Assume three shapes consisting of 3 and 4 and 2 parts with part labels (0,1,2) + (0,1,2,3) + (0,1) are combined.
          */
        enum LabelHandling
        {
          /** All parts will get a single label.
            * (0,1,2) + (0,1,2,3) + (0,1) --> (0,0,0)(0,0,0,0)(0,0) */
          Single,
          /** Every shape gets its unique label. If shape-parts already have different labels they will be merged (default behaviour).
            * (0,1,2) + (0,1,2,3) + (0,1) --> (0,0,0)(1,1,1,1)(2,2) */
          Merge,
          /** Keep all part-labels unique, thus append new labels if shapes have multiple labels already.
            * (0,1,2) + (0,1,2,3) + (0,1) --> (0,1,2)(3,4,5,6)(7,8) */
          Append,
          /** Preserve the labeling of the shapes.
            * (0,1,2) + (0,1,2,3) + (0,1) --> (0,1,2)(0,1,2,3)(0,1) */
          Preserve
        };
        
        /** \brief Constructor to generate a MultiShape object from several GeometricShape Pointers.
          * \param[in] shape_ptrs Vector of GeometricShapeBase::Ptr used to set the input shapes which should be combined to a complex MultiShape
          * \param[in] delete_overlap If true we remove points of the shapes which lie in an overlapping region of any two shapes. Default true.
          * \param[in] label_handling Define the way labels of the parts are handled. See enum LabelHandling for details. Dafault: Merge.
          * \note The shapes used in the constructor will be modified.
          */
        MultiShape (GeometricShapePtrVector shape_ptrs,
                    bool delete_overlap = true,
                    LabelHandling label_handling = Merge) :
          shapes_ptrs_ (shape_ptrs),
          delete_overlap_ (delete_overlap),
          label_handling_ (label_handling)
        {
        };

        /** \brief Constructor to generate a MultiShape object from two GeometricShapes.
          * \param[in] shape1 First shape
          * \param[in] shape2 Second shape
          * \param[in] delete_overlap If true we remove points of the shapes which lie in an overlapping region of any two shapes. Default true.
          * \param[in] label_handling Define the way labels of the parts are handled. See enum LabelHandling for details. Dafault: Merge.
          * \note The shapes used in the constructor will be modified.
          * \note If more than two shapes should be combined use the constructor taking GeometricShapePtrVector as a argument.
          */
        MultiShape (GeometricShapeBase::Ptr shape1,
                    GeometricShapeBase::Ptr shape2,
                    bool delete_overlap = true,
                    LabelHandling label_handling = Merge) :
          delete_overlap_ (delete_overlap),
          label_handling_ (label_handling)
        {
          shapes_ptrs_.clear ();
          shapes_ptrs_.push_back (shape1);
          shapes_ptrs_.push_back (shape2);
        }
        
        /** \brief Generates points for shape.
          * \param[in] resolution The density of points per square unit
          * \returns A shared pointer to the generated cloud
          * \note Generats points on all shapes in shapes_ptrs_ and combines them.
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      protected:
        /** \brief The vector of pointers of the underlying geometric shapes.*/
        GeometricShapePtrVector shapes_ptrs_;

        /** \brief If this variable is true we prune points which are in the overlap of any two shapes */
        bool delete_overlap_;

        /** \brief The way labels are handled*/
        LabelHandling label_handling_;

        /** \brief An empty protected default constructor which is needed for the RecipeFile class. This constructor is not exposed to the user. */
        MultiShape ()
        {
        }

        /** \brief Check if point lies within any of the shapes
          * \param[in] point The query point
          * \returns True if the query point lies within any of the shapes used in the constructor
          */
        bool
        isInside (const PointT &point) const;
    };

    /** \brief A class which allows the creation of hollow shapes / cavities.
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \note This shape will have a single label.
      * \ingroup simulation
      */
    class CutShape : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<CutShape> Ptr;

        /** \brief Constructor to generate a Cutshape from to GeometricShapeBase::Ptrs.
          * \param[in] outer_shape_ptr This points to the object which will be cut.
          * \param[in] inner_shape_ptr This points to the object which will cut the outer shape.
          * \note The shapes used in the constructor will be modified.
          * \note The CutShape's center will equal the outer object's center.
          */
        CutShape (GeometricShapeBase::Ptr outer_shape_ptr,
                  GeometricShapeBase::Ptr inner_shape_ptr) :
          outer_shape_ptr_ (outer_shape_ptr),
          inner_shape_ptr_ (inner_shape_ptr)
        {
          centroid_ = outer_shape_ptr_->centroid_;
        }
        /** \brief Generates points on the shape.
          * \param[in] resolution The density of points per square unit
          * \returns A shared pointer to the generated cloud
          * \note Removes all outer shape points which are in the inner shape. Flips inner shape points and removes all inner shape points which are outside the outer shape.
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
      protected:
        /** \brief outer_shape_ptr_ is a pointer to the shape which will be cut by the shape in inner_shape_ptr_*/
        GeometricShapeBase::Ptr outer_shape_ptr_, inner_shape_ptr_;

        /** \brief Returns true if the query point lies within outer_shape_ptr_ but not within inner_shape_ptr_
          * \param[in] point The query point
          * \returns True if the query point lies within outer_shape_ptr_ but not within inner_shape_ptr_.
          */
        bool
        isInside (const PointT &point) const;

        /** \brief Static helper function to flip normals of CloudPtr. inner_shape_ptr_ normals need to be flipped when combining outer and inner shapes.
          * \param[in,out] CloudPtr The cloud which normals get flipped.
          */
        inline static void
        flipNormals (PointCloudT::Ptr CloudPtr)
        {
          for (PointCloudT::iterator itr = CloudPtr->begin (); itr != CloudPtr->end (); ++itr)
          {
            itr->getNormalVector3fMap () *= -1;
          }
        }
    };
  }  // End namespace simulation
}  // End namespace pcl

#endif // PCL_SIMULATION_SHAPE_GENERATOR_COMPLEX_H_
