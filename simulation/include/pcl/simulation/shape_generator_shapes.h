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

#ifndef PCL_SIMULATION_SHAPE_GENERATOR_SHAPES_H_
#define PCL_SIMULATION_SHAPE_GENERATOR_SHAPES_H_

#include <pcl/simulation/shape_generator_base.h>

namespace pcl
{
  namespace simulation
  {
    /** \brief  Class which can be used to sample on any polygon given by a set of vertices and faces.
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \note We do check if all vertices in a face a planar and convex, but we do not check if the shape is convex.
      * \note If you provide vertices and faces which belong to a concave polygon the shape can still be sampled but the isInside function will not work properly. This is why you cannot use a concave polygon together with MultiShape or CutShape.
      * \ingroup simulation
      */
    class ConvexPolygon : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<ConvexPolygon> Ptr;
        typedef std::vector<Eigen::Vector3f> VerticeVectorT;
        typedef std::vector<std::vector<unsigned int> > FaceInformationT;

        /** \brief  Standard constructor which requires the call of setPolygons afterwards. */
        ConvexPolygon () :
          faces_set_ (false),
          area_ (0)
        {
        }

        /** \brief  Constructor which already initializes the polygon using setPolygons.
          * \param[in] vertices_vector std::vector of Eigen::Vector3f points which describe the polygon's vertices.
          * \param[in] face_information A vector of a vector of indices which describe how vertices are connected.
          */
        ConvexPolygon (const VerticeVectorT &vertices_vector,
                       const FaceInformationT &face_information) :
          faces_set_ (false),
          area_ (0)
        {
          setPolygons (vertices_vector, face_information);
        }

        /** \brief  Sets the vertices and the faces of the polyon
          * \param[in] vertices_vector std::vector of Eigen::Vector3f points which describe the polygon's vertices.
          * \param[in] face_information A vector of a vector of indices which describe how vertices are connected.
          * \note Normals are pointing in the right-hand grip rule direction of a face (ascending vertex order).
          */
        void
        setPolygons (const VerticeVectorT &vertices_vector,
                     const FaceInformationT &face_information);

        /** \brief  Returns the total area of the polygon
          * \returns The area of the polygon. If faces have not been set, returns 0.
          */
        float
        getPolygonArea ()
        {
          return (area_);
        }

        /** \brief  Generate points on the shape
          * \param[in] resolution points per square unit
          * \returns A shared pointer to the generated cloud
          * \note This function will call the sampleOnFace function of all faces.
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      protected:
        /** \brief  Class which holds the face information of the polygon, like normal, face-area.
          * \author Markus Schoeler (mschoeler@gwdg.de)
          * \note Each face is represented by one or multiple triangles. All points of a face need to lie in a common plane.
          * \note We only copy the indices of the vertices, but not their coordinates, this is why we need to set the parent and make this class friend of ConvexPolygon.
          * \ingroup simulation
          */
        struct Face
        {
          typedef boost::shared_ptr<Face> Ptr;
          ConvexPolygon* parent_;
          std::vector<unsigned int> vertex_list_;
          Eigen::Vector3f normal_;
          std::vector<float> triangle_areas_;
          float face_area_;

          Face (ConvexPolygon* parent) :
            parent_ (parent),
            face_area_ (0)
          {
          }

          /** \brief Samples random points on the face with resolution points per square unit.            
             * \param[in] resolution points per square unit
             * \param[in,out] cloud_ptr the cloud to which we add points
             * \param[in] label the label of the new sampled points (default = 0)
             */
          void
          sampleOnFace (float resolution, 
                        PointCloudT::Ptr cloud_ptr, 
                        int label = 0);

          /** \brief Check if query point lies on the side of the plane which points away from the normals.
             * \param[in] point Query point
             * \returns True if point lies on the side of the plane pointing away from the normal.
             */
          inline bool
          isInside (const PointT &point)
          {
            return (normal_.dot (point.getVector3fMap () - parent_->vertices_[vertex_list_[0]]) < 0);
          }
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };  // END struct Face

        friend class Face;

        /** \brief  A vector with the vertex coordinates */
        VerticeVectorT vertices_;

        /** \brief  A vector with the face pointers. */
        std::vector<Face::Ptr> faces_;

        /** \brief  Is true when faces have been set using setPolygons. */
        bool faces_set_;

        /** \brief  Stores the total area of the object. */
        float area_;

        /** \brief  Checks if query point is inside the shape.
          * \param[in] point Query point
          * \returns True if point lies within the convex polygon
          * \note For a convex polygon this yields true if all Face's isInside function return true. (Conjunction)
          * \note Since this is not valid for polygons with concavities we cannot use this isInside function for concave polygons.
          */
        virtual bool
        isInside (const PointT &point) const;
    };

    /** \brief  Class defining a sphere shape
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Sphere : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<Sphere> Ptr;

        /** \brief  Default constructor for the shape
          * \param[in] radius Sphere's radius
          */
        Sphere (float radius) :
          radius_ (radius)
        {
        }

        /** \brief  Generate points on the shape
          * \param[in] resolution points per square unit
          * \returns A shared pointer to the generated cloud
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
      protected:
        float radius_;

        /** \brief  Checks if query point is inside the shape.
          * \param[in] point Query point
          * \returns True if point lies within the shape.
          */
        virtual bool
        isInside (const PointT &point) const;
    };

    /** \brief  Class defining a cylinder shape
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Cylinder : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<Cylinder> Ptr;

        /** \brief  Default constructor for the shape
          * \param[in] radius Cylinder's radius (x,z- plane)
          * \param[in] height Cylinder's height (y-axis)
          */
        Cylinder (float radius,
                  float height) :
          radius_ (radius),
          height_ (height)
        {
        }

        /** \brief  Generate points on the shape
          * \param[in] resolution points per square unit
          * \returns A shared pointer to the generated cloud
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      protected:
        float radius_, height_;

        /** \brief  Checks if query point is inside the shape.
          * \param[in] point Query point
          * \returns True if point lies within the shape.
          */
        virtual bool
        isInside (const PointT &point) const;

        /** \brief  Add points on the cylinder middle
          * \param[in] resolution points per square unit
          * \param[in,out] cloud_ptr add point to the cloud_ptr
          * \returns A shared pointer to the generated cloud
          */
        void
        addCylinderMiddle (float resolution,
                           PointCloudT::Ptr cloud_ptr);

        /** \brief  Add points on the cylinder top and bottom
          * \param[in] resolution points per square unit
          * \param[in,out] cloud_ptr add point to the cloud_ptr
          * \returns A shared pointer to the generated cloud
          */
        void
        addCylinderTopAndBottom (float resolution, 
                                 PointCloudT::Ptr cloud_ptr);
    };

    /** \brief  Class defining a cone shape
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Cone : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<Cone> Ptr;

        /** \brief  Default constructor for the shape
          * \param[in] radius Cone's radius (x,z- plane)
          * \param[in] height Cone's height (y-axis)
          */
        Cone (float radius,
              float height) :
          radius_ (radius),
          height_ (height)
        {
        }

        /** \brief  Generate points on the shape
          * \param[in] resolution points per square unit
          * \returns A shared pointer to the generated cloud
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      protected:
        float radius_, height_;

        /** \brief  Checks if query point is inside the shape.
          * \param[in] point Query point
          * \returns True if point lies within the shape.
          */
        virtual bool
        isInside (const PointT &point) const;
    };

    /** \brief  Class defining a wedge shape
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Wedge : public ConvexPolygon
    {
      public:
        typedef boost::shared_ptr<Wedge> Ptr;
        /** \brief  Default constructor for the shape
          * \param[in] width Wedges bottom width (x-axis)
          * \param[in] depth Wedges bottom depth (z-axis)
          * \param[in] upwidth Wedges top width (x-axis)
          * \param[in] updepth Wedges top depth (z-axis)
          * \param[in] height Wedges height (y-axis)
          * \note This is a good example how one can implement a shape which inherits it's whole functionality from ConvexPolygon
          */
        Wedge (float width,
               float depth,
               float upwidth,
               float updepth,
               float height);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      protected:
        /** \brief Enumerator for accessing the eight corners */
        enum Corner
        {
          FrontBottomLeft, FrontBottomRight, BackBottomLeft, BackBottomRight, FrontTopLeft, FrontTopRight, BackTopLeft, BackTopRight
        };
    };

    /** \brief  Class defining a cuboid shape (special wedge case)
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Cuboid : public Wedge
    {
      public:
        typedef boost::shared_ptr<Cuboid> Ptr;
        
        /** \brief  Default constructor for the shape
          * \param[in] width Cuboid width (x-axis)
          * \param[in] height Cuboid height (y-axis)
          * \param[in] depth Cuboid depth (z-axis)
          * \note This is just a lightweight wrapper for a special case of a wedge.
          */
        Cuboid (float width,
                float height,
                float depth) :
          Wedge (width, depth, width, depth, height)
        {
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief  Class defining a full as well as partial torus shape
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class Torus : public GeometricShapeBase
    {
      public:
        typedef boost::shared_ptr<Torus> Ptr;

        /** \brief  Constructor for a full torus
          * \param[in] R Torus's outer radius
          * \param[in] r Torus's inner radius
          * \note Creates a full torus in the x,z plane.
          */
        Torus (float R,
               float r);

        /** \brief  Constructor for a partial torus
          * \param[in] R Torus's outer radius
          * \param[in] r Torus's inner radius
          * \param[in] min_theta Start of the torus segment in degrees. 0 is parallel to x-, 90 is parallel to z-axis.
          * \param[in] max_theta End of the torus segment in degrees. 0 is parallel to x-, 90 is parallel to z-axis.
          * \note Creates a partial torus in the x,z plane.
          */
        Torus (float R,
               float r,
               float min_theta,
               float max_theta);

        /** \brief  Generate points on the shape
          * \param[in] resolution points per square unit
          * \returns A shared pointer to the generated cloud
          */
        virtual PointCloudT::Ptr
        generate (float resolution);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
      protected:
        bool is_full_;
        float R_, r_, min_theta_, max_theta_;

        /** \brief  Checks if query point is inside the shape.
          * \param[in] point Query point
          * \returns True if point lies within the shape.
          */
        virtual bool
        isInside (const PointT &point) const;
    };
  }  // End namespace simulation
}  // End namespace pcl

#endif // PCL_SIMULATION_SHAPE_GENERATOR_SHAPES_H_
