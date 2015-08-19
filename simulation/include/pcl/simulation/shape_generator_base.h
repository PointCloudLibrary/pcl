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

#ifndef PCL_SIMULATION_SHAPE_GENERATOR_BASE_H_
#define PCL_SIMULATION_SHAPE_GENERATOR_BASE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>

#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
#include <boost/array.hpp>
#include <boost/assign.hpp>
#include <boost/math/special_functions/modf.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <ctime>

namespace pcl
{
  namespace simulation
  {
    /** \brief The abstract base class for all geometric shapes. This class defines basic cloud handling as well as transformations.
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \ingroup simulation
      */
    class GeometricShapeBase
    {
      public:
        friend class CutShape;
        friend class MultiShape;

        typedef pcl::PointXYZLNormal PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<GeometricShapeBase> Ptr;

        /** \brief Predefined rotation axes for use with rotate */
        enum Axis
        {
          XAxis, /**< Rotate around the x-axis */
          YAxis, /**< Rotate around the y-axis */
          ZAxis  /**< Rotate around the z-axis */
        };

        /** \brief Default constructor used to initialize most member variables.*/
        GeometricShapeBase ();

        /** \brief Generate points on the shape surface.
          * \param[in] resolution The density of points per square unit
          * \returns A shared pointer to the generated cloud
          * \note This function is abstract in the base class and needs to be implemented for all child classes.
          */
        virtual PointCloudT::Ptr
        generate (float resolution) = 0;

        /** \brief Center the shape around center.
          * \param[in] new_centroid The point to which the shape will be centered. Default (0, 0, 0)
          * \returns Returns the translation which has been applied to the shape to center it.
          */
        Eigen::Vector3f
        center (const Eigen::Vector3f &new_centroid = Eigen::Vector3f (0, 0, 0));

        /** \brief Translate the shape in a given direction.
          * \param[in] translation The translation vector
          */
        void
        translate (const Eigen::Vector3f &translation);

        /** \brief Translate the shape in a given direction.
          * \param[in] x The x direction of the translation
          * \param[in] y The y direction of the translation
          * \param[in] z The z direction of the translation
          */
        inline void
        translate (float x,
                   float y,
                   float z)
        {
          translate (Eigen::Vector3f (x, y, z));
        }

        /** \brief Rotate Shape around the x, y or z axis.
          * \param[in] axis Select x, y or z  axis
          * \param[in] degree The amount of rotation in degrees
          * \param[in] in_place True: Shape is rotated around its center. False: Shape is rotated around (0, 0, 0). Default true.
          */
        void
        rotate (GeometricShapeBase::Axis axis,
                float degree,
                bool in_place = true);

        /** \brief Rotate Shape around a custom axis.
          * \param[in] axis Custom axis. Does not need to be normalized.
          * \param[in] degree The amount of rotation in degrees
          * \param[in] in_place True: Shape is rotated around its center. False: Shape is rotated around (0, 0, 0). Default true.
          */
        void
        rotate (const Eigen::Vector3f& axis,
                     float degree,
                     bool in_place = true);

        /** \brief Reverse all transformations.*/
        inline void
        reverseTransformations ()
        {
          pcl::transformPoint (centroid_, centroid_, effective_transform_.inverse ());
          effective_transform_.setIdentity ();
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
      protected:
        typedef Eigen::Affine3f EigenTransformT;

        /** \brief Random number generator*/
        boost::random::mt19937 gen_;

        /** \brief Uniform distribution needed for most samplings*/
        boost::random::uniform_real_distribution<float> dist01_;

        /** \brief The effective transformations which have been applied to the shape*/
        EigenTransformT effective_transform_;

        /** \brief Stores the shapes centroid (this is usually the centroid of the bounding box)*/
        Eigen::Vector3f centroid_;

        static uint shape_id_;
        /// Functions
        /** \brief Apply the effective transformations to the pointcloud
          * \param[in,out] cloud_ptr The cloud to transform
          * \note This should be called at the end of the generate method in all child classes. It makes sure, that we apply all transformation to the generated points
          */
        inline void
        applyTransformations (PointCloudT::Ptr cloud_ptr) const
        {
          pcl::transformPointCloudWithNormals (*cloud_ptr, *cloud_ptr, effective_transform_);
        }

        /** \brief Check if a point is inside the shape in the shapes own reference frame.
          * \param[in] point The query point.
          * \returns The query result.
          * \note This function returns true if the Point is inside the object in the objects own reference frame (i.e. the frame in which generate also places points). Applied transformations are handled by the base class.
          */
        virtual bool
        isInside (const PointT &point) const = 0;

        /** \brief Delete all points from the other cloud which are inside or outside this shape.
          * \param[in,out] other_cloud_arg The query pointcloud.
          * \param[in] delete_inliers If true delete inliers, if false delete outliers.
          */
        void
        deletePointsFromOtherCloud (PointCloudT::Ptr other_cloud_arg,
                                    bool delete_inliers = true) const;
        
        /** \brief Calculates the number of points which should be sampled given the area and the resolution.
          * \param[in] area The size of the area we want to sample on
          * \param[in] resolution The resolution in points per square unit for the samplings
          * \returns The number of points which should be sampled on the area.
          */        
        inline unsigned int 
        calculateNumberOfPoints (const float area, 
                                 const float resolution)
        {
          float num_points_float = area * resolution;
          int num_points;
          float frac;
          frac = boost::math::modf (num_points_float , &num_points);
          if (dist01_ (gen_) < frac)
            num_points++;
          return (num_points);
        }
    };
    typedef std::vector<GeometricShapeBase::Ptr> GeometricShapePtrVector;
  }  // End namespace simulation
}  // End namespace pcl

#endif // PCL_SIMULATION_SHAPE_GENERATOR_BASE_H_
