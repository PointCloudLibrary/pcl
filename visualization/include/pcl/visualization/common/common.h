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
#ifndef PCL_PCL_VISUALIZER_COMMON_H_
#define PCL_PCL_VISUALIZER_COMMON_H_

#if defined __GNUC__
#pragma GCC system_header
#endif

#include <pcl/pcl_macros.h>
#include <pcl/visualization/eigen.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>

namespace pcl
{
  struct RGB;

  namespace visualization
  {
    /** \brief Get (good) random values for R/G/B.
      * \param[out] r the resultant R color value
      * \param[out] g the resultant G color value
      * \param[out] b the resultant B color value
      * \param[in] min minimum value for the colors
      * \param[in] max maximum value for the colors
      */
    PCL_EXPORTS void
    getRandomColors (double &r, double &g, double &b, double min = 0.2, double max = 2.8);

    /** \brief Get (good) random values for R/G/B.
      * \param[out] rgb the resultant RGB color value
      * \param[in] min minimum value for the colors
      * \param[in] max maximum value for the colors
      */
    PCL_EXPORTS void
    getRandomColors (pcl::RGB &rgb, double min = 0.2, double max = 2.8);

    PCL_EXPORTS Eigen::Matrix4d
    vtkToEigen (vtkMatrix4x4* vtk_matrix);

    PCL_EXPORTS Eigen::Vector2i
    worldToView (const Eigen::Vector4d &world_pt, const Eigen::Matrix4d &view_projection_matrix, int width, int height);

    PCL_EXPORTS void
    getViewFrustum (const Eigen::Matrix4d &view_projection_matrix, double planes[24]);

    enum FrustumCull
    {
      PCL_INSIDE_FRUSTUM,
      PCL_INTERSECT_FRUSTUM,
      PCL_OUTSIDE_FRUSTUM
    };

    PCL_EXPORTS int
    cullFrustum (double planes[24], const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb);

    PCL_EXPORTS float
    viewScreenArea (const Eigen::Vector3d &eye, const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const Eigen::Matrix4d &view_projection_matrix, int width, int height);

    /** \brief Set of rendering properties. */
    enum RenderingProperties
    {
      PCL_VISUALIZER_POINT_SIZE,            /**< integer starting from 1 */
      PCL_VISUALIZER_OPACITY,               /**< Float going from 0.0 (transparent) to 1.0 (opaque) */
      PCL_VISUALIZER_LINE_WIDTH,            /**< Integer starting from 1 */
      PCL_VISUALIZER_FONT_SIZE,
      PCL_VISUALIZER_COLOR,                 /**< 3 floats (R, G, B) going from 0.0 (dark) to 1.0 (light) */
      PCL_VISUALIZER_REPRESENTATION,
      PCL_VISUALIZER_IMMEDIATE_RENDERING,
      PCL_VISUALIZER_SHADING,
      PCL_VISUALIZER_LUT,                   /**< colormap type \ref pcl::visualization::LookUpTableRepresentationProperties */
      PCL_VISUALIZER_LUT_RANGE              /**< two doubles (min and max) or \ref pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_RANGE_AUTO */
    };

    enum RenderingRepresentationProperties
    {
      PCL_VISUALIZER_REPRESENTATION_POINTS,
      PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
      PCL_VISUALIZER_REPRESENTATION_SURFACE
    };

    enum ShadingRepresentationProperties
    {
      PCL_VISUALIZER_SHADING_FLAT,
      PCL_VISUALIZER_SHADING_GOURAUD,
      PCL_VISUALIZER_SHADING_PHONG
    };

    /*! Colormap properties. See [mathworks colormap page](http://www.mathworks.com/help/matlab/ref/colormap.html#input_argument_name) for image representations of the colormaps. */
    enum LookUpTableRepresentationProperties
    {
      PCL_VISUALIZER_LUT_JET,           /**< Jet colormap */
      PCL_VISUALIZER_LUT_JET_INVERSE,   /**< Inverse jet colormap */
      PCL_VISUALIZER_LUT_HSV,           /**< HSV colormap */
      PCL_VISUALIZER_LUT_HSV_INVERSE,   /**< Inverse HSV colormap */
      PCL_VISUALIZER_LUT_GREY,          /**< Grey colormap (black to white) */
      PCL_VISUALIZER_LUT_BLUE2RED,      /**< Blue to red colormap (blue to white to red) */
      PCL_VISUALIZER_LUT_RANGE_AUTO     /**< Set LUT range to min and max values of the data */
    };

    /** \brief Generate a lookup table for a colormap.
      * \param[in] colormap_type
      * \param[out] table a vtk lookup table
      * \note The list of available colormaps can be found in \ref pcl::visualization::LookUpTableRepresentationProperties.
      */    
    PCL_EXPORTS bool
    getColormapLUT  (LookUpTableRepresentationProperties colormap_type, vtkSmartPointer<vtkLookupTable> &table);

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Camera class holds a set of camera parameters together with the window pos/size. */
    class PCL_EXPORTS Camera
    {
      public:
        /** \brief Focal point or lookAt.
          * \note The view direction can be obtained by (focal-pos).normalized ()
          */
        double focal[3];

        /** \brief Position of the camera. */
        double pos[3];

        /** \brief Up vector of the camera.
          * \note Not to be confused with the view direction, bad naming here. */
        double view[3];

        /** \brief Clipping planes depths.
          * clip[0] is near clipping plane, and clip [1] is the far clipping plane
          */
        double clip[2];

        /** \brief Field of view angle in y direction (radians). */
        double fovy;

        // the following variables are the actual position and size of the window on the screen and NOT the viewport!
        // except for the size, which is the same the viewport is assumed to be centered and same size as the window.
        double window_size[2];
        double window_pos[2];


        /** \brief Computes View matrix for Camera (Based on gluLookAt)
          * \param[out] view_mat the resultant matrix
          */
        void 
        computeViewMatrix (Eigen::Matrix4d& view_mat) const;

        /** \brief Computes Projection Matrix for Camera
          *  \param[out] proj the resultant matrix
          */
        void 
        computeProjectionMatrix (Eigen::Matrix4d& proj) const;

        /** \brief converts point to window coordiantes
          * \param[in] pt xyz point to be converted
          * \param[out] window_cord vector containing the pts' window X,Y, Z and 1
          *
          * This function computes the projection and view matrix every time.
          * It is very inefficient to use this for every point in the point cloud!
          */
        template<typename PointT> void 
        cvtWindowCoordinates (const PointT& pt, Eigen::Vector4d& window_cord) const;

        /** \brief converts point to window coordiantes
          * \param[in] pt xyz point to be converted
          * \param[out] window_cord vector containing the pts' window X,Y, Z and 1
          * \param[in] composite_mat composite transformation matrix (proj*view)
          *
          * Use this function to compute window coordinates with a precomputed
          * transformation function.  The typical composite matrix will be
          * the projection matrix * the view matrix.  However, additional
          * matrices like a camera disortion matrix can also be added.
          */
        template<typename PointT> void 
        cvtWindowCoordinates (const PointT& pt, Eigen::Vector4d& window_cord, const Eigen::Matrix4d& composite_mat) const;
    };
  }
}

#include <pcl/visualization/common/impl/common.hpp>

#endif
