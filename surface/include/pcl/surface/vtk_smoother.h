/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef VTK_SMOOTHER_H_
#define VTK_SMOOTHER_H_

#include <pcl/pcl_base.h>
#include <pcl/PolygonMesh.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace pcl
{
  namespace surface
  {
    /** \brief VTKSmoother is a wrapper around some subdivision and filter methods from VTK.
      * \author Greg Long, Dirk Holz
      * \ingroup surface
      *
      * TODO: inheritance from some PCLBase-like interface for PolygonMesh
      * TODO: wrap more vtk functionality in here
      * TODO: Do we want to wrap any VTK functionality anyway or are we just going to provide conversion as in mesh2vtk and vtk2mesh?
      */
    class PCL_EXPORTS VTKSmoother
    {
      public:
        /** \brief Default Constructor. */
        VTKSmoother () :
          vtk_polygons_ (vtkPolyData::New ()), subdivision_filter_ (2), num_iter_ (20), feature_angle_ (120.0),
              pass_band_ (0.01)
        {
        };

        /** \brief Constructor. */
        VTKSmoother (int subdivision_filter, int num_iter, float feature_angle, float pass_band) :
          vtk_polygons_ (vtkPolyData::New ()), subdivision_filter_ (subdivision_filter), num_iter_ (num_iter),
              feature_angle_ (feature_angle), pass_band_ (pass_band)
        {
        };

        /** \brief Convert a PCL PolygonMesh to a VTK vtkPolyData.
          * \param[in] triangles PolygonMesh to be converted to vtkPolyData, stored in the object.
          */
        int
        convertToVTK (const pcl::PolygonMesh &triangles);

        /** \brief Subdivision using a given filter. TODO: make ENUM for all implemented filters. */
        void
        subdivideMesh ();

        /** \brief Perform smoothing on the mesh using the windowed sinc algorithm. */
        void
        smoothMeshWindowedSinc ();

        /** \brief Perform smoothing on the mesh using the laplacian algorithm. */
        void
        smoothMeshLaplacian ();

        /** \brief Convert the vtkPolyData object back to PolygonMesh.
          * \param[out] triangles the PolygonMesh to store the vtkPolyData in.
          */
        void
        convertToPCL (pcl::PolygonMesh &triangles);

        /** \brief Set the subdivision filter.
          * \param[in] subdivision_filter the number of the desired subdivision filter.
          */
        inline void
        setSubdivisionFilter (int subdivision_filter)
        {
          subdivision_filter_ = subdivision_filter;
        };

        /** \brief Set the number of iterations for the smoothing filter.
          * \param[in] num_iter the number of iterations
          */
        inline void
        setNumIter (int num_iter)
        {
          num_iter_ = num_iter;
        };

        /** \brief Set the feature angle for sharp edge identification.
          * \param[in] feature_angle the feature angle.
          */
        inline void
        setFeatureAngle (float feature_angle)
        {
          feature_angle_ = feature_angle;
        };

        /** \brief Set the pass band value for windowed sinc filtering.
         * \param[in] pass_band value for the pass band.
         */
        inline void
        setPassBand (float pass_band)
        {
          pass_band_ = pass_band;
        };

        /** \brief Get the subdivision filter number. */
        inline int
        getSubdivisionFilter ()
        {
          return subdivision_filter_;
        };

        /** \brief Get the number of iterations. */
        inline int
        getNumIter ()
        {
          return num_iter_;
        };

        /** \brief Get the feature angle. */
        inline float
        getFeatureAngle ()
        {
          return feature_angle_;
        };

        /** \brief get the pass band value. */
        inline float
        getPassBand ()
        {
          return pass_band_;
        };

      private:

        /** \brief Convert vtkPolyData object to a PCL PolygonMesh
          * \param[in] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
          * \param[out] mesh PCL Polygon Mesh to fill
          * \return Number of points in the point cloud of mesh.
          */
        int 
        vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh);

        /** \brief Convert a PCL PolygonMesh to a vtkPolyData object
          * \param[in] mesh Reference to PCL Polygon Mesh
          * \param[out] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
          * \return Number of points in the point cloud of mesh.
          */
        int 
        mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData> &poly_data);

        vtkSmartPointer<vtkPolyData> vtk_polygons_;

        /** \brief the subdivision filter number.  0 corresponds to no subdivision, 1 to linear
          * subdivision, 2 to loop subdivision, and 3 to butterfly.
          */
        int subdivision_filter_;

        /** \brief the number of iterations for the smoothing filter.  For the windowed sinc filter,
          * a typical number is 10-20.  For Laplacian, 100-200 is typical.
          */
        int num_iter_;

        /** \brief The angle that defines a sharp interior edge. */
        float feature_angle_;

        /** \brief The value for the pass band, used in windowed sinc smoothing, ranging from 0 to 2.
          * Lower values cause more smoothing.  A typical value is 0.1.
          */
        float pass_band_;
    };
  }
}

#endif /* VTK_SMOOTHER_H_ */

