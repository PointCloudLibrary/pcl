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
 */

#ifndef PCL_SURFACE_POISSON_H_
#define PCL_SURFACE_POISSON_H_

#include <pcl/surface/reconstruction.h>
#include <string>
#include "pcl/surface/poisson/geometry.h"

namespace pcl
{
  namespace surface
  {
    /** \brief The Poisson surface reconstruction algorithm.
     * \author Gregory Long
     * \ingroup surface
     *
     * TODO: Modify to use PCL's Octree & Marching Cubes code
     */
    template<typename PointNT>
      class Poisson : public SurfaceReconstruction<PointNT>
      {
      public:
        using SurfaceReconstruction<PointNT>::input_;
        using SurfaceReconstruction<PointNT>::tree_;

        typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

        typedef typename pcl::KdTree<PointNT> KdTree;
        typedef typename pcl::KdTree<PointNT>::Ptr KdTreePtr;

        /** \brief Constructor. */
        Poisson ();

        /** \brief Destructor. */
        ~Poisson ();

        /** \brief Create the surface.
         *
         * \param output the resultant polygonal mesh
         */
        void
        performReconstruction (pcl::PolygonMesh &output);

        void
        performReconstruction (pcl::PointCloud<PointNT> &points,
                               std::vector<pcl::Vertices> &polygons)
        {
          PCL_ERROR ("[pcl::surface::Poisson::performReconstruction] Method not implemented!\n");
        }

        inline void
        setNoResetSamples (bool no_reset_samples)
        {
          no_reset_samples_ = no_reset_samples;
        }
        ;

        inline void
        setNoClipTree (bool no_clip_tree)
        {
          no_clip_tree_ = no_clip_tree;
        }
        ;

        inline void
        setConfidence (bool confidence)
        {
          confidence_ = confidence;
        }
        ;

        inline void
        setManifold (bool manifold)
        {
          manifold_ = manifold;
        }
        ;

        inline void
        setDepth (int depth)
        {
          depth_ = depth;
        }
        ;

        inline void
        setSolverDivide (int solver_divide)
        {
          solver_divide_ = solver_divide;
        }
        ;

        inline void
        setIsoDivide (int iso_divide)
        {
          iso_divide_ = iso_divide;
        }
        ;

        inline void
        setRefine (int refine)
        {
          refine_ = refine;
        }
        ;

        inline void
        setKernelDepth (int kernel_depth)
        {
          kernel_depth_ = kernel_depth;
        }
        ;

        inline void
        setSamplesPerNode (float samples_per_node)
        {
          samples_per_node_ = samples_per_node;
        }
        ;

        inline void
        setScale (float scale)
        {
          scale_ = scale;
        }
        ;

      protected:
        /** \brief The point cloud input (XYZ+Normals). */
        PointCloudPtr data_;

        /** \brief Class get name method. */
        std::string
        getClassName () const
        {
          return ("Poisson");
        }

      private:
        bool no_reset_samples_;
        bool no_clip_tree_;
        bool confidence_;
        bool manifold_;

        int depth_;
        int solver_divide_;
        int iso_divide_;
        int refine_;
        int kernel_depth_;

        float samples_per_node_;
        float scale_;

        template<int Degree> void
        execute (PolygonMesh &output);

        void
        writeOutput (PolygonMesh &output, poisson::CoredMeshData* mesh, const poisson::Point3D<float>& translate, const float& scale);

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
  }
}

#endif  // PCL_SURFACE_POISSON_H_
