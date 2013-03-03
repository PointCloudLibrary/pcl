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

#ifndef PCL_MESH_PROCESSING_H_
#define PCL_MESH_PROCESSING_H_

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace pcl
{
  /** \brief @b CloudSurfaceProcessing represents the base class for algorithms that takes a point cloud as input and
    * produces a new output cloud that has been modified towards a better surface representation. These types of
    * algorithms include surface smoothing, hole filling, cloud upsampling etc.
    *
    * \author Alexandru E. Ichim
    * \ingroup surface
    */
  template <typename PointInT, typename PointOutT>
  class CloudSurfaceProcessing : public PCLBase<PointInT>
  {
    public:
      typedef boost::shared_ptr<CloudSurfaceProcessing<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const CloudSurfaceProcessing<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

    public:
      /** \brief Constructor. */
      CloudSurfaceProcessing () : PCLBase<PointInT> ()
      {};
      
      /** \brief Empty destructor */
      virtual ~CloudSurfaceProcessing () {}

      /** \brief Process the input cloud and store the results
        * \param[out] output the cloud where the results will be stored
        */
      virtual void
      process (pcl::PointCloud<PointOutT> &output);

    protected:
      /** \brief Abstract cloud processing method */
      virtual void
      performProcessing (pcl::PointCloud<PointOutT> &output) = 0;

  };


  /** \brief @b MeshProcessing represents the base class for mesh processing algorithms.
    * \author Alexandru E. Ichim
    * \ingroup surface
    */
  class PCL_EXPORTS MeshProcessing
  {
    public:
      typedef boost::shared_ptr<MeshProcessing> Ptr;
      typedef boost::shared_ptr<const MeshProcessing> ConstPtr;

      typedef PolygonMesh::ConstPtr PolygonMeshConstPtr;

      /** \brief Constructor. */
      MeshProcessing () : input_mesh_ () {}

      /** \brief Destructor. */
      virtual ~MeshProcessing () {}

      /** \brief Set the input mesh that we want to process
        * \param[in] input the input polygonal mesh
        */
      inline void
      setInputMesh (const pcl::PolygonMeshConstPtr &input) 
      { input_mesh_ = input; }

      /** \brief Get the input mesh to be processed
        * \returns the mesh
        */
      inline pcl::PolygonMeshConstPtr
      getInputMesh () const
      { return input_mesh_; }

      /** \brief Process the input surface mesh and store the results
        * \param[out] output the resultant processed surface model
        */
      void 
      process (pcl::PolygonMesh &output);

    protected:
      /** \brief Initialize computation. Must be called before processing starts. */
      virtual bool 
      initCompute ();
      
      /** \brief UnInitialize computation. Must be called after processing ends. */
      virtual void 
      deinitCompute ();

      /** \brief Abstract surface processing method. */
      virtual void 
      performProcessing (pcl::PolygonMesh &output) = 0;

      /** \brief Abstract class get name method. */
      virtual std::string 
      getClassName () const
      { return (""); }

      /** \brief Input polygonal mesh. */
      pcl::PolygonMeshConstPtr input_mesh_;
  };
}

#include "pcl/surface/impl/processing.hpp"

#endif  /* PCL_MESH_PROCESSING_H_ */

