/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_ELCH_H_
#define PCL_ELCH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <Eigen/Geometry>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b ELCH (Explicit Loop Closing Heuristic) class
      * \author Jochen Sprickerhof
      * \ingroup registration
      */
    template <typename PointT>
    class ELCH : public PCLBase<PointT>
    {
      public:
        typedef boost::shared_ptr< ELCH<PointT> > Ptr;
        typedef boost::shared_ptr< const ELCH<PointT> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        /** \brief graph structure to hold the SLAM graph */
        typedef boost::adjacency_list<
          boost::vecS, boost::vecS, boost::undirectedS,
          PointCloudPtr,
          boost::no_property>
        LoopGraph;

        typedef typename pcl::Registration<PointT, PointT> Registration;
        typedef typename Registration::Ptr RegistrationPtr;
        typedef typename Registration::ConstPtr RegistrationConstPtr;

        /** \brief Empty constructor. */
        ELCH () : loop_graph_ (), loop_start_ (), loop_end_ (), reg_ (pcl::IterativeClosestPoint<PointT, PointT> ())
        {};

        /** \brief Add a new point cloud to the internal graph.
         * \param[in] cloud the new point cloud
         */
        inline void
        addPointCloud (PointCloudConstPtr cloud)
        {
          add_vertex (loop_graph_, cloud);
        }

        /** \brief Getter for the internal graph. */
        inline LoopGraph
        getLoopGraph ()
        {
          return (loop_graph_);
        }

        /** \brief Setter for a new internal graph.
         * \param[in] loop_graph the new graph
         */
        inline void
        setLoopGraph (LoopGraph loop_graph)
        {
          loop_graph_ = loop_graph;
        }

        /** \brief Getter for the first scan of a loop. */
        inline PointCloudConstPtr
        getLoopStart ()
        {
          return (loop_start_);
        }

        /** \brief Setter for the first scan of a loop.
         * \param[in] loop_start the scan that starts the loop
         */
        inline void
        setLoopStart (PointCloudConstPtr loop_start)
        {
          loop_start_ = loop_start;
        }

        /** \brief Getter for the last scan of a loop. */
        inline PointCloudConstPtr
        getLoopend ()
        {
          return (loop_end_);
        }

        /** \brief Setter for the last scan of a loop.
         * \param[in] loop_end the scan that ends the loop
         */
        inline void
        setLoopend (PointCloudConstPtr loop_end)
        {
          loop_end_ = loop_end;
        }

        /** \brief Getter for the registration algorithm. */
        inline RegistrationConstPtr
        getReg ()
        {
          return (reg_);
        }

        /** \brief Setter for the registration algorithm.
         * \param[in] reg the registration algorithm used to compute the transformation between the start and the end of the loop
         */
        inline void
        setReg (RegistrationConstPtr reg)
        {
          reg_ = reg;
        }

        /** \brief Getter for the transformation between the first and the last scan. */
        inline Eigen::Affine3f &
        getLoopTransform ()
        {
          return (loop_transform_);
        }

        /** \brief Setter for the transformation between the first and the last scan.
         * \param[in] loop_transform the transformation between the first and the last scan
         */
        inline void
        setLoopTransform (Eigen::Affine3f &loop_transform)
        {
          loop_transform_ = loop_transform;
        }

        /** \brief Computes now poses for all point clouds by closing the loop
         * between start and end point cloud. This will transform all given point
         * clouds for now!
         */
        void
        compute ();

      protected:
        using PCLBase<PointT>::deinitCompute;

        /** \brief This method should get called before starting the actual computation. */
        virtual bool
        initCompute ();

      private:
        /** \brief graph structure for the internal optimization graph */
        typedef boost::adjacency_list<
          boost::vecS, boost::vecS, boost::undirectedS,
          boost::no_property,
          boost::property< boost::edge_weight_t, double > >
        LOAGraph;

        /**
         * graph balancer algorithm computes the weights
         * @param[in] g the graph
         * @param[in] f index of the first node
         * @param[in] l index of the last node
         * @param[out] weights array for the weights
         */
        void
        loopOptimizerAlgorithm (LOAGraph &g, int f, int l, double *weights);

        /** \brief The internal loop graph. */
        LoopGraph *loop_graph_;

        /** \brief The first scan of the loop. */
        PointCloudConstPtr loop_start_;

        /** \brief The last scan of the loop. */
        PointCloudConstPtr loop_end_;

        /** \brief The registration object used to close the loop. */
        RegistrationConstPtr reg_;

        /** \brief The transformation between that start and end of the loop. */
        Eigen::Affine3f *loop_transform_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include "pcl/registration/impl/elch.hpp"

#endif // PCL_ELCH_H_
