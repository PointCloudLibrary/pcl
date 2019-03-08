/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#pragma once

#include <pcl/pcl_base.h>

namespace pcl
{

  namespace tracking
  {

    /** \brief @b PointCoherence is a base class to compute coherence between the two points.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class PointCoherence
    {
    public:
      typedef boost::shared_ptr< PointCoherence<PointInT> > Ptr;
      typedef boost::shared_ptr< const PointCoherence<PointInT> > ConstPtr;
      
    public:
      /** \brief empty constructor */
      PointCoherence () : coherence_name_ () {}
      
      /** \brief empty distructor */
      virtual ~PointCoherence () {}

      /** \brief compute coherence from the source point to the target point.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      inline double
      compute (PointInT &source, PointInT &target);

    protected:

      /** \brief The coherence name. */
      std::string coherence_name_;

      /** \brief abstract method to calculate coherence.
        * \param[in] source instance of source point.
        * \param[in] target instance of target point.
        */
      virtual double 
      computeCoherence (PointInT &source, PointInT &target) = 0;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (coherence_name_); }

    };

    /** \brief @b PointCloudCoherence is a base class to compute coherence between the two PointClouds.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class PointCloudCoherence
    {
    public:
      typedef boost::shared_ptr< PointCloudCoherence<PointInT> > Ptr;
      typedef boost::shared_ptr< const PointCloudCoherence<PointInT> > ConstPtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      
      typedef typename PointCoherence<PointInT>::Ptr PointCoherencePtr;
      /** \brief Constructor. */
      PointCloudCoherence () : coherence_name_ (), target_input_ (), point_coherences_ () {}

      /** \brief Destructor. */
      virtual ~PointCloudCoherence () {}

      /** \brief compute coherence between two pointclouds. */
      inline void
      compute (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices,
               float &w_i);

      /** \brief get a list of pcl::tracking::PointCoherence.*/
      inline std::vector<PointCoherencePtr>
      getPointCoherences () { return point_coherences_; }

      /** \brief set a list of pcl::tracking::PointCoherence.
        * \param coherences a list of pcl::tracking::PointCoherence.
        */
      inline void
      setPointCoherences (std::vector<PointCoherencePtr> coherences) { point_coherences_ = coherences; }

      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief add a PointCoherence to the PointCloudCoherence.
        * \param coherence a pointer to PointCoherence.
        */
      inline void
      addPointCoherence (PointCoherencePtr coherence) { point_coherences_.push_back (coherence); }

      /** \brief add a PointCoherence to the PointCloudCoherence.
        * \param cloud a pointer to PointCoherence.
        */
      virtual inline void
      setTargetCloud (const PointCloudInConstPtr &cloud)  { target_input_ = cloud; }
      
    protected:
      /** \brief Abstract method to compute coherence. */
      virtual void
      computeCoherence (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w_j) = 0;
      
      inline double calcPointCoherence (PointInT &source, PointInT &target);
      
      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (coherence_name_); }
      
      
      /** \brief The coherence name. */
      std::string coherence_name_;

      /** \brief a pointer to target point cloud*/
      PointCloudInConstPtr target_input_;

      /** \brief a list of pointers to PointCoherence.*/
      std::vector<PointCoherencePtr> point_coherences_;
    };
    
  }
}

#include <pcl/tracking/impl/coherence.hpp>
