/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: pfh.hpp 5027 2012-03-12 03:10:45Z rusu $
 *
 */
#ifndef PCL_ESF_H_
#define PCL_ESF_H_

#include <pcl/features/feature.h>
#define GRIDSIZE 64
#define GRIDSIZE_H GRIDSIZE/2
#include <vector>

namespace pcl
{
  /** \brief @b ESFEstimation estimates the ensemble of shape functions descriptors for a given point cloud
    * dataset containing points. Shape functions are D2, D3, A3.  For more information about the ESF descriptor, see:
    * Walter Wohlkinger and Markus Vincze, "Ensemble of Shape Functions for 3D Object Classification", 
    * IEEE International Conference on Robotics and Biomimetics (IEEE-ROBIO), 2011
    * \author Walter Wohlkinger
    * \ingroup features
    */

  template <typename PointInT,  typename PointOutT = pcl::ESFSignature640>
  class ESFEstimation: public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<ESFEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const ESFEstimation<PointInT, PointOutT> > ConstPtr;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;

      typedef typename pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      ESFEstimation () : lut_ (), local_cloud_ ()
      {
        feature_name_ = "ESFEstimation";
        lut_.resize (GRIDSIZE);
        for (int i = 0; i < GRIDSIZE; ++i)
        {
          lut_[i].resize (GRIDSIZE);
          for (int j = 0; j < GRIDSIZE; ++j)
            lut_[i][j].resize (GRIDSIZE);
        }
        //lut_.resize (boost::extents[GRIDSIZE][GRIDSIZE][GRIDSIZE]);
        search_radius_ = 0;
        k_ = 5;
      }

      /** \brief Overloaded computed method from pcl::Feature.
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut &output);

    protected:

      /** \brief Estimate the Ensebmel of Shape Function (ESF) descriptors at a set of points given by
        * <setInputCloud (),
        * \param output the resultant point cloud model histogram that contains the ESF feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief ... */
      int
      lci (const int x1, const int y1, const int z1, 
           const int x2, const int y2, const int z2, 
           float &ratio, int &incnt, int &pointcount);
     
      /** \brief ... */
      void
      computeESF (PointCloudIn &pc, std::vector<float> &hist);
      
      /** \brief ... */
      void
      voxelize9 (PointCloudIn &cluster);
      
      /** \brief ... */
      void
      cleanup9 (PointCloudIn &cluster);

      /** \brief ... */
      void
      scale_points_unit_sphere (const pcl::PointCloud<PointInT> &pc, float scalefactor, Eigen::Vector4f& centroid);

    private:

      /** \brief ... */
      std::vector<std::vector<std::vector<int> > > lut_;
      
      /** \brief ... */
      PointCloudIn local_cloud_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/esf.hpp>
#endif

#endif // #
