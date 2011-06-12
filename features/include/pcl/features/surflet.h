/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru Ichim
 *                      Willow Garage, Inc
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

#ifndef PCL_SURFLET_H_
#define PCL_SURFLET_H_

#include <pcl/features/feature.h>
#include <boost/unordered_map.hpp>

namespace pcl
{
  /** \brief Estimate 3D Surflet features.
    *
    * paper...
    *
    * \author Alexandru Ichim
    */
  template <typename PointInT, typename PointOutT>
  class SurfletEstimation : public Feature<PointInT, PointOutT>
  {
    public:
      typedef typename PointCloud<PointInT>::ConstPtr PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /// slight hack to enable the usage of the boost::hash <pair <A, B> >
      struct HashKeyStruct : public std::pair <int, std::pair <int, std::pair <int, int> > > 
      {
        HashKeyStruct(int a, int b, int c, int d)
        {
          this->first = a;
          this->second.first = b;
          this->second.second.first = c;
          this->second.second.second = d;
        }
      };

      typedef boost::unordered_multimap<HashKeyStruct, std::pair<size_t, size_t> > FeatureHashMapType;
      typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;


      SurfletEstimation (float a_angle_discretization_step = 12.0 / 180 * M_PI, 
                         float a_distance_discretization_step = 0.01)
      {
        angle_discretization_step = a_angle_discretization_step;
        distance_discretization_step = a_distance_discretization_step;
      }

      FeatureHashMapTypePtr
      computeSurfletModel (const pcl::PointCloud<PointInT> &cloud /* output goes here */);

      std::vector < std::pair <Eigen::Affine3f, float> >
      registerModelToScene (const pcl::PointCloud<PointInT> &cloud_model, 
                            const pcl::PointCloud<PointOutT> &cloud_model_normals, 
                            const pcl::PointCloud<PointInT> &cloud_scene, FeatureHashMapTypePtr feature_hashmap_model);


      protected:
        void
        computeFeature (PointCloudOut &output);

      private:
        float angle_discretization_step, distance_discretization_step;
  };
}

#endif /* PCL_SURFLET_H_ */
