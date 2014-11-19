/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2013, Martin Szarski
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

#ifndef PCL_CPPF_H_
#define PCL_CPPF_H_

#include <pcl/features/feature.h>
#include <pcl/features/boost.h>

namespace pcl
{
  /** \brief
    * \param[in] p1 
    * \param[in] n1
    * \param[in] p2 
    * \param[in] n2
    * \param[in] c1
    * \param[in] c2
    * \param[out] f1
    * \param[out] f2
    * \param[out] f3
    * \param[out] f4
    * \param[out] f5
    * \param[out] f6
    * \param[out] f7
    * \param[out] f8
    * \param[out] f9
    * \param[out] f10
    */
  PCL_EXPORTS bool
  computeCPPFPairFeature (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, const Eigen::Vector4i &c1,
                            const Eigen::Vector4f &p2, const Eigen::Vector4f &n2, const Eigen::Vector4i &c2,
                            float &f1, float &f2, float &f3, float &f4, float &f5, float &f6, float &f7, float &f8, float &f9, float &f10);



  /** \brief Class that calculates the "surflet" features for each pair in the given
    * pointcloud. Please refer to the following publication for more details:
    *    C. Choi, Henrik Christensen
    *    3D Pose Estimation of Daily Objects Using an RGB-D Camera
    *    Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
    *    2012
    *
    * PointOutT is meant to be pcl::CPPFSignature - contains the 10 values of the Surflet
    * feature and in addition, alpha_m for the respective pair - optimization proposed by
    * the authors (see above)
    *
    * \author Martin Szarski, Alexandru-Eugen Ichim
    */

  template <typename PointInT, typename PointNT, typename PointOutT>
  class CPPFEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<CPPFEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const CPPFEstimation<PointInT, PointNT, PointOutT> > ConstPtr;
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      /** \brief Empty Constructor. */
      CPPFEstimation ();


    private:
      /** \brief The method called for actually doing the computations
        * \param[out] output the resulting point cloud (which should be of type pcl::CPPFSignature);
        * its size is the size of the input cloud, squared (i.e., one point for each pair in
        * the input cloud);
        */
      void
      computeFeature (PointCloudOut &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/cppf.hpp>
#endif

#endif // PCL_CPPF_H_
